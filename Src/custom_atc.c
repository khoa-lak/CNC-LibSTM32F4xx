/*
  =============================================================================
  plugins/custom_atc.c - Module Thay Dao Tự Động Ổ Cố Định (Rack ATC)
  Part of grblHAL for STM32F4xx
  =============================================================================
  NGUYÊN LÝ HOẠT ĐỘNG:
  1. Tọa độ các ổ dao được tính toán tự động dựa trên vị trí ổ số 1 và khoảng cách giữa các ổ.
  2. Tại vị trí XY của ô dao, Spindle sẽ dịch chuyển vào/ra ngàm kẹp theo hướng trượt an toàn (Slide Offset).
  3. Khi gắp dao mới, hệ thống tự động kích hoạt bù trừ chiều dài dao (TLO - Tool Length Offset)
     theo bảng khai báo của từng ổ mà không cần chạm cọc Probe/Toolsetter.
  
  TÀI LIỆU KẾ THỪA TỪ CORE GỐC grblHAL:
  - [grbl/plugins_init.h]: Gọi hàm khởi tạo `atc_init()` khi bật cờ #define ATC_ENABLE 1.
  - [grbl/hal.h]: Giao diện trừu tượng phần cứng `hal.tool.change`, `hal.tool.select`, `hal.tool.atc_get_state`.
  - [grbl/motion_control.h]: Hàm sinh quỹ đạo nội suy chuyển động `mc_line()`.
  - [grbl/protocol.h]: Hàm đồng bộ hóa hàng đợi đệm chuyển động `protocol_buffer_synchronize()`.
  - [grbl/system.h]: Hàm đọc và chuyển đổi tọa độ máy `system_convert_array_steps_to_mpos()`.
  - [grbl/gcode.h]: Hàm bù trừ chiều dài dao động `gc_set_tool_offset()`.
  - [grbl/spindle_control.h]: Hàm điều khiển và khôi phục trạng thái Spindle `spindle_all_off()`, `spindle_restore()`.
  - [grbl/coolant_control.h]: Hàm khôi phục trạng thái nước tưới nguội `coolant_restore()`.
  =============================================================================
*/

#include "driver.h"

#if ATC_ENABLE

#include <math.h>
#include <string.h>

// [KẾ THỪA CORE]: Khai báo các API tiêu chuẩn của grblHAL
#include "grbl/hal.h"             // Cung cấp con trỏ hàm hal.tool, hal.delay_ms
#include "grbl/motion_control.h"  // Cung cấp hàm mc_line() và struct plan_line_data_t
#include "grbl/protocol.h"        // Cung cấp hàm protocol_buffer_synchronize()
#include "grbl/gcode.h"           // Cung cấp hàm gc_set_tool_offset() và enum ToolLengthOffset
#include "grbl/state_machine.h"   // Cung cấp hàm quản lý trạng thái máy CNC
#include "grbl/system.h"          // Cung cấp hàm chuyển đổi tọa độ bước xung sang mm
#include "grbl/spindle_control.h" // Cung cấp hàm điều khiển dừng/bật lại Spindle
#include "grbl/coolant_control.h" // Cung cấp hàm khôi phục nước làm mát

// =============================================================================
// CẤU HÌNH THÔNG SỐ HÌNH HỌC VÀ CƠ KHÍ KHAY DAO (RACK CONFIGURATION)
// =============================================================================

// Tổng số lượng ổ dao có trên khay
#define ATC_NUM_POCKETS         6

// Cao độ Z an toàn tuyệt đối khi di chuyển qua lại (Tọa độ máy G53)
#define ATC_Z_CLEAR             0.0f        // Thường là điểm cao nhất Z = 0 (Home)

// Cao độ Z khi Spindle hạ xuống ôm khớp côn dao / nhả dao vào ngàm
#define ATC_Z_DROP              -110.0f     // Tọa độ Z khi kẹp/nhả dao

// Tọa độ (X, Y) của Ổ DAO SỐ 1 (Pocket 1)
#define POCKET_1_X              30.0f
#define POCKET_1_Y              260.0f

// Khoảng cách giữa các ổ dao liên tiếp (Bước nhảy)
// Ví dụ: Xếp thành hàng ngang dọc theo trục X -> DELTA_X = 50.0mm, DELTA_Y = 0.0mm
#define POCKET_DELTA_X          50.0f       // Khoảng cách giữa 2 ổ dao theo trục X
#define POCKET_DELTA_Y          0.0f        // Khoảng cách giữa 2 ổ dao theo trục Y (nếu xếp theo Y)

// Khoảng dịch trượt (Slide Offset) để vào/ra ngàm kẹp dao tránh va chạm
// Ví dụ: Ngàm kẹp mở ra phía trước (trục Y) -> Trượt lùi 40mm theo Y để thoát dao
#define SLIDE_OFFSET_X          0.0f        // Độ lệch trượt theo trục X (+/- mm)
#define SLIDE_OFFSET_Y          -40.0f      // Độ lệch trượt theo trục Y (+/- mm)

// Tốc độ chạy vào/ra ngàm giữ dao (mm/min)
#define FEED_RATE_DOCK          1200.0f

// Thời gian trễ chờ xi lanh khí nén đóng/mở (milliseconds)
#define ATC_VALVE_DELAY_MS      500

// =============================================================================
// BẢNG KHAI BÁO CHIỀU DÀI DAO CHO TỪNG Ổ (TOOL LENGTH OFFSETS - mm)
// =============================================================================
// Giá trị âm (-) nếu dao ngắn hơn dao số 1, giá trị dương (+) nếu dài hơn dao số 1.
static const float tool_length_table[ATC_NUM_POCKETS] = {
    0.0f,       // Ổ 1: Dao chuẩn (Reference Tool, Offset = 0.0mm)
    -5.25f,     // Ổ 2: Dao phay ngón (Ngắn hơn dao 1 là 5.25mm)
    8.40f,      // Ổ 3: Mũi khoan dài (Dài hơn dao 1 là 8.40mm)
    -1.10f,     // Ổ 4: Dao vát mép
    0.0f,       // Ổ 5: Dao dự phòng
    0.0f        // Ổ 6: Dao dự phòng
};

// =============================================================================
// BIẾN NỘI BỘ QUẢN LÝ TRẠNG THÁI DAO
// =============================================================================
static tool_id_t current_tool_id = 0;     // Dao đang kẹp trên Spindle (0: Chưa kẹp dao)
static tool_data_t *selected_tool = NULL; // Dao được chọn từ lệnh T...

// -----------------------------------------------------------------------------
// [TỰ VIẾT]: Hàm tính toán tọa độ vật lý của ổ dao thứ `pocket_idx` (1-indexed)
// -----------------------------------------------------------------------------
static inline void calculate_pocket_position (uint8_t pocket_idx, float *x_out, float *y_out)
{
    uint8_t idx = pocket_idx - 1;
    *x_out = POCKET_1_X + (float)idx * POCKET_DELTA_X;
    *y_out = POCKET_1_Y + (float)idx * POCKET_DELTA_Y;
}

// -----------------------------------------------------------------------------
// [TỰ VIẾT]: Điều khiển van khí nén kẹp / nhả dao và thổi khí vệ sinh côn
// - Kế thừa: hal.delay_ms() từ grblHAL để trì hoãn non-blocking / hardware timer.
// -----------------------------------------------------------------------------
static void atc_set_drawbar (bool unclamp)
{
#ifdef ATC_UNCLAMP_PORT
    DIGITAL_OUT(ATC_UNCLAMP_PORT, ATC_UNCLAMP_PIN, unclamp);
#endif

#ifdef ATC_AIR_BLAST_PORT
    // Tự động bật khí thổi làm sạch côn khi mở ngàm nhả dao
    DIGITAL_OUT(ATC_AIR_BLAST_PORT, ATC_AIR_BLAST_PIN, unclamp);
#endif

    // [KẾ THỪA CORE]: Hàm trễ mili-giây chuẩn của grblHAL
    hal.delay_ms(ATC_VALVE_DELAY_MS, NULL);
}

// -----------------------------------------------------------------------------
// [KẾ THỪA CORE]: Handler chọn dao được Core gọi khi phân tích lệnh `T<n>`
// - Được gán vào con trỏ hal.tool.select trong hàm atc_init().
// -----------------------------------------------------------------------------
static void atc_on_tool_select (tool_data_t *tool, bool next)
{
    selected_tool = tool;
    if (!next && tool) {
        current_tool_id = tool->tool_id;
    }
}

// -----------------------------------------------------------------------------
// [KẾ THỪA CORE & TỰ VIẾT]: Chu trình thay dao tự động khi nhận lệnh `M6`
// - Được gán vào con trỏ hal.tool.change trong hàm atc_init().
// - Sử dụng các hàm API cốt lõi của grblHAL:
//   + mc_line(): Chạy nội suy quỹ đạo tuyến tính.
//   + protocol_buffer_synchronize(): Chờ chuyển động vật lý kết thúc hoàn toàn.
//   + gc_set_tool_offset(): Áp dụng bù trừ chiều dài dao động (G43.1).
//   + spindle_all_off() / spindle_restore(): Dừng và khôi phục Spindle an toàn.
//   + coolant_restore(): Khôi phục nước tưới nguội.
// -----------------------------------------------------------------------------
static status_code_t atc_execute_tool_change (parser_state_t *gc_state)
{
    if (selected_tool == NULL)
        return Status_GCodeToolError;

    uint8_t new_t = selected_tool->tool_id;
    uint8_t old_t = current_tool_id;

    // 1. Nếu dao yêu cầu trùng dao đang kẹp -> Trả về Status_OK ngay lập tức
    if (new_t == old_t && new_t != 0)
        return Status_OK;

    // Kiểm tra số hiệu dao có nằm trong phạm vi khay dao không
    if (new_t > ATC_NUM_POCKETS)
        return Status_GCodeToolError;

    // [KẾ THỪA CORE]: Khởi tạo cấu trúc dữ liệu lệnh chạy đường thẳng mc_line
    plan_line_data_t pl_data;
    plan_data_init(&pl_data);
    float target[N_AXIS];
    coord_data_t prev_work_pos;

    // [KẾ THỪA CORE]: Lưu lại toạ độ máy trước khi di chuyển vào ổ thay dao
    system_convert_array_steps_to_mpos(prev_work_pos.values, sys.position);

    // [KẾ THỪA CORE]: Tắt Spindle và Coolant an toàn trước khi di chuyển
    spindle_all_off(false);
    hal.coolant.set_state((coolant_state_t){0});

    // [KẾ THỪA CORE]: Nhấc trục Z lên cao độ an toàn tối đa (Z_CLEAR)
    system_convert_array_steps_to_mpos(target, sys.position);
    target[Z_AXIS] = ATC_Z_CLEAR;
    pl_data.condition.rapid_motion = On; // Chạy nhanh G0
    mc_line(target, &pl_data);
    protocol_buffer_synchronize();       // Ép CPU chờ chạy xong trục Z

    float pocket_x, pocket_y;

    // =========================================================================
    // 5. TRẢ DAO CŨ VỀ Ổ CŨ (NẾU ĐANG CÓ DAO TRÊN TRỤC CHÍNH)
    // =========================================================================
    if (old_t >= 1 && old_t <= ATC_NUM_POCKETS) {
        // [TỰ VIẾT]: Tính toán tọa độ ô dao cũ
        calculate_pocket_position(old_t, &pocket_x, &pocket_y);

        // A. Chạy nhanh XY đến vị trí tiếp cận (vị trí chờ bên ngoài ngàm kẹp)
        target[X_AXIS] = pocket_x + SLIDE_OFFSET_X;
        target[Y_AXIS] = pocket_y + SLIDE_OFFSET_Y;
        pl_data.condition.rapid_motion = On;
        mc_line(target, &pl_data);
        protocol_buffer_synchronize();

        // B. Hạ Z xuống cao độ kẹp ngàm
        target[Z_AXIS] = ATC_Z_DROP;
        pl_data.condition.rapid_motion = Off;
        pl_data.feed_rate = FEED_RATE_DOCK; // Tốc độ trượt an toàn F1200
        mc_line(target, &pl_data);
        protocol_buffer_synchronize();

        // C. Trượt XY thẳng vào tâm ngàm kẹp giữ dao của ổ
        target[X_AXIS] = pocket_x;
        target[Y_AXIS] = pocket_y;
        mc_line(target, &pl_data);
        protocol_buffer_synchronize();

        // D. [TỰ VIẾT]: Mở ngàm khí nén để nhả dao cũ vào khay
        atc_set_drawbar(true);

        // E. Nhấc thẳng Z lên vị trí an toàn (dao cũ ở lại khay)
        pl_data.condition.rapid_motion = On;
        target[Z_AXIS] = ATC_Z_CLEAR;
        mc_line(target, &pl_data);
        protocol_buffer_synchronize();

        current_tool_id = 0;
    }

    // =========================================================================
    // 6. LẤY DAO MỚI TỪ Ổ MỚI (NẾU YÊU CẦU DAO MỚI > 0)
    // =========================================================================
    if (new_t >= 1 && new_t <= ATC_NUM_POCKETS) {
        // [TỰ VIẾT]: Tính toán tọa độ ô dao mới
        calculate_pocket_position(new_t, &pocket_x, &pocket_y);

        // A. Mở sẵn ngàm khí nén (để sẵn sàng ôm côn dao)
        atc_set_drawbar(true);

        // B. Chạy nhanh XY đến thẳng đỉnh ô dao mới
        target[X_AXIS] = pocket_x;
        target[Y_AXIS] = pocket_y;
        pl_data.condition.rapid_motion = On;
        mc_line(target, &pl_data);
        protocol_buffer_synchronize();

        // C. Hạ Z xuống ôm chặt côn dao mới
        target[Z_AXIS] = ATC_Z_DROP;
        pl_data.condition.rapid_motion = Off;
        pl_data.feed_rate = FEED_RATE_DOCK;
        mc_line(target, &pl_data);
        protocol_buffer_synchronize();

        // D. [TỰ VIẾT]: Đóng ngàm kẹp chặt dao
        atc_set_drawbar(false);

        // E. Trượt XY ra vị trí tiếp cận để rút dao ra khỏi ngàm giữ
        target[X_AXIS] = pocket_x + SLIDE_OFFSET_X;
        target[Y_AXIS] = pocket_y + SLIDE_OFFSET_Y;
        mc_line(target, &pl_data);
        protocol_buffer_synchronize();

        // F. Nhấc Z lên vị trí an toàn tối đa
        pl_data.condition.rapid_motion = On;
        target[Z_AXIS] = ATC_Z_CLEAR;
        mc_line(target, &pl_data);
        protocol_buffer_synchronize();

        current_tool_id = new_t;

        // =====================================================================
        // 7. [KẾ THỪA CORE]: TỰ ĐỘNG BÙ CHIỀU DÀI DAO (TLO - G43.1)
        // - Dùng API gc_set_tool_offset() của parser G-Code trong grbl/gcode.c
        // =====================================================================
        float offset = tool_length_table[new_t - 1];
        gc_set_tool_offset(ToolLengthOffset_EnableDynamic, Z_AXIS, offset);
    } else if (new_t == 0) {
        // Nếu lệnh M6 T0 (chỉ cất dao vào khay) -> Hủy bù dao (G49)
        gc_set_tool_offset(ToolLengthOffset_Cancel, Z_AXIS, 0.0f);
    }

    // =========================================================================
    // 8. [KẾ THỪA CORE]: KHÔI PHỤC VỊ TRÍ GIA CÔNG CŨ & KHỞI ĐỘNG LẠI SPINDLE
    // =========================================================================
    // A. Chạy XY về lại tọa độ trước khi thay dao
    target[X_AXIS] = prev_work_pos.values[X_AXIS];
    target[Y_AXIS] = prev_work_pos.values[Y_AXIS];
    pl_data.condition.rapid_motion = On;
    mc_line(target, &pl_data);
    protocol_buffer_synchronize();

    // B. [KẾ THỪA CORE]: Bật lại nước làm mát và Spindle (có tự động chờ delay đạt RPM)
    coolant_restore(gc_state->modal.coolant, settings.coolant.on_delay);
    spindle_t *spindle = gc_spindle_get(-1);
    spindle_restore(spindle->hal, spindle->state, spindle->rpm, settings.spindle.on_delay);

    // C. Hạ Z xuống lại cao độ cắt gia công trước đó (đã áp dụng bù chiều dài dao mới)
    target[Z_AXIS] = prev_work_pos.values[Z_AXIS];
    mc_line(target, &pl_data);
    protocol_buffer_synchronize();

    return Status_OK;
}

// -----------------------------------------------------------------------------
// [KẾ THỪA CORE]: Báo cho Core biết module ATC đang Online (sẵn sàng hoạt động)
// - Giúp grbl/tool_change.c nhận diện và nhường quyền xử lý M6 cho module này.
// -----------------------------------------------------------------------------
static atc_status_t atc_get_status (void)
{
    return ATC_Online;
}

// =============================================================================
// [KẾ THỪA CORE]: HÀM KHỞI TẠO VÀ ĐĂNG KÝ MODULE VỚI HAL
// - Tên hàm `atc_init` là chuẩn của grblHAL, được tự động gọi từ grbl/plugins_init.h
// =============================================================================
void atc_init (void)
{
    // Cấu hình chân GPIO điều khiển van khí nén là Output
#ifdef ATC_UNCLAMP_PORT
    GPIO_InitTypeDef GPIO_InitStruct = {
        .Pin = 1 << ATC_UNCLAMP_PIN,
        .Mode = GPIO_MODE_OUTPUT_PP,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_LOW
    };
    HAL_GPIO_Init(ATC_UNCLAMP_PORT, &GPIO_InitStruct);
    DIGITAL_OUT(ATC_UNCLAMP_PORT, ATC_UNCLAMP_PIN, 0); // Mặc định Kẹp dao (0)
#endif

#ifdef ATC_AIR_BLAST_PORT
    GPIO_InitTypeDef GPIO_AirStruct = {
        .Pin = 1 << ATC_AIR_BLAST_PIN,
        .Mode = GPIO_MODE_OUTPUT_PP,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_LOW
    };
    HAL_GPIO_Init(ATC_AIR_BLAST_PORT, &GPIO_AirStruct);
    DIGITAL_OUT(ATC_AIR_BLAST_PORT, ATC_AIR_BLAST_PIN, 0); // Mặc định tắt thổi khí
#endif

    // [KẾ THỪA CORE]: Gán các hàm xử lý vào hệ thống HAL của grblHAL
    hal.driver_cap.atc = On;
    hal.tool.change = atc_execute_tool_change;
    hal.tool.select = atc_on_tool_select;
    hal.tool.atc_get_state = atc_get_status;
}

#endif // ATC_ENABLE
