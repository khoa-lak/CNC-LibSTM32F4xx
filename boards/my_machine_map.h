/*
  =============================================================================
  my_machine_map.h - Sơ đồ gán chân GPIO tùy chỉnh cho STM32F407VET6
  Part of grblHAL
  =============================================================================
  HƯỚNG DẪN REVIEW & CHỈNH SỬA:
  - File này được kích hoạt thông qua macro `BOARD_MY_MACHINE` trong `Inc/my_machine.h`.
  - Bạn có thể tùy chỉnh Port (GPIOA..GPIOE) và Pin (0..15) tương ứng với mạch PCB của bạn.
  - Chế độ `GPIO_BITBAND` cho phép các chân Step, Dir, Limit được đặt rải rác ở bất kỳ 
    chân nào trên MCU mà không bị gò bó vào cùng 1 Port.
  =============================================================================
*/

#ifndef _MY_MACHINE_MAP_H_
#define _MY_MACHINE_MAP_H_

#define BOARD_NAME "STM32F407VET6 Custom CNC"

// -----------------------------------------------------------------------------
// 1. TẦN SỐ THẠCH ANH NGOÀI (HSE Crystal Frequency)
// -----------------------------------------------------------------------------
// Nếu board dùng thạch anh 8MHz  -> để 8000000 (8 triệu)
// Nếu board dùng thạch anh 25MHz -> đổi thành 25000000 (25 triệu)
#ifndef HSE_VALUE
#define HSE_VALUE 8000000
#endif

// -----------------------------------------------------------------------------
// 2. CHẾ ĐỘ XUẤT / NHẬP GPIO (Bitband Mode)
// -----------------------------------------------------------------------------
// GPIO_BITBAND: Cho phép định nghĩa từng chân độc lập (không cần chung PORT).
#define STEP_OUTMODE            GPIO_BITBAND
#define DIRECTION_OUTMODE       GPIO_BITBAND
#define LIMIT_INMODE            GPIO_BITBAND

// =============================================================================
// 3. CÁC CHÂN ĐIỀU KHIỂN ĐỘNG CƠ BƯỚC (STEP / DIRECTION / ENABLE)
// =============================================================================
// --- Trục X ---
#define X_STEP_PORT             GPIOD
#define X_STEP_PIN              11
#define X_DIRECTION_PORT        GPIOE
#define X_DIRECTION_PIN         4

// --- Trục Y ---
#define Y_STEP_PORT             GPIOD
#define Y_STEP_PIN              12
#define Y_DIRECTION_PORT        GPIOE
#define Y_DIRECTION_PIN         5

// --- Trục Z ---
#define Z_STEP_PORT             GPIOD
#define Z_STEP_PIN              13
#define Z_DIRECTION_PORT        GPIOE
#define Z_DIRECTION_PIN         6

// --- (Tùy chọn) Trục thứ 4 (Trục A / M3) ---
#if N_ABC_MOTORS >= 1
#define M3_AVAILABLE
#define M3_STEP_PORT            GPIOD
#define M3_STEP_PIN             14
#define M3_DIRECTION_PORT       GPIOE
#define M3_DIRECTION_PIN        0
#endif

// --- Chân Enable chung cho Driver động cơ (Bật/Tắt nguồn cuộn dây động cơ) ---
#define STEPPERS_ENABLE_PORT    GPIOD
#define STEPPERS_ENABLE_PIN     4

// =============================================================================
// 4. CÔNG TẮC HÀNH TRÌNH CÁC TRỤC (LIMIT SWITCHES)
// =============================================================================
#define LIMIT_PORT              GPIOA
#define X_LIMIT_PORT            GPIOA
#define X_LIMIT_PIN             4
#define Y_LIMIT_PORT            GPIOA
#define Y_LIMIT_PIN             5
#define Z_LIMIT_PORT            GPIOA
#define Z_LIMIT_PIN             6

// =============================================================================
// 5. ĐIỀU KHIỂN TRỤC CHÍNH (SPINDLE)
// =============================================================================
// AUXOUTPUT0: PWM điều khiển tốc độ quay Spindle (Lệnh S...)
#define AUXOUTPUT0_PORT         GPIOB
#define AUXOUTPUT0_PIN          0

// AUXOUTPUT1: Chân chọn chiều quay Spindle (Lệnh M3 thuận / M4 ngược)
#define AUXOUTPUT1_PORT         GPIOC
#define AUXOUTPUT1_PIN          6

// AUXOUTPUT2: Chân kích hoạt Spindle Enable (Lệnh M3 bật / M5 tắt)
#define AUXOUTPUT2_PORT         GPIOC
#define AUXOUTPUT2_PIN          7

// AUXOUTPUT3: Bơm nước tưới nguội Flood (Lệnh M8 bật / M9 tắt)
#define AUXOUTPUT3_PORT         GPIOD
#define AUXOUTPUT3_PIN          8

// AUXOUTPUT4: Van khí / Phun sương Mist (Lệnh M7 bật / M9 tắt)
#define AUXOUTPUT4_PORT         GPIOD
#define AUXOUTPUT4_PIN          9

// Gán các chân trên vào hệ thống điều khiển Spindle của grblHAL
#if DRIVER_SPINDLE_ENABLE & SPINDLE_ENA
#define SPINDLE_ENABLE_PORT     AUXOUTPUT2_PORT
#define SPINDLE_ENABLE_PIN      AUXOUTPUT2_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
#define SPINDLE_PWM_PORT        AUXOUTPUT0_PORT
#define SPINDLE_PWM_PIN         AUXOUTPUT0_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
#define SPINDLE_DIRECTION_PORT  AUXOUTPUT1_PORT
#define SPINDLE_DIRECTION_PIN   AUXOUTPUT1_PIN
#endif

// =============================================================================
// 6. NƯỚC TƯỚI NGUỘI / KHÍ LÀM MÁT (COOLANT)
// =============================================================================
#if COOLANT_ENABLE & COOLANT_FLOOD
#define COOLANT_FLOOD_PORT      AUXOUTPUT3_PORT
#define COOLANT_FLOOD_PIN       AUXOUTPUT3_PIN
#endif
#if COOLANT_ENABLE & COOLANT_MIST
#define COOLANT_MIST_PORT       AUXOUTPUT4_PORT
#define COOLANT_MIST_PIN        AUXOUTPUT4_PIN
#endif

// =============================================================================
// 7. CÁC NÚT BẤM ĐIỀU KHIỂN & ĐẦU VÀO PHỤ (CONTROL INPUTS & AUX INPUTS)
// =============================================================================
// AUXINPUT0: Cảm biến cửa an toàn (Safety Door)
#define AUXINPUT0_PORT          GPIOB
#define AUXINPUT0_PIN           8

// AUXINPUT1: Cảm biến dò dao Probe
#define AUXINPUT1_PORT          GPIOA
#define AUXINPUT1_PIN           7

// AUXINPUT2: Nút dừng khẩn cấp / Reset (E-Stop)
#define AUXINPUT2_PORT          GPIOC
#define AUXINPUT2_PIN           0

// AUXINPUT3: Nút tạm dừng gia công (Feed Hold)
#define AUXINPUT3_PORT          GPIOB
#define AUXINPUT3_PIN           7

// AUXINPUT4: Nút tiếp tục gia công (Cycle Start)
#define AUXINPUT4_PORT          GPIOB
#define AUXINPUT4_PIN           1

// AUXINPUT5: Cảm biến đo dao tự động cố định (Toolsetter)
#define AUXINPUT5_PORT          GPIOB
#define AUXINPUT5_PIN           12

// AUXINPUT6: Cảm biến báo lỗi Driver động cơ (Motor Fault)
#define AUXINPUT6_PORT          GPIOB
#define AUXINPUT6_PIN           13

// AUXINPUT7: Công tắc mở khóa giới hạn hành trình (Limits Override)
#define AUXINPUT7_PORT          GPIOB
#define AUXINPUT7_PIN           14

#if CONTROL_ENABLE & CONTROL_HALT
#define RESET_PORT              AUXINPUT2_PORT
#define RESET_PIN               AUXINPUT2_PIN
#endif
#if CONTROL_ENABLE & CONTROL_FEED_HOLD
#define FEED_HOLD_PORT          AUXINPUT3_PORT
#define FEED_HOLD_PIN           AUXINPUT3_PIN
#endif
#if CONTROL_ENABLE & CONTROL_CYCLE_START
#define CYCLE_START_PORT        AUXINPUT4_PORT
#define CYCLE_START_PIN         AUXINPUT4_PIN
#endif

#if PROBE_ENABLE
#define PROBE_PORT              AUXINPUT1_PORT
#define PROBE_PIN               AUXINPUT1_PIN
#endif

#if TOOLSETTER_ENABLE
#define TOOLSETTER_PORT         AUXINPUT5_PORT
#define TOOLSETTER_PIN          AUXINPUT5_PIN
#endif

#if SAFETY_DOOR_ENABLE
#define SAFETY_DOOR_PORT        AUXINPUT0_PORT
#define SAFETY_DOOR_PIN         AUXINPUT0_PIN
#endif

#if MOTOR_FAULT_ENABLE
#define MOTOR_FAULT_PORT        AUXINPUT6_PORT
#define MOTOR_FAULT_PIN         AUXINPUT6_PIN
#endif

#if LIMITS_OVERRIDE_ENABLE
#define LIMITS_OVERRIDE_PORT    AUXINPUT7_PORT
#define LIMITS_OVERRIDE_PIN     AUXINPUT7_PIN
#endif

// =============================================================================
// 8. CÁC CHÂN ĐIỀU KHIỂN KHÍ NÉN CHO THAY DAO TỰ ĐỘNG (ATC PNEUMATICS)
// =============================================================================
#ifndef ATC_ENABLE
#define ATC_ENABLE              1       // 1: Kích hoạt module thay dao tự động ATC
#endif

#define ATC_UNCLAMP_PORT        GPIOB   // Van khí nén nhả dao (1: Mở ngàm, 0: Kẹp)
#define ATC_UNCLAMP_PIN         10

#define ATC_AIR_BLAST_PORT      GPIOB   // Van khí thổi sạch côn dao Spindle
#define ATC_AIR_BLAST_PIN       11

#endif // _MY_MACHINE_MAP_H_

