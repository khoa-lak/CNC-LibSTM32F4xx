/*
  =============================================================================
  CHƯƠNG TRÌNH KIỂM TRA TOÀN DIỆN CƠ CẤU MÁY CNC (STM32F407VET6 CUSTOM CNC)
  Thư mục: testing/code_test/code_test.ino
  Tác giả: CNC Controller Team
  Baudrate: 115200 bps
  =============================================================================
  SƠ ĐỒ CHÂN (CẤU HÌNH TRỰC TIẾP TRONG FILE NÀY):
  - ĐỘNG CƠ BƯỚC (MỖI DRIVER CÓ CHÂN ENABLE RIÊNG BIỆT):
    + Trục X: Step PE5, Dir PE3, Enable PC13
    + Trục Y: Step PE4, Dir PE2, Enable PE6
    + Trục Z: Step PC1, Dir PC0, Enable PC2
    + Trục A: Step PD14, Dir PE0, Enable PD7
    * Mức kích hoạt Driver: LOW kích hoạt (Active LOW đối với TB6600, DM542, A4988...)
  - CÔNG TẮC HÀNH TRÌNH (LIMIT SWITCHES):
    + X Limit: PA0 (Input Pull-up, chạm = LOW)
    + Y Limit: PA1 (Input Pull-up, chạm = LOW)
    + Z Limit: PA2 (Input Pull-up, chạm = LOW)
  - TRỤC CHÍNH (SPINDLE):
    + Spindle PWM: PB0 (0 - 255)
    + Spindle DIR: PC6 (CW: 0, CCW: 1)
    + Spindle ENA: PC7 (1: Bật, 0: Tắt)
  - TƯỚI NGUỘI & KHÍ LÀM MÁT (COOLANT & MIST):
    + Flood Coolant (Bơm tưới nguội M8): PD8
    + Mist Coolant (Khí làm mát M7): PD9
  - THAY DAO TỰ ĐỘNG (ATC PNEUMATICS):
    + ATC Unclamp (Van nhả ngàm kẹp dao): PB10
    + ATC Air Blast (Van thổi khí côn Spindle): PB11
  - CẢM BIẾN & NÚT NHẤN ĐIỀU KHIỂN (INPUTS):
    + Nút dừng khẩn cấp (E-Stop/Reset): PE11
    + Nút tạm dừng (Feed Hold): PB7
    + Nút bắt đầu (Cycle Start): PB1
    + Cảm biến cửa (Safety Door): PB8
    + Cảm biến dò dao (Probe): PA7
  =============================================================================
*/

#include <Arduino.h>
#include <math.h>

// =============================================================================
// 1. ĐỊNH NGHĨA CHÂN PHẦN CỨNG (GPIO PIN DEFINITIONS)
// =============================================================================

// --- Động cơ bước (Step / Dir / Enable riêng từng trục) ---
#define PIN_STEP_Y PE5
#define PIN_DIR_Y PE3
#define PIN_ENABLE_Y PC13  // Chân Enable riêng cho Driver trục X

#define PIN_STEP_X PE4
#define PIN_DIR_X PE2
#define PIN_ENABLE_X PE6   // Chân Enable riêng cho Driver trục Y

#define PIN_STEP_Z PC1
#define PIN_DIR_Z PC0
#define PIN_ENABLE_Z PC2   // Chân Enable riêng cho Driver trục Z

#define PIN_STEP_A PD14
#define PIN_DIR_A PE0
#define PIN_ENABLE_A PD7   // Chân Enable riêng cho Driver trục A (Trục 4)

// Mức kích hoạt Driver (LOW = Enable giữ lực, HIGH = Disable thả trôi)
#define DRIVER_ENABLE_ACTIVE LOW
#define DRIVER_DISABLE_ACTIVE HIGH

// --- Công tắc hành trình ---
#define PIN_LIMIT_X PA0
#define PIN_LIMIT_Y PA1
#define PIN_LIMIT_Z PA2

// Mức logic khi chạm công tắc (chạm công tắc nối Mass GND -> LOW)
#define LIMIT_TRIGGERED_LEVEL LOW

// --- Trục chính (Spindle) ---
#define PIN_SPINDLE_PWM PB0
#define PIN_SPINDLE_DIR PC6
#define PIN_SPINDLE_ENABLE PC7

// --- Nước tưới nguội & Khí làm mát ---
#define PIN_COOLANT_FLOOD PD8
#define PIN_COOLANT_MIST PD9

// --- Khí nén thay dao tự động ATC ---
#define PIN_ATC_UNCLAMP PB10
#define PIN_ATC_AIR_BLAST PB11

// --- Cảm biến & Nút bấm ---
#define PIN_INPUT_ESTOP PE11
#define PIN_INPUT_FEED_HOLD PB7
#define PIN_INPUT_CYCLE_START PB1
#define PIN_INPUT_SAFETY_DOOR PB8
#define PIN_INPUT_PROBE PA7

// --- LED báo trạng thái bo mạch ---
#if defined(LED_BUILTIN)
#define PIN_STATUS_LED LED_BUILTIN
#elif defined(PA6)
#define PIN_STATUS_LED PA6
#else
#define PIN_STATUS_LED PC13
#endif

// =============================================================================
// 2. CẤU HÌNH THÔNG SỐ CƠ KHÍ & KÍCH THƯỚC CỨNG (HARDCODED DIMENSIONS & CONFIG)
// =============================================================================

// --- Tỉ lệ bước trên mm (Steps per mm) ---
#define STEPS_PER_MM_X          80.0f    // Số bước cho 1mm trục X
#define STEPS_PER_MM_Y          80.0f    // Số bước cho 1mm trục Y
#define STEPS_PER_MM_Z          400.0f   // Số bước cho 1mm trục Z (Vít me bước nhỏ)
#define STEPS_PER_MM_A          80.0f    // Số bước cho 1 đơn vị trục A

// --- Quy ước mức logic cho chiều chuyển động DƯƠNG (+) ---
#define DIR_POS_X               LOW     // Mức logic để X chạy theo chiều Dương (+)
#define DIR_POS_Y               HIGH     // Mức logic để Y chạy theo chiều Dương (+)
#define DIR_POS_Z               LOW     // Mức logic để Z chạy theo chiều Dương (+)
#define DIR_POS_A               HIGH     // Mức logic để A chạy theo chiều Dương (+)

// --- Cấu hình Homing (Về gốc máy) ---
#define HOMING_DIR_X            -1       // Hướng Homing X: -1 (Về X- Min), +1 (Về X+ Max)
#define HOMING_DIR_Y            -1       // Hướng Homing Y: -1 (Về Y- Min), +1 (Về Y+ Max)
#define HOMING_DIR_Z             1       // Hướng Homing Z: +1 (Nhấc dao lên Z+ Max an toàn)

#define HOMING_SEARCH_FEED_XY   1000.0f  // Tốc độ dò nhanh công tắc X/Y (mm/phút)
#define HOMING_SEARCH_FEED_Z    400.0f   // Tốc độ dò nhanh công tắc Z (mm/phút)
#define HOMING_LATCH_FEED_XY    150.0f   // Tốc độ rà chậm xác định điểm switch X/Y (mm/phút)
#define HOMING_LATCH_FEED_Z     80.0f    // Tốc độ rà chậm xác định điểm switch Z (mm/phút)

#define HOMING_PULLOFF_MM       3.0f     // Khoảng cách lùi ra khỏi công tắc sau khi chạm (mm)
#define HOMING_MAX_TRAVEL_X     300.0f   // Hành trình dò tối đa trục X (mm) trước khi báo lỗi kẹt
#define HOMING_MAX_TRAVEL_Y     300.0f   // Hành trình dò tối đa trục Y (mm) trước khi báo lỗi kẹt
#define HOMING_MAX_TRAVEL_Z     120.0f   // Hành trình dò tối đa trục Z (mm) trước khi báo lỗi kẹt

// --- Vị trí và độ cao an toàn khi chạy biên dạng ---
#define SHAPE_START_X_MM        15.0f    // Vị trí gốc X bắt đầu chạy các biên dạng (mm)
#define SHAPE_START_Y_MM        15.0f    // Vị trí gốc Y bắt đầu chạy các biên dạng (mm)
#define SHAPE_SAFE_Z_MM          5.0f    // Độ cao Z an toàn khi di chuyển không gia công (mm)
#define SHAPE_WORK_Z_MM          0.0f    // Độ cao Z làm việc / mô phỏng cắt bề mặt (mm)
#define RAPID_FEEDRATE          1500.0f  // Tốc độ di chuyển nhanh G0 (mm/phút)

// --- 1. Kích thước Hình Vuông (Square) ---
#define SQUARE_SIZE_MM          40.0f    // Kích thước cạnh hình vuông (40mm x 40mm)
#define SQUARE_FEEDRATE         1200.0f  // Tốc độ chạy hình vuông (mm/phút)

// --- 2. Kích thước Hình Tam Giác Đều (Equilateral Triangle) ---
#define TRIANGLE_SIDE_MM        40.0f    // Chiều dài cạnh tam giác đều (40mm)
#define TRIANGLE_FEEDRATE       1200.0f  // Tốc độ chạy hình tam giác (mm/phút)

// --- 3. Kích thước Hình Tròn (Circle) ---
#define CIRCLE_RADIUS_MM        20.0f    // Bán kính hình tròn (Đường kính 40mm)
#define CIRCLE_SEGMENTS         72       // Số đoạn nội suy (72 đoạn = 5 độ/bước)
#define CIRCLE_FEEDRATE         1000.0f  // Tốc độ chạy hình tròn (mm/phút)

// --- 4. Kích thước Biên dạng 3D Xoắn ốc (3D Helical Spiral Contour) ---
#define PROFILE_3D_RADIUS_MM    20.0f    // Bán kính xoắn ốc 3D (Đường kính 40mm)
#define PROFILE_3D_DEPTH_MM      6.0f    // Tổng chiều sâu trục Z ăn dốc xuống (6mm)
#define PROFILE_3D_TURNS            3    // Số vòng xoắn ốc ăn sâu xuống (3 vòng)
#define PROFILE_3D_FEEDRATE     800.0f   // Tốc độ chạy biên dạng 3D (mm/phút)

// =============================================================================
// 3. BIẾN TOÀN CỤC & TRẠNG THÁI HỆ THỐNG
// =============================================================================
bool enable_x_state = true;
bool enable_y_state = true;
bool enable_z_state = true;
bool enable_a_state = true;

bool spindle_running = false;
bool spindle_dir_ccw = false;
uint8_t spindle_speed = 0;
bool flood_active = false;
bool mist_active = false;
bool atc_unclamp_on = false;
bool atc_air_blast_on = false;

// Cấu hình Jogging bàn phím
uint16_t jog_steps = 400;
uint16_t jog_delay_us = 400;

// Tọa độ hiện tại của máy (Đơn vị: mm)
float cur_pos_x = 0.0f;
float cur_pos_y = 0.0f;
float cur_pos_z = 0.0f;
bool is_machine_homed = false;

// =============================================================================
// 4. NGUYÊN MẪU HÀM (FUNCTION PROTOTYPES)
// =============================================================================
void print_header();
void print_main_menu();
void handle_main_menu(char cmd);

void set_axis_enable(char axis, bool enable);
void set_all_axes_enable(bool enable);
void print_steppers_status();

void menu_steppers();
void step_single_axis(uint32_t stepPin, uint32_t dirPin, bool dir, uint32_t steps, uint16_t delayUs);
void menu_jogging();

void menu_spindle();
void set_spindle(bool enable, bool dir_ccw, uint8_t pwm_val);

void menu_coolant();
void menu_atc();
void test_atc_full_cycle();

void monitor_sensors_live();
void run_auto_self_test();

// --- Homing & Multi-Axis Motion Prototypes ---
bool home_single_axis(char axis, uint32_t stepPin, uint32_t dirPin, uint32_t limitPin,
                      int homingDir, bool dirPosLevel, float stepsPerMm,
                      float searchFeed, float latchFeed, float pulloffMm, float maxTravelMm);
bool run_homing_sequence();
void menu_homing();

bool move_to_linear(float target_x, float target_y, float target_z, float feedrate_mm_min);
void run_shape_square();
void run_shape_triangle();
void run_shape_circle();
void run_shape_3d_helix();
void run_full_sequence_test();
void menu_shapes();

char read_char_blocking();
void flush_serial_input();

// =============================================================================
// 5. HÀM KHỞI TẠO HỆ THỐNG (SETUP)
// =============================================================================
void setup() {
  Serial.begin(115200);

  // Chờ Serial kết nối ổn định (đặc biệt khi dùng USB CDC)
  uint32_t start_wait = millis();
  while (!Serial && (millis() - start_wait < 2500)) {
    delay(10);
  }

  // --- Cấu hình các chân OUTPUT cho Động cơ bước ---
  pinMode(PIN_STEP_X, OUTPUT);
  pinMode(PIN_DIR_X, OUTPUT);
  pinMode(PIN_ENABLE_X, OUTPUT);

  pinMode(PIN_STEP_Y, OUTPUT);
  pinMode(PIN_DIR_Y, OUTPUT);
  pinMode(PIN_ENABLE_Y, OUTPUT);

  pinMode(PIN_STEP_Z, OUTPUT);
  pinMode(PIN_DIR_Z, OUTPUT);
  pinMode(PIN_ENABLE_Z, OUTPUT);

  pinMode(PIN_STEP_A, OUTPUT);
  pinMode(PIN_DIR_A, OUTPUT);
  pinMode(PIN_ENABLE_A, OUTPUT);

  // Mặc định: Giữ chân STEP ở mức LOW, kích hoạt toàn bộ 4 driver
  digitalWrite(PIN_STEP_X, LOW);
  digitalWrite(PIN_STEP_Y, LOW);
  digitalWrite(PIN_STEP_Z, LOW);
  digitalWrite(PIN_STEP_A, LOW);
  digitalWrite(PIN_DIR_X, LOW);
  digitalWrite(PIN_DIR_Y, LOW);
  digitalWrite(PIN_DIR_Z, LOW);
  digitalWrite(PIN_DIR_A, LOW);

  set_all_axes_enable(true);

  // --- Cấu hình các chân OUTPUT cho Spindle ---
  pinMode(PIN_SPINDLE_PWM, OUTPUT);
  pinMode(PIN_SPINDLE_DIR, OUTPUT);
  pinMode(PIN_SPINDLE_ENABLE, OUTPUT);
  analogWrite(PIN_SPINDLE_PWM, 0);
  digitalWrite(PIN_SPINDLE_DIR, LOW);
  digitalWrite(PIN_SPINDLE_ENABLE, LOW);

  // --- Cấu hình các chân OUTPUT cho Coolant & Mist ---
  pinMode(PIN_COOLANT_FLOOD, OUTPUT);
  pinMode(PIN_COOLANT_MIST, OUTPUT);
  digitalWrite(PIN_COOLANT_FLOOD, LOW);
  digitalWrite(PIN_COOLANT_MIST, LOW);

  // --- Cấu hình các chân OUTPUT cho ATC Pneumatics ---
  pinMode(PIN_ATC_UNCLAMP, OUTPUT);
  pinMode(PIN_ATC_AIR_BLAST, OUTPUT);
  digitalWrite(PIN_ATC_UNCLAMP, LOW);
  digitalWrite(PIN_ATC_AIR_BLAST, LOW);

  // --- Cấu hình các chân INPUT cho Cảm biến & Công tắc (INPUT_PULLUP) ---
  pinMode(PIN_LIMIT_X, INPUT_PULLUP);
  pinMode(PIN_LIMIT_Y, INPUT_PULLUP);
  pinMode(PIN_LIMIT_Z, INPUT_PULLUP);
  pinMode(PIN_INPUT_ESTOP, INPUT_PULLUP);
  pinMode(PIN_INPUT_FEED_HOLD, INPUT_PULLUP);
  pinMode(PIN_INPUT_CYCLE_START, INPUT_PULLUP);
  pinMode(PIN_INPUT_SAFETY_DOOR, INPUT_PULLUP);
  pinMode(PIN_INPUT_PROBE, INPUT_PULLUP);

  // Cấu hình LED hiển thị
  pinMode(PIN_STATUS_LED, OUTPUT);
  digitalWrite(PIN_STATUS_LED, HIGH);

  delay(200);
  print_header();
  print_main_menu();
}

// =============================================================================
// 6. ĐIỀU KHIỂN CHÂN ENABLE TỪNG TRỤC RIÊNG BIỆT
// =============================================================================
void set_axis_enable(char axis, bool enable) {
  uint8_t level = enable ? DRIVER_ENABLE_ACTIVE : DRIVER_DISABLE_ACTIVE;
  switch (axis) {
    case 'X':
    case 'x':
      enable_x_state = enable;
      digitalWrite(PIN_ENABLE_X, level);
      break;
    case 'Y':
    case 'y':
      enable_y_state = enable;
      digitalWrite(PIN_ENABLE_Y, level);
      break;
    case 'Z':
    case 'z':
      enable_z_state = enable;
      digitalWrite(PIN_ENABLE_Z, level);
      break;
    case 'A':
    case 'a':
      enable_a_state = enable;
      digitalWrite(PIN_ENABLE_A, level);
      break;
  }
}

void set_all_axes_enable(bool enable) {
  set_axis_enable('X', enable);
  set_axis_enable('Y', enable);
  set_axis_enable('Z', enable);
  set_axis_enable('A', enable);
}

void print_steppers_status() {
  Serial.print(F("[Trang thai Enable Driver -> "));
  Serial.print(F("X: "));
  Serial.print(enable_x_state ? F("ON ") : F("OFF"));
  Serial.print(F(" | Y: "));
  Serial.print(enable_y_state ? F("ON ") : F("OFF"));
  Serial.print(F(" | Z: "));
  Serial.print(enable_z_state ? F("ON ") : F("OFF"));
  Serial.print(F(" | A: "));
  Serial.print(enable_a_state ? F("ON ") : F("OFF"));
  Serial.println(F("]"));
}

// =============================================================================
// 7. VÒNG LẶP CHÍNH (LOOP)
// =============================================================================
void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd != '\r' && cmd != '\n' && cmd != ' ') {
      handle_main_menu(cmd);
    }
  }

  // Nhấp nháy LED nhẹ để biết chip MCU đang chạy bình thường
  static uint32_t last_blink = 0;
  if (millis() - last_blink >= 500) {
    last_blink = millis();
    digitalWrite(PIN_STATUS_LED, !digitalRead(PIN_STATUS_LED));
  }
}

// =============================================================================
// 8. GIAO DIỆN MENU CHÍNH (SERIAL CLI)
// =============================================================================
void print_header() {
  Serial.println(F("\r\n============================================================"));
  Serial.println(F("     CNC MACHINE MECHANISM TEST SUITE - STM32F407VET6       "));
  Serial.println(F("   Bo mach: STM32F407VET6 Custom CNC (Tung Driver Enable Rieng)"));
  Serial.println(F("============================================================"));
}

void print_main_menu() {
  Serial.println(F("\r\n--- MENU CHUC NANG KIEM TRA ---"));
  Serial.println(F(" [1] Test Dong co buoc (Steppers X, Y, Z, A & Enable rieng tung truc)"));
  Serial.println(F(" [2] Che do Jogging ban phim (Thu cong W/A/S/D/Q/E/Z/C)"));
  Serial.println(F(" [3] Test Truc chinh Spindle (Enable, Chieu DIR, PWM 0-100%)"));
  Serial.println(F(" [4] Test Tuoi nguoi & Lam mat (Coolant Flood & Mist)"));
  Serial.println(F(" [5] Test Khi nen Thay dao tu dong ATC (Unclamp & Air Blast)"));
  Serial.println(F(" [6] Theo doi Cam bien & Cong tac hanh trinh (Live Monitor)"));
  Serial.println(F(" [7] Chay Tu Dong Kiem Tra Toan Dien (Auto Self-Test)"));
  Serial.println(F(" [8] Test Quy trinh Ve Home (Homing Test Z -> X -> Y)"));
  Serial.println(F(" [9] Test Chay Bien Dang: Tron, Vuong, Tam Giac, 3D Helix"));
  Serial.println(F(" [0] In lai Menu nay"));
  Serial.print(F(">> Nhap lua chon cua ban: "));
}

void handle_main_menu(char cmd) {
  switch (cmd) {
    case '1':
      menu_steppers();
      print_main_menu();
      break;
    case '2':
      menu_jogging();
      print_main_menu();
      break;
    case '3':
      menu_spindle();
      print_main_menu();
      break;
    case '4':
      menu_coolant();
      print_main_menu();
      break;
    case '5':
      menu_atc();
      print_main_menu();
      break;
    case '6':
      monitor_sensors_live();
      print_main_menu();
      break;
    case '7':
      run_auto_self_test();
      print_main_menu();
      break;
    case '8':
      menu_homing();
      print_main_menu();
      break;
    case '9':
      menu_shapes();
      print_main_menu();
      break;
    case '0':
    case 'm':
    case 'M':
      print_header();
      print_main_menu();
      break;
    default:
      Serial.print(F("\r\n[!] Lenh khong hop le: '"));
      Serial.print(cmd);
      Serial.println(F("'. Nhap '0' de xem lai Menu."));
      break;
  }
}

// =============================================================================
// 9. MODULE 1: KIỂM TRA ĐỘNG CƠ BƯỚC & ENABLE RIÊNG (STEPPER MOTORS)
// =============================================================================
void step_single_axis(uint32_t stepPin, uint32_t dirPin, bool dir, uint32_t steps, uint16_t delayUs) {
  digitalWrite(dirPin, dir ? HIGH : LOW);
  delayMicroseconds(20);

  for (uint32_t i = 0; i < steps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(5);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(delayUs);
  }
}

void menu_steppers() {
  Serial.println(F("\r\n============================================================"));
  Serial.println(F("         KIEM TRA DONG CO BUOC & ENABLE RIENG TUNG TRUC     "));
  Serial.println(F("============================================================"));
  Serial.println(F(" DIEU KHIEN ENABLE RIENG TUNG DRIVER:"));
  Serial.println(F("   [1] Bat/Tat Enable Trục X (PIN_ENABLE_X)"));
  Serial.println(F("   [2] Bat/Tat Enable Trục Y (PIN_ENABLE_Y)"));
  Serial.println(F("   [3] Bat/Tat Enable Trục Z (PIN_ENABLE_Z)"));
  Serial.println(F("   [4] Bat/Tat Enable Trục A (PIN_ENABLE_A)"));
  Serial.println(F("   [E] Bat/Tat Enable TAT CA cac truc cung luc"));
  Serial.println(F(" TEST CHAY PHAT XUNG TUNG TRUC:"));
  Serial.println(F("   [X] Test Truc X - Quay 800 buoc Thuan/Nghich"));
  Serial.println(F("   [Y] Test Truc Y - Quay 800 buoc Thuan/Nghich"));
  Serial.println(F("   [Z] Test Truc Z - Quay 800 buoc Len/Xuong"));
  Serial.println(F("   [A] Test Truc A - Quay 800 buoc Thuan/Nghich"));
  Serial.println(F("   [S] Test Quet Tan So Xung (Tang toc tu 1kHz -> 5kHz tren truc X)"));
  Serial.println(F("   [0] Quay lai Menu chinh"));

  while (true) {
    Serial.println();
    print_steppers_status();
    Serial.print(F("[Steppers] Nhap lenh (1..4/E/X/Y/Z/A/S/0): "));
    char c = read_char_blocking();

    if (c == '0' || c == 'q' || c == 'Q') {
      Serial.println(F("-> Thoat ve Menu chinh."));
      break;
    }

    switch (c) {
      case '1':
        set_axis_enable('X', !enable_x_state);
        Serial.print(F("-> Enable Driver X: "));
        Serial.println(enable_x_state ? F("ENABLED (Giu luc)") : F("DISABLED (Tha tu do)"));
        break;

      case '2':
        set_axis_enable('Y', !enable_y_state);
        Serial.print(F("-> Enable Driver Y: "));
        Serial.println(enable_y_state ? F("ENABLED (Giu luc)") : F("DISABLED (Tha tu do)"));
        break;

      case '3':
        set_axis_enable('Z', !enable_z_state);
        Serial.print(F("-> Enable Driver Z: "));
        Serial.println(enable_z_state ? F("ENABLED (Giu luc)") : F("DISABLED (Tha tu do)"));
        break;

      case '4':
        set_axis_enable('A', !enable_a_state);
        Serial.print(F("-> Enable Driver A: "));
        Serial.println(enable_a_state ? F("ENABLED (Giu luc)") : F("DISABLED (Tha tu do)"));
        break;

      case 'e':
      case 'E':
        {
          bool new_state = !(enable_x_state && enable_y_state && enable_z_state && enable_a_state);
          set_all_axes_enable(new_state);
          Serial.print(F("-> Da chuyen tat ca Driver Enable thanh: "));
          Serial.println(new_state ? F("ENABLED (Giu luc)") : F("DISABLED (Tha tu do)"));
        }
        break;

      case 'x':
      case 'X':
        set_axis_enable('X', true);
        Serial.println(F("-> Dang test Truc X: Quay THUAN (800 steps)..."));
        step_single_axis(PIN_STEP_X, PIN_DIR_X, HIGH, 800, 500);
        delay(300);
        Serial.println(F("-> Dang test Truc X: Quay NGHICH (800 steps)..."));
        step_single_axis(PIN_STEP_X, PIN_DIR_X, LOW, 800, 500);
        Serial.println(F("-> Hoan thanh test Truc X."));
        break;

      case 'y':
      case 'Y':
        set_axis_enable('Y', true);
        Serial.println(F("-> Dang test Truc Y: Quay THUAN (800 steps)..."));
        step_single_axis(PIN_STEP_Y, PIN_DIR_Y, HIGH, 800, 500);
        delay(300);
        Serial.println(F("-> Dang test Truc Y: Quay NGHICH (800 steps)..."));
        step_single_axis(PIN_STEP_Y, PIN_DIR_Y, LOW, 800, 500);
        Serial.println(F("-> Hoan thanh test Truc Y."));
        break;

      case 'z':
      case 'Z':
        set_axis_enable('Z', true);
        Serial.println(F("-> Dang test Truc Z: Di chuyen LEN (800 steps)..."));
        step_single_axis(PIN_STEP_Z, PIN_DIR_Z, HIGH, 800, 500);
        delay(300);
        Serial.println(F("-> Dang test Truc Z: Di chuyen XUONG (800 steps)..."));
        step_single_axis(PIN_STEP_Z, PIN_DIR_Z, LOW, 800, 500);
        Serial.println(F("-> Hoan thanh test Truc Z."));
        break;

      case 'a':
      case 'A':
        set_axis_enable('A', true);
        Serial.println(F("-> Dang test Truc A: Quay THUAN (800 steps)..."));
        step_single_axis(PIN_STEP_A, PIN_DIR_A, HIGH, 800, 500);
        delay(300);
        Serial.println(F("-> Dang test Truc A: Quay NGHICH (800 steps)..."));
        step_single_axis(PIN_STEP_A, PIN_DIR_A, LOW, 800, 500);
        Serial.println(F("-> Hoan thanh test Truc A."));
        break;

      case 's':
      case 'S':
        set_axis_enable('X', true);
        Serial.println(F("-> Dang test Quet Tan So Xung tren Truc X (Ramp up delay 1000us -> 200us)..."));
        digitalWrite(PIN_DIR_X, HIGH);
        for (uint16_t d = 1000; d >= 200; d -= 20) {
          for (int i = 0; i < 40; i++) {
            digitalWrite(PIN_STEP_X, HIGH);
            delayMicroseconds(5);
            digitalWrite(PIN_STEP_X, LOW);
            delayMicroseconds(d);
          }
        }
        Serial.println(F("-> Quet tan so xong."));
        break;

      default:
        Serial.println(F("[!] Lenh khong hop le."));
        break;
    }
  }
}

// =============================================================================
// 10. MODULE 2: CHẾ ĐỘ JOGGING THỦ CÔNG (KEYBOARD JOGGING)
// =============================================================================
void menu_jogging() {
  Serial.println(F("\r\n============================================================"));
  Serial.println(F("              CHE DO JOGGING BAN PHIM THU CONG              "));
  Serial.println(F("============================================================"));
  Serial.println(F(" Huong dan pham vi phim:"));
  Serial.println(F("   [A] / [D] : Trục X (A: X- Sang Trai | D: X+ Sang Phai)"));
  Serial.println(F("   [W] / [S] : Trục Y (W: Y+ Tien Len  | S: Y- Lui Xuong)"));
  Serial.println(F("   [Q] / [E] : Trục Z (Q: Z+ Nhac Len  | E: Z- Ha Xuong)"));
  Serial.println(F("   [Z] / [C] : Trục A (Z: A- Quay Trai | C: A+ Quay Phai)"));
  Serial.println(F("   [+] / [-] : Tang / Giam so buoc moi lan nhan phim"));
  Serial.println(F("   [F] / [G] : Tang toc do (Giam delay) / Giam toc do"));
  Serial.println(F("   [0] / [X] : Thoat khoi che do Jogging"));
  Serial.println(F("------------------------------------------------------------"));
  Serial.print(F(" Thong so hien tai: Steps/Lan = "));
  Serial.print(jog_steps);
  Serial.print(F(" | Delay = "));
  Serial.print(jog_delay_us);
  Serial.println(F(" us"));

  set_all_axes_enable(true);

  while (true) {
    char c = read_char_blocking();

    if (c == '0' || c == 'x' || c == 'X') {
      Serial.println(F("\r\n-> Da thoat che do Jogging."));
      break;
    }

    switch (c) {
      case 'a':
      case 'A':
        Serial.print(F("<X- "));
        step_single_axis(PIN_STEP_X, PIN_DIR_X, LOW, jog_steps, jog_delay_us);
        break;
      case 'd':
      case 'D':
        Serial.print(F(" X+>"));
        step_single_axis(PIN_STEP_X, PIN_DIR_X, HIGH, jog_steps, jog_delay_us);
        break;

      case 'w':
      case 'W':
        Serial.print(F("^Y+ "));
        step_single_axis(PIN_STEP_Y, PIN_DIR_Y, HIGH, jog_steps, jog_delay_us);
        break;
      case 's':
      case 'S':
        Serial.print(F(" vY-"));
        step_single_axis(PIN_STEP_Y, PIN_DIR_Y, LOW, jog_steps, jog_delay_us);
        break;

      case 'q':
      case 'Q':
        Serial.print(F(" ^Z+"));
        step_single_axis(PIN_STEP_Z, PIN_DIR_Z, HIGH, jog_steps, jog_delay_us);
        break;
      case 'e':
      case 'E':
        Serial.print(F(" vZ-"));
        step_single_axis(PIN_STEP_Z, PIN_DIR_Z, LOW, jog_steps, jog_delay_us);
        break;

      case 'z':
      case 'Z':
        Serial.print(F(" <A-"));
        step_single_axis(PIN_STEP_A, PIN_DIR_A, LOW, jog_steps, jog_delay_us);
        break;
      case 'c':
      case 'C':
        Serial.print(F(" A+>"));
        step_single_axis(PIN_STEP_A, PIN_DIR_A, HIGH, jog_steps, jog_delay_us);
        break;

      case '+':
      case '=':
        jog_steps += 200;
        if (jog_steps > 3200) jog_steps = 3200;
        Serial.print(F("\r\n-> Steps moi lan Jog: "));
        Serial.println(jog_steps);
        break;

      case '-':
      case '_':
        if (jog_steps > 100) jog_steps -= 100;
        Serial.print(F("\r\n-> Steps moi lan Jog: "));
        Serial.println(jog_steps);
        break;

      case 'f':
      case 'F':
        if (jog_delay_us > 100) jog_delay_us -= 50;
        Serial.print(F("\r\n-> Delay: "));
        Serial.print(jog_delay_us);
        Serial.println(F(" us (NHANH HON)"));
        break;

      case 'g':
      case 'G':
        jog_delay_us += 50;
        if (jog_delay_us > 2000) jog_delay_us = 2000;
        Serial.print(F("\r\n-> Delay: "));
        Serial.print(jog_delay_us);
        Serial.println(F(" us (CHAM HON)"));
        break;

      default:
        break;
    }
  }
}

// =============================================================================
// 11. MODULE 3: KIỂM TRA TRỤC CHÍNH (SPINDLE)
// =============================================================================
void set_spindle(bool enable, bool dir_ccw, uint8_t pwm_val) {
  spindle_running = enable;
  spindle_dir_ccw = dir_ccw;
  spindle_speed = pwm_val;

  digitalWrite(PIN_SPINDLE_DIR, dir_ccw ? HIGH : LOW);
  digitalWrite(PIN_SPINDLE_ENABLE, enable ? HIGH : LOW);
  analogWrite(PIN_SPINDLE_PWM, enable ? pwm_val : 0);
}

void menu_spindle() {
  Serial.println(F("\r\n============================================================"));
  Serial.println(F("                  KIEM TRA TRUC CHINH (SPINDLE)             "));
  Serial.println(F("============================================================"));
  Serial.println(F(" [1] Bat / Tat Spindle (PC7 Enable)"));
  Serial.println(F(" [2] Dao chieu quay Spindle DIR (PC6: CW Thuan / CCW Nguoc)"));
  Serial.println(F(" [3] Chon toc do PWM: 25% (PWM = 64)"));
  Serial.println(F(" [4] Chon toc do PWM: 50% (PWM = 128)"));
  Serial.println(F(" [5] Chon toc do PWM: 75% (PWM = 191)"));
  Serial.println(F(" [6] Chon toc do PWM: 100% (PWM = 255)"));
  Serial.println(F(" [R] Test tang toc tu dong (Ramp Up 0% -> 100% -> 0% an toan)"));
  Serial.println(F(" [0] Tat Spindle va quay lai"));

  while (true) {
    Serial.print(F("\r\n[Spindle Status: "));
    Serial.print(spindle_running ? F("RUNNING") : F("STOPPED"));
    Serial.print(F(" | DIR: "));
    Serial.print(spindle_dir_ccw ? F("CCW (M4)") : F("CW (M3)"));
    Serial.print(F(" | PWM: "));
    Serial.print(spindle_speed);
    Serial.print(F(" ("));
    Serial.print((spindle_speed * 100) / 255);
    Serial.println(F("%)]"));
    Serial.print(F("Nhap lenh (1..6/R/0): "));

    char c = read_char_blocking();

    if (c == '0' || c == 'q' || c == 'Q') {
      set_spindle(false, false, 0);
      Serial.println(F("-> Da tat Spindle va quay ve Menu chinh."));
      break;
    }

    switch (c) {
      case '1':
        spindle_running = !spindle_running;
        if (spindle_running && spindle_speed == 0) spindle_speed = 128;
        set_spindle(spindle_running, spindle_dir_ccw, spindle_speed);
        Serial.println(spindle_running ? F("-> Da BAT Spindle!") : F("-> Da TAT Spindle!"));
        break;

      case '2':
        spindle_dir_ccw = !spindle_dir_ccw;
        set_spindle(spindle_running, spindle_dir_ccw, spindle_speed);
        Serial.print(F("-> Chieu quay hien tai: "));
        Serial.println(spindle_dir_ccw ? F("CCW (Quay nguoc kim dong ho)") : F("CW (Quay thuan kim dong ho)"));
        break;

      case '3':
        set_spindle(true, spindle_dir_ccw, 64);
        Serial.println(F("-> Toc do dat 25% (PWM=64)."));
        break;

      case '4':
        set_spindle(true, spindle_dir_ccw, 128);
        Serial.println(F("-> Toc do dat 50% (PWM=128)."));
        break;

      case '5':
        set_spindle(true, spindle_dir_ccw, 191);
        Serial.println(F("-> Toc do dat 75% (PWM=191)."));
        break;

      case '6':
        set_spindle(true, spindle_dir_ccw, 255);
        Serial.println(F("-> Toc do dat 100% (PWM=255 Max)."));
        break;

      case 'r':
      case 'R':
        Serial.println(F("-> Bat dau Ramp-Up test: 0% -> 100% trong 3 giay..."));
        digitalWrite(PIN_SPINDLE_DIR, LOW);
        digitalWrite(PIN_SPINDLE_ENABLE, HIGH);
        for (int p = 0; p <= 255; p += 5) {
          analogWrite(PIN_SPINDLE_PWM, p);
          delay(60);
        }
        delay(1000);
        Serial.println(F("-> Ramp-Down test: 100% -> 0% trong 3 giay..."));
        for (int p = 255; p >= 0; p -= 5) {
          analogWrite(PIN_SPINDLE_PWM, p);
          delay(60);
        }
        set_spindle(false, false, 0);
        Serial.println(F("-> Hoan thanh test Ramp Spindle."));
        break;

      default:
        Serial.println(F("[!] Lenh khong hop le."));
        break;
    }
  }
}

// =============================================================================
// 12. MODULE 4: KIỂM TRA TƯỚI NGUỘI & KHÍ LÀM MÁT (COOLANT & MIST)
// =============================================================================
void menu_coolant() {
  Serial.println(F("\r\n============================================================"));
  Serial.println(F("          KIEM TRA TUOI NGUOI & LAM MAT (COOLANT & MIST)    "));
  Serial.println(F("============================================================"));
  Serial.println(F(" [1] Bat / Tat Bom tuoi nguoi Flood (PD8 - Lenh M8/M9)"));
  Serial.println(F(" [2] Bat / Tat Van khi lam mat Mist (PD9 - Lenh M7/M9)"));
  Serial.println(F(" [3] Nhap nha Relay Flood 5 lan (Kiem tra dong ngat ro-le)"));
  Serial.println(F(" [4] Nhap nha Van Mist 5 lan (Kiem tra van dien tu khi)"));
  Serial.println(F(" [0] Tat tat ca va quay lai"));

  while (true) {
    Serial.print(F("\r\n[Trang thai -> Flood (PD8): "));
    Serial.print(flood_active ? F("ON") : F("OFF"));
    Serial.print(F(" | Mist (PD9): "));
    Serial.print(mist_active ? F("ON") : F("OFF"));
    Serial.println(F("]"));
    Serial.print(F("Nhap lenh (1..4/0): "));

    char c = read_char_blocking();

    if (c == '0' || c == 'q' || c == 'Q') {
      digitalWrite(PIN_COOLANT_FLOOD, LOW);
      digitalWrite(PIN_COOLANT_MIST, LOW);
      flood_active = false;
      mist_active = false;
      Serial.println(F("-> Da tat toan bo Coolant va quay ve Menu chinh."));
      break;
    }

    switch (c) {
      case '1':
        flood_active = !flood_active;
        digitalWrite(PIN_COOLANT_FLOOD, flood_active ? HIGH : LOW);
        Serial.print(F("-> Bom Flood (PD8): "));
        Serial.println(flood_active ? F("BAT (ON)") : F("TAT (OFF)"));
        break;

      case '2':
        mist_active = !mist_active;
        digitalWrite(PIN_COOLANT_MIST, mist_active ? HIGH : LOW);
        Serial.print(F("-> Van Mist (PD9): "));
        Serial.println(mist_active ? F("BAT (ON)") : F("TAT (OFF)"));
        break;

      case '3':
        Serial.println(F("-> Dang nhap nha Relay Flood 5 lan (500ms ON / 500ms OFF)..."));
        for (int i = 1; i <= 5; i++) {
          Serial.print(F("   Nhip "));
          Serial.println(i);
          digitalWrite(PIN_COOLANT_FLOOD, HIGH);
          delay(500);
          digitalWrite(PIN_COOLANT_FLOOD, LOW);
          delay(500);
        }
        flood_active = false;
        Serial.println(F("-> Hoan thanh test Flood."));
        break;

      case '4':
        Serial.println(F("-> Dang nhap nha Van Mist 5 lan (300ms ON / 300ms OFF)..."));
        for (int i = 1; i <= 5; i++) {
          Serial.print(F("   Nhip "));
          Serial.println(i);
          digitalWrite(PIN_COOLANT_MIST, HIGH);
          delay(300);
          digitalWrite(PIN_COOLANT_MIST, LOW);
          delay(300);
        }
        mist_active = false;
        Serial.println(F("-> Hoan thanh test Mist."));
        break;

      default:
        Serial.println(F("[!] Lenh khong hop le."));
        break;
    }
  }
}

// =============================================================================
// 13. MODULE 5: KIỂM TRA KHÍ NÉN THAY DAO TỰ ĐỘNG (ATC PNEUMATICS)
// =============================================================================
void test_atc_full_cycle() {
  Serial.println(F("\r\n--- MOPHONG CHU TRINH THAY DAO TU DONG ATC ---"));
  Serial.println(F("1. Kich hoat van Mo ngam nha dao (Unclamp PB10 = HIGH)..."));
  digitalWrite(PIN_ATC_UNCLAMP, HIGH);
  delay(300);

  Serial.println(F("2. Bat van Thoi khi ve sinh con dao (Air Blast PB11 = HIGH)..."));
  digitalWrite(PIN_ATC_AIR_BLAST, HIGH);
  delay(600);

  Serial.println(F("3. Tat van Thoi khi (Air Blast PB11 = LOW)..."));
  digitalWrite(PIN_ATC_AIR_BLAST, LOW);
  delay(300);

  Serial.println(F("4. Dong van Mo ngam -> Kep chat dao (Unclamp PB10 = LOW)..."));
  digitalWrite(PIN_ATC_UNCLAMP, LOW);
  delay(400);

  atc_unclamp_on = false;
  atc_air_blast_on = false;
  Serial.println(F("-> Chu trinh thay dao ATC hoan tat an toan!"));
}

void menu_atc() {
  Serial.println(F("\r\n============================================================"));
  Serial.println(F("       KIEM TRA KHI NEN THAY DAO TU DONG (ATC PNEUMATICS)   "));
  Serial.println(F("============================================================"));
  Serial.println(F(" [1] Kich hoat Van Mo ngam nha dao (PB10 ATC Unclamp)"));
  Serial.println(F(" [2] Kich hoat Van Thoi khi ve sinh con (PB11 ATC Air Blast)"));
  Serial.println(F(" [3] Chay chu trinh test Mo ngam -> Thoi khi -> Kep dao mo phong"));
  Serial.println(F(" [0] Dong toan bo van va quay lai"));

  while (true) {
    Serial.print(F("\r\n[Trang thai -> Unclamp (PB10): "));
    Serial.print(atc_unclamp_on ? F("MO NGAM (HIGH)") : F("KEP DAO (LOW)"));
    Serial.print(F(" | Air Blast (PB11): "));
    Serial.print(atc_air_blast_on ? F("DANG THOI (HIGH)") : F("TAT (LOW)"));
    Serial.println(F("]"));
    Serial.print(F("Nhap lenh (1..3/0): "));

    char c = read_char_blocking();

    if (c == '0' || c == 'q' || c == 'Q') {
      digitalWrite(PIN_ATC_UNCLAMP, LOW);
      digitalWrite(PIN_ATC_AIR_BLAST, LOW);
      atc_unclamp_on = false;
      atc_air_blast_on = false;
      Serial.println(F("-> Da tat toan bo van ATC va quay ve Menu chinh."));
      break;
    }

    switch (c) {
      case '1':
        atc_unclamp_on = !atc_unclamp_on;
        digitalWrite(PIN_ATC_UNCLAMP, atc_unclamp_on ? HIGH : LOW);
        Serial.print(F("-> Van Unclamp (PB10): "));
        Serial.println(atc_unclamp_on ? F("MO NGAM NHA DAO (HIGH)") : F("DONG NGAM KEP DAO (LOW)"));
        break;

      case '2':
        atc_air_blast_on = !atc_air_blast_on;
        digitalWrite(PIN_ATC_AIR_BLAST, atc_air_blast_on ? HIGH : LOW);
        Serial.print(F("-> Van Air Blast (PB11): "));
        Serial.println(atc_air_blast_on ? F("BAT THOI KHI (HIGH)") : F("TAT THOI KHI (LOW)"));
        break;

      case '3':
        test_atc_full_cycle();
        break;

      default:
        Serial.println(F("[!] Lenh khong hop le."));
        break;
    }
  }
}

// =============================================================================
// 14. MODULE 6: THEO DÕI CẢM BIẾN & CÔNG TẮC HÀNH TRÌNH (LIVE SENSOR MONITOR)
// =============================================================================
void monitor_sensors_live() {
  Serial.println(F("\r\n============================================================"));
  Serial.println(F("    THEO DOI CAM BIEN & CONG TAC HANH TRINH (LIVE MONITOR)  "));
  Serial.println(F("============================================================"));
  Serial.println(F(" Huong dan:"));
  Serial.println(F(" - Hay dung tay nhan/chạm vao tung cong tac hanh trinh hoac nut bam."));
  Serial.println(F(" - Man hinh se ngay lap tuc bao su kien khi co tin hieu kich hoat."));
  Serial.println(F(" - Nhan bat ky phim nao (hoac '0', 'X') tren ban phim de thoat."));
  Serial.println(F("------------------------------------------------------------"));

  int last_lim_x = digitalRead(PIN_LIMIT_X);
  int last_lim_y = digitalRead(PIN_LIMIT_Y);
  int last_lim_z = digitalRead(PIN_LIMIT_Z);
  int last_estop = digitalRead(PIN_INPUT_ESTOP);
  int last_hold = digitalRead(PIN_INPUT_FEED_HOLD);
  int last_start = digitalRead(PIN_INPUT_CYCLE_START);
  int last_door = digitalRead(PIN_INPUT_SAFETY_DOOR);
  int last_probe = digitalRead(PIN_INPUT_PROBE);

  Serial.println(F("[TRANG THAI HIEN TAI BAN DAU]:"));
  Serial.print(F("  LimX(PA0): "));
  Serial.print(last_lim_x ? F("OPEN (1)") : F("TRIGGERED (0)"));
  Serial.print(F(" | LimY(PA1): "));
  Serial.print(last_lim_y ? F("OPEN (1)") : F("TRIGGERED (0)"));
  Serial.print(F(" | LimZ(PA2): "));
  Serial.println(last_lim_z ? F("OPEN (1)") : F("TRIGGERED (0)"));

  Serial.print(F("  E-Stop(PE11): "));
  Serial.print(last_estop ? F("NORMAL (1)") : F("PRESSED (0)"));
  Serial.print(F(" | FeedHold(PB7): "));
  Serial.print(last_hold ? F("NORMAL (1)") : F("PRESSED (0)"));
  Serial.print(F(" | CycleStart(PB1): "));
  Serial.println(last_start ? F("NORMAL (1)") : F("PRESSED (0)"));

  Serial.print(F("  Door(PB8): "));
  Serial.print(last_door ? F("CLOSED (1)") : F("OPEN (0)"));
  Serial.print(F(" | Probe(PA7): "));
  Serial.println(last_probe ? F("CLEAR (1)") : F("TOUCHED (0)"));
  Serial.println(F("------------------------------------------------------------"));
  Serial.println(F(">> Dang lang nghe su kien thoi gian thuc..."));

  flush_serial_input();

  while (Serial.available() == 0) {
    int cur_lim_x = digitalRead(PIN_LIMIT_X);
    int cur_lim_y = digitalRead(PIN_LIMIT_Y);
    int cur_lim_z = digitalRead(PIN_LIMIT_Z);
    int cur_estop = digitalRead(PIN_INPUT_ESTOP);
    int cur_hold = digitalRead(PIN_INPUT_FEED_HOLD);
    int cur_start = digitalRead(PIN_INPUT_CYCLE_START);
    int cur_door = digitalRead(PIN_INPUT_SAFETY_DOOR);
    int cur_probe = digitalRead(PIN_INPUT_PROBE);

    if (cur_lim_x != last_lim_x) {
      last_lim_x = cur_lim_x;
      Serial.print(F(" [EVENT] LIMIT X -> "));
      Serial.println(cur_lim_x == LOW ? F(">>> TRIGGERED (DONG/CHAM) <<<") : F("RELEASED (NHA/HO)"));
    }
    if (cur_lim_y != last_lim_y) {
      last_lim_y = cur_lim_y;
      Serial.print(F(" [EVENT] LIMIT Y -> "));
      Serial.println(cur_lim_y == LOW ? F(">>> TRIGGERED (DONG/CHAM) <<<") : F("RELEASED (NHA/HO)"));
    }
    if (cur_lim_z != last_lim_z) {
      last_lim_z = cur_lim_z;
      Serial.print(F(" [EVENT] LIMIT Z -> "));
      Serial.println(cur_lim_z == LOW ? F(">>> TRIGGERED (DONG/CHAM) <<<") : F("RELEASED (NHA/HO)"));
    }
    if (cur_estop != last_estop) {
      last_estop = cur_estop;
      Serial.print(F(" [EVENT] NUT E-STOP -> "));
      Serial.println(cur_estop == LOW ? F(">>> NUT DUNG KHAN CAP BI BAM! <<<") : F("DA NHA NUT"));
    }
    if (cur_hold != last_hold) {
      last_hold = cur_hold;
      Serial.print(F(" [EVENT] NUT FEED HOLD -> "));
      Serial.println(cur_hold == LOW ? F(">>> BAM FEED HOLD <<<") : F("NHA FEED HOLD"));
    }
    if (cur_start != last_start) {
      last_start = cur_start;
      Serial.print(F(" [EVENT] NUT CYCLE START (PB1) -> "));
      Serial.println(cur_start == LOW ? F(">>> BAM CYCLE START <<<") : F("NHA CYCLE START"));
    }
    if (cur_door != last_door) {
      last_door = cur_door;
      Serial.print(F(" [EVENT] CUA AN TOAN (PB8) -> "));
      Serial.println(cur_door == LOW ? F(">>> CUA MO (SAFETY DOOR OPEN) <<<") : F("CUA DONG (SAFETY DOOR CLOSED)"));
    }
    if (cur_probe != last_probe) {
      last_probe = cur_probe;
      Serial.print(F(" [EVENT] CAM BIEN PROBE (PA7) -> "));
      Serial.println(cur_probe == LOW ? F(">>> DA CHAM DAO/PHOI (PROBE ACTIVE) <<<") : F("HO DAO (PROBE CLEAR)"));
    }

    delay(20);
  }

  flush_serial_input();
  Serial.println(F("-> Da thoat che do theo doi cam bien."));
}

// =============================================================================
// 15. MODULE 7: TỰ ĐỘNG KIỂM TRA TOÀN DIỆN (AUTO SELF-TEST SEQUENCE)
// =============================================================================
void run_auto_self_test() {
  Serial.println(F("\r\n============================================================"));
  Serial.println(F("        CHUONG TRINH TU DONG KIEM TRA TOAN DIEN (SELF-TEST) "));
  Serial.println(F("============================================================"));
  Serial.println(F(" CANH BAO: Vui long dam bao khong gian may CNC an toan"));
  Serial.println(F(" va khong co vat can truoc khi chay chuong trinh nay."));
  Serial.println(F(" Nhan 'Y' de tiep tuc, hoac bat ky phim nao khac de huy: "));

  char confirm = read_char_blocking();
  if (confirm != 'y' && confirm != 'Y') {
    Serial.println(F("-> Da huy qua trinh Auto Self-Test."));
    return;
  }

  Serial.println(F("\r\n[1/6] Kiem tra tung chan Driver Enable (X, Y, Z, A)..."));
  set_all_axes_enable(true);
  delay(300);
  Serial.println(F("   -> PASS: Enable 4 truc da duoc kich hoat."));

  Serial.println(F("[2/6] Kiem tra phat xung 4 truc dong co buoc (X, Y, Z, A)..."));
  Serial.print(F("   -> Test Truc X... "));
  step_single_axis(PIN_STEP_X, PIN_DIR_X, HIGH, 300, 400);
  step_single_axis(PIN_STEP_X, PIN_DIR_X, LOW, 300, 400);
  Serial.println(F("OK"));

  Serial.print(F("   -> Test Truc Y... "));
  step_single_axis(PIN_STEP_Y, PIN_DIR_Y, HIGH, 300, 400);
  step_single_axis(PIN_STEP_Y, PIN_DIR_Y, LOW, 300, 400);
  Serial.println(F("OK"));

  Serial.print(F("   -> Test Truc Z... "));
  step_single_axis(PIN_STEP_Z, PIN_DIR_Z, HIGH, 300, 400);
  step_single_axis(PIN_STEP_Z, PIN_DIR_Z, LOW, 300, 400);
  Serial.println(F("OK"));

  Serial.print(F("   -> Test Truc A... "));
  step_single_axis(PIN_STEP_A, PIN_DIR_A, HIGH, 300, 400);
  step_single_axis(PIN_STEP_A, PIN_DIR_A, LOW, 300, 400);
  Serial.println(F("OK"));

  Serial.println(F("[3/6] Kiem tra Role Tuoi nguoi (Flood PD8) & Van khi (Mist PD9)..."));
  digitalWrite(PIN_COOLANT_FLOOD, HIGH);
  delay(400);
  digitalWrite(PIN_COOLANT_FLOOD, LOW);
  delay(200);
  digitalWrite(PIN_COOLANT_MIST, HIGH);
  delay(400);
  digitalWrite(PIN_COOLANT_MIST, LOW);
  Serial.println(F("   -> PASS: Coolant & Mist output relay/valve OK."));

  Serial.println(F("[4/6] Kiem tra Van khi nen Thay dao tu dong ATC (PB10, PB11)..."));
  test_atc_full_cycle();
  Serial.println(F("   -> PASS: ATC Pneumatics OK."));

  Serial.println(F("[5/6] Kiem tra Tin hieu dieu khien Spindle (PWM PB0, DIR PC6, ENA PC7)..."));
  digitalWrite(PIN_SPINDLE_DIR, LOW);
  digitalWrite(PIN_SPINDLE_ENABLE, HIGH);
  for (int p = 0; p <= 128; p += 10) {
    analogWrite(PIN_SPINDLE_PWM, p);
    delay(30);
  }
  delay(500);
  for (int p = 128; p >= 0; p -= 10) {
    analogWrite(PIN_SPINDLE_PWM, p);
    delay(30);
  }
  set_spindle(false, false, 0);
  Serial.println(F("   -> PASS: Spindle PWM & Enable signals OK."));

  Serial.println(F("[6/6] Doc trang thai cac cong tac & cam bien dau vao..."));
  Serial.print(F("   -> X_Limit (PA0): "));
  Serial.println(digitalRead(PIN_LIMIT_X) ? F("HIGH (Pull-up Open)") : F("LOW (Triggered)"));
  Serial.print(F("   -> Y_Limit (PA1): "));
  Serial.println(digitalRead(PIN_LIMIT_Y) ? F("HIGH (Pull-up Open)") : F("LOW (Triggered)"));
  Serial.print(F("   -> Z_Limit (PA2): "));
  Serial.println(digitalRead(PIN_LIMIT_Z) ? F("HIGH (Pull-up Open)") : F("LOW (Triggered)"));
  Serial.print(F("   -> E-Stop  (PE11): "));
  Serial.println(digitalRead(PIN_INPUT_ESTOP) ? F("HIGH (Normal)") : F("LOW (Pressed)"));
  Serial.print(F("   -> Door    (PB8): "));
  Serial.println(digitalRead(PIN_INPUT_SAFETY_DOOR) ? F("HIGH (Closed)") : F("LOW (Open)"));
  Serial.print(F("   -> Probe   (PA7): "));
  Serial.println(digitalRead(PIN_INPUT_PROBE) ? F("HIGH (Clear)") : F("LOW (Touched)"));

  Serial.println(F("\r\n============================================================"));
  Serial.println(F("     KET QUA: DA HOAN TAT CHUONG TRINH TU DONG KIEM TRA!    "));
  Serial.println(F("============================================================"));
}

// =============================================================================
// 16. MODULE 8: QUY TRÌNH TỰ ĐỘNG VỀ GỐC TỌA ĐỘ MÁY (HOMING SEQUENCE)
// =============================================================================

// Hàm thực thi Homing cho 1 trục đơn (Dò 2 pha: Tìm nhanh -> Lùi ra -> Rà chậm -> Pull-off)
bool home_single_axis(char axis, uint32_t stepPin, uint32_t dirPin, uint32_t limitPin,
                      int homingDir, bool dirPosLevel, float stepsPerMm,
                      float searchFeed, float latchFeed, float pulloffMm, float maxTravelMm) {
  Serial.print(F("-> Dang ve Home truc "));
  Serial.print(axis);
  Serial.println(F("..."));

  // Xác định mức DIR để di chuyển theo hướng Homing
  bool dir_search = (homingDir > 0) ? dirPosLevel : !dirPosLevel;
  bool dir_pulloff = !dir_search;

  // Tính delay xung từ Feedrate (us)
  // step_rate = (feed_mm_min / 60.0) * stepsPerMm -> step_delay_us = 1,000,000 / step_rate
  uint32_t search_delay_us = (uint32_t)(60000000.0f / (searchFeed * stepsPerMm));
  uint32_t latch_delay_us = (uint32_t)(60000000.0f / (latchFeed * stepsPerMm));
  if (search_delay_us < 50) search_delay_us = 50;
  if (latch_delay_us < 100) latch_delay_us = 100;

  uint32_t max_steps = (uint32_t)(maxTravelMm * stepsPerMm);
  uint32_t pulloff_steps = (uint32_t)(pulloffMm * stepsPerMm);

  // --- PHA 0: Nếu công tắc đang bị đè sẵn, lùi ra trước ---
  if (digitalRead(limitPin) == LIMIT_TRIGGERED_LEVEL) {
    Serial.print(F("   [Pha 0] Cong tac "));
    Serial.print(axis);
    Serial.println(F(" dang cham -> Dang lui ra..."));
    digitalWrite(dirPin, dir_pulloff);
    delayMicroseconds(20);
    uint32_t release_steps = 0;
    while (digitalRead(limitPin) == LIMIT_TRIGGERED_LEVEL && release_steps < pulloff_steps * 3) {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(5);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(search_delay_us);
      release_steps++;
    }
    delay(100);
    if (digitalRead(limitPin) == LIMIT_TRIGGERED_LEVEL) {
      Serial.print(F("   [!] LOI: Cong tac "));
      Serial.print(axis);
      Serial.println(F(" bi ket khong the nha ra!"));
      return false;
    }
  }

  // --- PHA 1: Tìm kiếm nhanh (Fast Search) ---
  Serial.print(F("   [Pha 1] Tim cong tac toc do nhanh ("));
  Serial.print(searchFeed, 0);
  Serial.println(F(" mm/phut)..."));
  digitalWrite(dirPin, dir_search);
  delayMicroseconds(20);

  uint32_t step_count = 0;
  bool hit_limit = false;

  while (step_count < max_steps) {
    if (digitalRead(limitPin) == LIMIT_TRIGGERED_LEVEL) {
      hit_limit = true;
      break;
    }
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(5);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(search_delay_us);
    step_count++;
  }

  if (!hit_limit) {
    Serial.print(F("   [!] LOI: Vuot qua hanh trinh toi da ("));
    Serial.print(maxTravelMm, 0);
    Serial.print(F("mm) ma khong cham cong tac "));
    Serial.print(axis);
    Serial.println(F("!"));
    return false;
  }

  delay(100);

  // --- PHA 2: Lùi ra khoảng cách an toàn (Pulloff) ---
  Serial.print(F("   [Pha 2] Lui ra khoi cong tac "));
  Serial.print(pulloffMm, 1);
  Serial.println(F(" mm..."));
  digitalWrite(dirPin, dir_pulloff);
  delayMicroseconds(20);
  for (uint32_t i = 0; i < pulloff_steps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(5);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(search_delay_us);
  }
  delay(100);

  // --- PHA 3: Rà chậm xác định điểm chuyển trạng thái (Slow Latch) ---
  Serial.print(F("   [Pha 3] Ra cham chinh xac ("));
  Serial.print(latchFeed, 0);
  Serial.println(F(" mm/phut)..."));
  digitalWrite(dirPin, dir_search);
  delayMicroseconds(20);

  step_count = 0;
  hit_limit = false;
  uint32_t max_latch_steps = pulloff_steps * 2;

  while (step_count < max_latch_steps) {
    if (digitalRead(limitPin) == LIMIT_TRIGGERED_LEVEL) {
      hit_limit = true;
      break;
    }
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(5);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(latch_delay_us);
    step_count++;
  }

  if (!hit_limit) {
    Serial.print(F("   [!] LOI: Khong xac dinh duoc diem cham cham o truc "));
    Serial.print(axis);
    Serial.println(F("!"));
    return false;
  }

  delay(100);

  // --- PHA 4: Lùi về vị trí Gốc 0.0 an toàn (Final Pulloff) ---
  Serial.print(F("   [Pha 4] Lui ve vi tri chuan "));
  Serial.print(pulloffMm, 1);
  Serial.println(F(" mm va thiet lap goc toan bo..."));
  digitalWrite(dirPin, dir_pulloff);
  delayMicroseconds(20);
  for (uint32_t i = 0; i < pulloff_steps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(5);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(search_delay_us);
  }

  Serial.print(F("-> PASS: Truc "));
  Serial.print(axis);
  Serial.println(F(" da ve Home thanh cong!"));
  return true;
}

// Chu trình Homing đầy đủ theo thứ tự chuẩn: Z (nhấc lên an toàn) -> X -> Y
bool run_homing_sequence() {
  Serial.println(F("\r\n============================================================"));
  Serial.println(F("        BAT DAU QUY TRINH VE HOME CHUAN CNC (Z -> X -> Y)   "));
  Serial.println(F("============================================================"));
  set_all_axes_enable(true);
  delay(100);

  // 1. Trục Z về Home trước tiên để dao nhấc lên cao nhất, tránh va quẹt
  Serial.println(F("[1/3] Homing Truc Z (Nac dao len cao an toan)..."));
  bool ok_z = home_single_axis('Z', PIN_STEP_Z, PIN_DIR_Z, PIN_LIMIT_Z,
                               HOMING_DIR_Z, DIR_POS_Z, STEPS_PER_MM_Z,
                               HOMING_SEARCH_FEED_Z, HOMING_LATCH_FEED_Z,
                               HOMING_PULLOFF_MM, HOMING_MAX_TRAVEL_Z);
  if (!ok_z) {
    Serial.println(F("\r\n[!] DUNG KHAN: Homing Truc Z THAT BAI!"));
    return false;
  }
  cur_pos_z = 0.0f;
  delay(300);

  // 2. Trục X về Home
  Serial.println(F("[2/3] Homing Truc X..."));
  bool ok_x = home_single_axis('X', PIN_STEP_X, PIN_DIR_X, PIN_LIMIT_X,
                               HOMING_DIR_X, DIR_POS_X, STEPS_PER_MM_X,
                               HOMING_SEARCH_FEED_XY, HOMING_LATCH_FEED_XY,
                               HOMING_PULLOFF_MM, HOMING_MAX_TRAVEL_X);
  if (!ok_x) {
    Serial.println(F("\r\n[!] DUNG KHAN: Homing Truc X THAT BAI!"));
    return false;
  }
  cur_pos_x = 0.0f;
  delay(300);

  // 3. Trục Y về Home
  Serial.println(F("[3/3] Homing Truc Y..."));
  bool ok_y = home_single_axis('Y', PIN_STEP_Y, PIN_DIR_Y, PIN_LIMIT_Y,
                               HOMING_DIR_Y, DIR_POS_Y, STEPS_PER_MM_Y,
                               HOMING_SEARCH_FEED_XY, HOMING_LATCH_FEED_XY,
                               HOMING_PULLOFF_MM, HOMING_MAX_TRAVEL_Y);
  if (!ok_y) {
    Serial.println(F("\r\n[!] DUNG KHAN: Homing Truc Y THAT BAI!"));
    return false;
  }
  cur_pos_y = 0.0f;
  delay(200);

  is_machine_homed = true;
  Serial.println(F("\r\n============================================================"));
  Serial.println(F("   DA HOAN TAT HOMING TOAN BO CAC TRUC! TOA DO: (0, 0, 0)   "));
  Serial.println(F("============================================================"));
  return true;
}

void menu_homing() {
  Serial.println(F("\r\n============================================================"));
  Serial.println(F("            MENU KIEM TRA VE GOC TOA DO (HOMING)            "));
  Serial.println(F("============================================================"));
  Serial.println(F(" [1] Chay chu trinh Homing toan bo truc (Thu tu Z -> X -> Y)"));
  Serial.println(F(" [Z] Chi ve Home rieng Truc Z"));
  Serial.println(F(" [X] Chi ve Home rieng Truc X"));
  Serial.println(F(" [Y] Chi ve Home rieng Truc Y"));
  Serial.println(F(" [0] Quay lai Menu chinh"));

  while (true) {
    Serial.println();
    Serial.print(F("Trang thai Homing hien tai: "));
    Serial.println(is_machine_homed ? F("DA HOMED (San sang gia cong)") : F("CHUA HOMED (Toa do chua xac dinh)"));
    Serial.print(F("Toa do hien tai: X="));
    Serial.print(cur_pos_x, 2);
    Serial.print(F(" mm | Y="));
    Serial.print(cur_pos_y, 2);
    Serial.print(F(" mm | Z="));
    Serial.print(cur_pos_z, 2);
    Serial.println(F(" mm"));
    Serial.print(F("[Homing] Nhap lua chon (1/Z/X/Y/0): "));

    char c = read_char_blocking();

    if (c == '0' || c == 'q' || c == 'Q') {
      Serial.println(F("-> Thoat ve Menu chinh."));
      break;
    }

    switch (c) {
      case '1':
        run_homing_sequence();
        break;

      case 'z':
      case 'Z':
        set_all_axes_enable(true);
        if (home_single_axis('Z', PIN_STEP_Z, PIN_DIR_Z, PIN_LIMIT_Z,
                             HOMING_DIR_Z, DIR_POS_Z, STEPS_PER_MM_Z,
                             HOMING_SEARCH_FEED_Z, HOMING_LATCH_FEED_Z,
                             HOMING_PULLOFF_MM, HOMING_MAX_TRAVEL_Z)) {
          cur_pos_z = 0.0f;
        }
        break;

      case 'x':
      case 'X':
        set_all_axes_enable(true);
        if (home_single_axis('X', PIN_STEP_X, PIN_DIR_X, PIN_LIMIT_X,
                             HOMING_DIR_X, DIR_POS_X, STEPS_PER_MM_X,
                             HOMING_SEARCH_FEED_XY, HOMING_LATCH_FEED_XY,
                             HOMING_PULLOFF_MM, HOMING_MAX_TRAVEL_X)) {
          cur_pos_x = 0.0f;
        }
        break;

      case 'y':
      case 'Y':
        set_all_axes_enable(true);
        if (home_single_axis('Y', PIN_STEP_Y, PIN_DIR_Y, PIN_LIMIT_Y,
                             HOMING_DIR_Y, DIR_POS_Y, STEPS_PER_MM_Y,
                             HOMING_SEARCH_FEED_XY, HOMING_LATCH_FEED_XY,
                             HOMING_PULLOFF_MM, HOMING_MAX_TRAVEL_Y)) {
          cur_pos_y = 0.0f;
        }
        break;

      default:
        Serial.println(F("[!] Lenh khong hop le."));
        break;
    }
  }
}

// =============================================================================
// 17. MODULE 9: ĐIỀU KHIỂN NỘI SUY ĐA TRỤC & CHẠY BIÊN DẠNG HÌNH HỌC (SHAPES & 3D)
// =============================================================================

// Bộ tạo chuyển động nội suy tuyến tính 3 trục đồng thời (3D DDA Linear Interpolator)
bool move_to_linear(float target_x, float target_y, float target_z, float feedrate_mm_min) {
  float dx = target_x - cur_pos_x;
  float dy = target_y - cur_pos_y;
  float dz = target_z - cur_pos_z;
  float dist = sqrt(dx * dx + dy * dy + dz * dz);

  if (dist < 0.0005f) {
    cur_pos_x = target_x;
    cur_pos_y = target_y;
    cur_pos_z = target_z;
    return true;
  }

  // Tính số bước xung cần phát trên mỗi trục
  int32_t steps_x = (int32_t)round(fabs(dx) * STEPS_PER_MM_X);
  int32_t steps_y = (int32_t)round(fabs(dy) * STEPS_PER_MM_Y);
  int32_t steps_z = (int32_t)round(fabs(dz) * STEPS_PER_MM_Z);
  int32_t max_steps = max(steps_x, max(steps_y, steps_z));

  if (max_steps == 0) {
    cur_pos_x = target_x;
    cur_pos_y = target_y;
    cur_pos_z = target_z;
    return true;
  }

  // Thiết lập chân DIR theo dấu của độ dịch chuyển (dx, dy, dz)
  bool dir_x = (dx >= 0) ? DIR_POS_X : !DIR_POS_X;
  bool dir_y = (dy >= 0) ? DIR_POS_Y : !DIR_POS_Y;
  bool dir_z = (dz >= 0) ? DIR_POS_Z : !DIR_POS_Z;

  digitalWrite(PIN_DIR_X, dir_x);
  digitalWrite(PIN_DIR_Y, dir_y);
  digitalWrite(PIN_DIR_Z, dir_z);
  delayMicroseconds(20);

  // Tính thời gian delay giữa các bước phát xung
  float total_time_us = (dist / feedrate_mm_min) * 60.0f * 1000000.0f;
  uint32_t step_delay_us = (uint32_t)(total_time_us / max_steps);
  if (step_delay_us < 35) step_delay_us = 35;
  if (step_delay_us > 4000) step_delay_us = 4000;

  // Thuật toán tích lũy DDA (Bresenham đa chiều)
  int32_t acc_x = max_steps / 2;
  int32_t acc_y = max_steps / 2;
  int32_t acc_z = max_steps / 2;

  for (int32_t step = 0; step < max_steps; step++) {
    bool pulse_x = false;
    bool pulse_y = false;
    bool pulse_z = false;

    acc_x += steps_x;
    if (acc_x >= max_steps) {
      acc_x -= max_steps;
      pulse_x = true;
    }

    acc_y += steps_y;
    if (acc_y >= max_steps) {
      acc_y -= max_steps;
      pulse_y = true;
    }

    acc_z += steps_z;
    if (acc_z >= max_steps) {
      acc_z -= max_steps;
      pulse_z = true;
    }

    if (pulse_x) digitalWrite(PIN_STEP_X, HIGH);
    if (pulse_y) digitalWrite(PIN_STEP_Y, HIGH);
    if (pulse_z) digitalWrite(PIN_STEP_Z, HIGH);

    delayMicroseconds(5);

    if (pulse_x) digitalWrite(PIN_STEP_X, LOW);
    if (pulse_y) digitalWrite(PIN_STEP_Y, LOW);
    if (pulse_z) digitalWrite(PIN_STEP_Z, LOW);

    delayMicroseconds(step_delay_us - 5);

    // Cho phép ngắt dừng khẩn cấp bằng phím nếu gửi qua Serial ('x', 'q' hoặc ESC)
    if ((step & 0x7F) == 0 && Serial.available() > 0) {
      char abort_c = Serial.read();
      if (abort_c == 'x' || abort_c == 'X' || abort_c == 'q' || abort_c == 'Q' || abort_c == 27) {
        Serial.println(F("\r\n[!] NGAT KHAN CAP: Nguoi dung da dung chuyen dong!"));
        cur_pos_x = target_x;
        cur_pos_y = target_y;
        cur_pos_z = target_z;
        return false;
      }
    }
  }

  cur_pos_x = target_x;
  cur_pos_y = target_y;
  cur_pos_z = target_z;
  return true;
}

// 1. Chạy biên dạng Hình Vuông (Square)
void run_shape_square() {
  Serial.println(F("\r\n--- DANG CHAY BIEN DANG: HINH VUONG (SQUARE) ---"));
  Serial.print(F(" Kich thuoc: "));
  Serial.print(SQUARE_SIZE_MM, 1);
  Serial.print(F("mm x "));
  Serial.print(SQUARE_SIZE_MM, 1);
  Serial.print(F("mm | Toc do: "));
  Serial.print(SQUARE_FEEDRATE, 0);
  Serial.println(F(" mm/phut"));

  set_all_axes_enable(true);

  float x0 = SHAPE_START_X_MM;
  float y0 = SHAPE_START_Y_MM;
  float w = SQUARE_SIZE_MM;

  // 1. Nhấc Z lên Safe Z
  Serial.println(F(" 1. Nhac Z len do cao an toan (Safe Z)..."));
  move_to_linear(cur_pos_x, cur_pos_y, SHAPE_SAFE_Z_MM, RAPID_FEEDRATE);

  // 2. Di chuyển nhanh đến góc xuất phát (X0, Y0)
  Serial.println(F(" 2. Di chuyen nhanh den goc xuat phat (X0, Y0)..."));
  move_to_linear(x0, y0, SHAPE_SAFE_Z_MM, RAPID_FEEDRATE);

  // 3. Hạ Z xuống độ cao làm việc
  Serial.println(F(" 3. Ha Z xuong do cao lam viec (Work Z)..."));
  move_to_linear(x0, y0, SHAPE_WORK_Z_MM, 300.0f);

  // 4. Chạy 4 cạnh của hình vuông
  Serial.println(F(" 4. Canh 1: (X0, Y0) -> (X0+W, Y0)..."));
  move_to_linear(x0 + w, y0, SHAPE_WORK_Z_MM, SQUARE_FEEDRATE);

  Serial.println(F(" 5. Canh 2: (X0+W, Y0) -> (X0+W, Y0+W)..."));
  move_to_linear(x0 + w, y0 + w, SHAPE_WORK_Z_MM, SQUARE_FEEDRATE);

  Serial.println(F(" 6. Canh 3: (X0+W, Y0+W) -> (X0, Y0+W)..."));
  move_to_linear(x0, y0 + w, SHAPE_WORK_Z_MM, SQUARE_FEEDRATE);

  Serial.println(F(" 7. Canh 4: (X0, Y0+W) -> (X0, Y0) [Dong kin]..."));
  move_to_linear(x0, y0, SHAPE_WORK_Z_MM, SQUARE_FEEDRATE);

  // 5. Nhấc Z lên Safe Z sau khi hoàn tất
  Serial.println(F(" 8. Nhac Z len do cao an toan..."));
  move_to_linear(x0, y0, SHAPE_SAFE_Z_MM, RAPID_FEEDRATE);

  Serial.println(F("-> Hoan tat chay bien dang Hinh Vuong!"));
}

// 2. Chạy biên dạng Hình Tam Giác Đều (Equilateral Triangle)
void run_shape_triangle() {
  Serial.println(F("\r\n--- DANG CHAY BIEN DANG: HINH TAM GIAC (TRIANGLE) ---"));
  Serial.print(F(" Canh tam giac: "));
  Serial.print(TRIANGLE_SIDE_MM, 1);
  Serial.print(F("mm | Toc do: "));
  Serial.print(TRIANGLE_FEEDRATE, 0);
  Serial.println(F(" mm/phut"));

  set_all_axes_enable(true);

  float x0 = SHAPE_START_X_MM;
  float y0 = SHAPE_START_Y_MM;
  float side = TRIANGLE_SIDE_MM;
  float height = side * 0.8660254f;  // Chiều cao tam giác đều: side * sqrt(3)/2

  float x_v1 = x0;
  float y_v1 = y0;
  float x_v2 = x0 + side;
  float y_v2 = y0;
  float x_v3 = x0 + (side / 2.0f);
  float y_v3 = y0 + height;

  // 1. Nhấc Z lên Safe Z
  Serial.println(F(" 1. Nhac Z len do cao an toan (Safe Z)..."));
  move_to_linear(cur_pos_x, cur_pos_y, SHAPE_SAFE_Z_MM, RAPID_FEEDRATE);

  // 2. Di chuyển nhanh đến đỉnh 1
  Serial.println(F(" 2. Di chuyen den Dinh 1 (Duoi trai)..."));
  move_to_linear(x_v1, y_v1, SHAPE_SAFE_Z_MM, RAPID_FEEDRATE);

  // 3. Hạ Z xuống độ cao làm việc
  Serial.println(F(" 3. Ha Z xuong do cao lam viec (Work Z)..."));
  move_to_linear(x_v1, y_v1, SHAPE_WORK_Z_MM, 300.0f);

  // 4. Cạnh đáy: Đỉnh 1 -> Đỉnh 2
  Serial.println(F(" 4. Canh 1: Dinh 1 -> Dinh 2 (Duoi phai)..."));
  move_to_linear(x_v2, y_v2, SHAPE_WORK_Z_MM, TRIANGLE_FEEDRATE);

  // 5. Cạnh nghiêng phải: Đỉnh 2 -> Đỉnh 3 (Đỉnh trên)
  Serial.println(F(" 5. Canh 2: Dinh 2 -> Dinh 3 (Chot dinh)..."));
  move_to_linear(x_v3, y_v3, SHAPE_WORK_Z_MM, TRIANGLE_FEEDRATE);

  // 6. Cạnh nghiêng trái: Đỉnh 3 -> Đỉnh 1 (Khép kín)
  Serial.println(F(" 6. Canh 3: Dinh 3 -> Dinh 1 [Dong kin]..."));
  move_to_linear(x_v1, y_v1, SHAPE_WORK_Z_MM, TRIANGLE_FEEDRATE);

  // 7. Nhấc Z lên Safe Z
  Serial.println(F(" 7. Nhac Z len do cao an toan..."));
  move_to_linear(x_v1, y_v1, SHAPE_SAFE_Z_MM, RAPID_FEEDRATE);

  Serial.println(F("-> Hoan tat chay bien dang Hinh Tam Giac!"));
}

// 3. Chạy biên dạng Hình Tròn (Circle)
void run_shape_circle() {
  Serial.println(F("\r\n--- DANG CHAY BIEN DANG: HINH TRON (CIRCLE) ---"));
  Serial.print(F(" Ban kinh R = "));
  Serial.print(CIRCLE_RADIUS_MM, 1);
  Serial.print(F("mm (Duong kinh D = "));
  Serial.print(CIRCLE_RADIUS_MM * 2.0f, 1);
  Serial.print(F("mm) | So doan: "));
  Serial.print(CIRCLE_SEGMENTS);
  Serial.print(F(" | Toc do: "));
  Serial.print(CIRCLE_FEEDRATE, 0);
  Serial.println(F(" mm/phut"));

  set_all_axes_enable(true);

  float r = CIRCLE_RADIUS_MM;
  float xc = SHAPE_START_X_MM + r;  // Tâm đường tròn
  float yc = SHAPE_START_Y_MM + r;

  // Điểm xuất phát trên chu vi (theta = 0)
  float x_start = xc + r;
  float y_start = yc;

  // 1. Nhấc Z lên Safe Z
  Serial.println(F(" 1. Nhac Z len do cao an toan (Safe Z)..."));
  move_to_linear(cur_pos_x, cur_pos_y, SHAPE_SAFE_Z_MM, RAPID_FEEDRATE);

  // 2. Di chuyển đến điểm xuất phát chu vi đường tròn
  Serial.println(F(" 2. Di chuyen den diem bat dau tren duong tron..."));
  move_to_linear(x_start, y_start, SHAPE_SAFE_Z_MM, RAPID_FEEDRATE);

  // 3. Hạ Z xuống độ cao làm việc
  Serial.println(F(" 3. Ha Z xuong do cao lam viec (Work Z)..."));
  move_to_linear(x_start, y_start, SHAPE_WORK_Z_MM, 300.0f);

  // 4. Nội suy cung tròn liên tục 360 độ
  Serial.println(F(" 4. Dang noi suy duong tron 360 do..."));
  for (int i = 1; i <= CIRCLE_SEGMENTS; i++) {
    float theta = ((float)i / (float)CIRCLE_SEGMENTS) * 2.0f * 3.14159265f;
    float xi = xc + r * cos(theta);
    float yi = yc + r * sin(theta);
    move_to_linear(xi, yi, SHAPE_WORK_Z_MM, CIRCLE_FEEDRATE);
  }

  // 5. Nhấc Z lên Safe Z
  Serial.println(F(" 5. Nhac Z len do cao an toan..."));
  move_to_linear(x_start, y_start, SHAPE_SAFE_Z_MM, RAPID_FEEDRATE);

  Serial.println(F("-> Hoan tat chay bien dang Hinh Tron!"));
}

// 4. Chạy biên dạng 3D Xoắn ốc (3D Helical Spiral Contour)
void run_shape_3d_helix() {
  Serial.println(F("\r\n--- DANG CHAY BIEN DANG: XOAN OC 3D (3D HELICAL CONTOUR) ---"));
  Serial.print(F(" Ban kinh R = "));
  Serial.print(PROFILE_3D_RADIUS_MM, 1);
  Serial.print(F("mm | Chieu sau Z = "));
  Serial.print(PROFILE_3D_DEPTH_MM, 1);
  Serial.print(F("mm | So vong xoan = "));
  Serial.print(PROFILE_3D_TURNS);
  Serial.print(F(" | Toc do 3D: "));
  Serial.print(PROFILE_3D_FEEDRATE, 0);
  Serial.println(F(" mm/phut"));

  set_all_axes_enable(true);

  float r = PROFILE_3D_RADIUS_MM;
  float xc = SHAPE_START_X_MM + r;
  float yc = SHAPE_START_Y_MM + r;
  float z_top = SHAPE_WORK_Z_MM;
  float z_bottom = SHAPE_WORK_Z_MM - PROFILE_3D_DEPTH_MM;

  int segments_per_turn = 72;
  int total_segments = PROFILE_3D_TURNS * segments_per_turn;

  float x_start = xc + r;
  float y_start = yc;

  // 1. Nhấc Z lên Safe Z
  Serial.println(F(" 1. Nhac Z len do cao an toan (Safe Z)..."));
  move_to_linear(cur_pos_x, cur_pos_y, SHAPE_SAFE_Z_MM, RAPID_FEEDRATE);

  // 2. Di chuyển nhanh đến điểm bắt đầu xoắn ốc
  Serial.println(F(" 2. Di chuyen den vi tri bat dau cua chu trinh 3D..."));
  move_to_linear(x_start, y_start, SHAPE_SAFE_Z_MM, RAPID_FEEDRATE);

  // 3. Hạ Z xuống mặt phẳng gia công
  move_to_linear(x_start, y_start, z_top, 300.0f);

  // 4. Chạy xoắn ốc 3D ăn sâu dần (Helical Ramp Down 3D Simultaneous Interpolation)
  Serial.println(F(" 3. [Pha 3D Xuong] Noi suy 3 truc X,Y,Z an sau theo duong xoan oc..."));
  for (int j = 1; j <= total_segments; j++) {
    float theta = ((float)j / (float)segments_per_turn) * 2.0f * 3.14159265f;
    float zj = z_top - (PROFILE_3D_DEPTH_MM * ((float)j / (float)total_segments));
    float xj = xc + r * cos(theta);
    float yj = yc + r * sin(theta);
    move_to_linear(xj, yj, zj, PROFILE_3D_FEEDRATE);
  }

  // 5. Cắt 1 vòng làm phẳng đáy ở độ sâu Z tối đa (Bottom Cleanup Circle)
  Serial.println(F(" 4. [Lam phang day] Chay 1 vong tron day o do sau toi da..."));
  for (int i = 1; i <= segments_per_turn; i++) {
    float theta = ((float)i / (float)segments_per_turn) * 2.0f * 3.14159265f;
    float xj = xc + r * cos(theta);
    float yj = yc + r * sin(theta);
    move_to_linear(xj, yj, z_bottom, PROFILE_3D_FEEDRATE);
  }

  // 6. Chạy xoắn ốc 3D đi ngược lên lại bề mặt (Helical Ramp Up)
  Serial.println(F(" 5. [Pha 3D Len] Xoan oc 3 truc nguoc len be mat..."));
  for (int j = 1; j <= total_segments; j++) {
    float theta = ((float)j / (float)segments_per_turn) * 2.0f * 3.14159265f;
    float zj = z_bottom + (PROFILE_3D_DEPTH_MM * ((float)j / (float)total_segments));
    float xj = xc + r * cos(theta);
    float yj = yc + r * sin(theta);
    move_to_linear(xj, yj, zj, PROFILE_3D_FEEDRATE);
  }

  // 7. Nhấc Z lên Safe Z sau khi hoàn tất
  Serial.println(F(" 6. Nhac Z len do cao an toan..."));
  move_to_linear(cur_pos_x, cur_pos_y, SHAPE_SAFE_Z_MM, RAPID_FEEDRATE);

  Serial.println(F("-> Hoan tat chay bien dang Xoan Oc 3D!"));
}

// 5. Chu trình tự động toàn diện: Homing -> Vuông -> Tam Giác -> Tròn -> 3D Helix -> Park
void run_full_sequence_test() {
  Serial.println(F("\r\n============================================================"));
  Serial.println(F("  CHUONG TRINH TU DONG: HOMING -> CHAY TOAN BO CAC BIEN DANG "));
  Serial.println(F("============================================================"));
  Serial.println(F(" Chu trinh se gom 5 buoc lien hoan:"));
  Serial.println(F("   [1] Ve Home toan bo cac truc (Z -> X -> Y)"));
  Serial.println(F("   [2] Chay bien dang Hinh Vuong 2D"));
  Serial.println(F("   [3] Chay bien dang Hinh Tam Giac 2D"));
  Serial.println(F("   [4] Chay bien dang Hinh Tron 2D"));
  Serial.println(F("   [5] Chay bien dang Xoan Oc 3D Helical"));
  Serial.println(F("   [6] Nhac Z va do may ve vi tri an toan (Park)"));
  Serial.println(F(" Nhan 'Y' de bat dau, hoac bat ky phim nao khac de huy: "));

  char confirm = read_char_blocking();
  if (confirm != 'y' && confirm != 'Y') {
    Serial.println(F("-> Da huy chuong trinh."));
    return;
  }

  // Bước 1: Homing
  Serial.println(F("\r\n>>> BƯỚC 1/5: THUC HIEN VE GOC TOA DO (HOMING) <<<"));
  if (!run_homing_sequence()) {
    Serial.println(F("[!] Homing that bai. Huy chu trinh tu dong!"));
    return;
  }
  delay(500);

  // Bước 2: Hình vuông
  Serial.println(F("\r\n>>> BƯỚC 2/5: CHAY BIEN DANG HINH VUONG <<<"));
  run_shape_square();
  delay(500);

  // Bước 3: Hình tam giác
  Serial.println(F("\r\n>>> BƯỚC 3/5: CHAY BIEN DANG HINH TAM GIAC <<<"));
  run_shape_triangle();
  delay(500);

  // Bước 4: Hình tròn
  Serial.println(F("\r\n>>> BƯỚC 4/5: CHAY BIEN DANG HINH TRON <<<"));
  run_shape_circle();
  delay(500);

  // Bước 5: Biên dạng 3D
  Serial.println(F("\r\n>>> BƯỚC 5/5: CHAY BIEN DANG 3D XOAN OC <<<"));
  run_shape_3d_helix();
  delay(500);

  // Về vị trí đỗ an toàn (Park position)
  Serial.println(F("\r\n>>> DO BAN MAY VE VI TRI AN TOAN (PARK POSITION) <<<"));
  move_to_linear(cur_pos_x, cur_pos_y, SHAPE_SAFE_Z_MM, RAPID_FEEDRATE);
  move_to_linear(0.0f, 0.0f, SHAPE_SAFE_Z_MM, RAPID_FEEDRATE);

  Serial.println(F("\r\n============================================================"));
  Serial.println(F("      XIN CHUC MUNG: DA HOAN TAT TOAN BO CHU TRINH TEST!    "));
  Serial.println(F("============================================================"));
}

void menu_shapes() {
  Serial.println(F("\r\n============================================================"));
  Serial.println(F("         MENU CHAY CAC BIEN DANG MAU (SHAPES & 3D PROFILE)   "));
  Serial.println(F("============================================================"));
  Serial.println(F(" [1] Chay FULL Chu Trinh: Homing -> Vuong -> Tam Giac -> Tron -> 3D -> Park"));
  Serial.println(F(" [2] Chay rieng Bien dang HINH VUONG (Square 40x40mm)"));
  Serial.println(F(" [3] Chay rieng Bien dang HINH TAM GIAC (Triangle canh 40mm)"));
  Serial.println(F(" [4] Chay rieng Bien dang HINH TRON (Circle ban kinh 20mm)"));
  Serial.println(F(" [5] Chay rieng Bien dang 3D XOAN OC (3D Helical Spiral)"));
  Serial.println(F(" [I] Xem danh sach thong so kich thuoc dang cai dat trong code"));
  Serial.println(F(" [0] Quay lai Menu chinh"));

  while (true) {
    Serial.println();
    Serial.print(F("Toa do hien tai: X="));
    Serial.print(cur_pos_x, 2);
    Serial.print(F(" mm | Y="));
    Serial.print(cur_pos_y, 2);
    Serial.print(F(" mm | Z="));
    Serial.print(cur_pos_z, 2);
    Serial.print(F(" mm ["));
    Serial.print(is_machine_homed ? F("Homed") : F("Not Homed"));
    Serial.println(F("]"));
    Serial.print(F("[Shapes] Nhap lua chon (1..5/I/0): "));

    char c = read_char_blocking();

    if (c == '0' || c == 'q' || c == 'Q') {
      Serial.println(F("-> Thoat ve Menu chinh."));
      break;
    }

    switch (c) {
      case '1':
        run_full_sequence_test();
        break;

      case '2':
        run_shape_square();
        break;

      case '3':
        run_shape_triangle();
        break;

      case '4':
        run_shape_circle();
        break;

      case '5':
        run_shape_3d_helix();
        break;

      case 'i':
      case 'I':
        Serial.println(F("\r\n--- THONG SO KICH THUOC DANG CAI DAT TRONG CODE ---"));
        Serial.print(F(" Steps/mm: X="));
        Serial.print(STEPS_PER_MM_X, 1);
        Serial.print(F(" | Y="));
        Serial.print(STEPS_PER_MM_Y, 1);
        Serial.print(F(" | Z="));
        Serial.println(STEPS_PER_MM_Z, 1);
        Serial.print(F(" Goc xuat phat gia cong: X="));
        Serial.print(SHAPE_START_X_MM, 1);
        Serial.print(F("mm, Y="));
        Serial.print(SHAPE_START_Y_MM, 1);
        Serial.println(F("mm"));
        Serial.print(F(" Safe Z = "));
        Serial.print(SHAPE_SAFE_Z_MM, 1);
        Serial.print(F("mm | Work Z = "));
        Serial.print(SHAPE_WORK_Z_MM, 1);
        Serial.println(F("mm"));
        Serial.print(F(" Hinh Vuong: Canh "));
        Serial.print(SQUARE_SIZE_MM, 1);
        Serial.print(F("mm @ Feed "));
        Serial.print(SQUARE_FEEDRATE, 0);
        Serial.println(F(" mm/min"));
        Serial.print(F(" Hinh Tam Giac: Canh "));
        Serial.print(TRIANGLE_SIDE_MM, 1);
        Serial.print(F("mm @ Feed "));
        Serial.print(TRIANGLE_FEEDRATE, 0);
        Serial.println(F(" mm/min"));
        Serial.print(F(" Hinh Tron: Ban kinh "));
        Serial.print(CIRCLE_RADIUS_MM, 1);
        Serial.print(F("mm @ Feed "));
        Serial.print(CIRCLE_FEEDRATE, 0);
        Serial.println(F(" mm/min"));
        Serial.print(F(" 3D Helix: R="));
        Serial.print(PROFILE_3D_RADIUS_MM, 1);
        Serial.print(F("mm, Sau="));
        Serial.print(PROFILE_3D_DEPTH_MM, 1);
        Serial.print(F("mm, "));
        Serial.print(PROFILE_3D_TURNS);
        Serial.print(F(" vong @ Feed "));
        Serial.print(PROFILE_3D_FEEDRATE, 0);
        Serial.println(F(" mm/min"));
        break;

      default:
        Serial.println(F("[!] Lenh khong hop le."));
        break;
    }
  }
}

// =============================================================================
// 18. TIỆN ÍCH TRUYỀN THÔNG SERIAL (UTILITIES)
// =============================================================================
char read_char_blocking() {
  flush_serial_input();
  while (true) {
    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c != '\r' && c != '\n' && c != ' ') {
        return c;
      }
    }
    delay(5);
  }
}

void flush_serial_input() {
  while (Serial.available() > 0) {
    Serial.read();
    delay(2);
  }
}
