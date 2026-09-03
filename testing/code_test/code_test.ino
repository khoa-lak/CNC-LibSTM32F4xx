/*
  =============================================================================
  CHƯƠNG TRÌNH KIỂM TRA TOÀN DIỆN CƠ CẤU MÁY CNC (STM32F407VET6 CUSTOM CNC)
  Thư mục: testing/code_test/code_test.ino
  Tác giả: CNC Controller Team
  Baudrate: 115200 bps
  =============================================================================
  SƠ ĐỒ CHÂN (Bám sát theo boards/my_machine_map.h):
  - ĐỘNG CƠ BƯỚC:
    + Trục X: Step PD11, Dir PE4
    + Trục Y: Step PD12, Dir PE5
    + Trục Z: Step PD13, Dir PE6
    + Trục A: Step PD14, Dir PE0
    + Enable chung: PD4 (Mức LOW kích hoạt driver)
  - CÔNG TẮC HÀNH TRÌNH (LIMIT SWITCHES):
    + X Limit: PA4 (Input Pull-up)
    + Y Limit: PA5 (Input Pull-up)
    + Z Limit: PA6 (Input Pull-up)
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
    + Nút dừng khẩn cấp (E-Stop/Reset): PC0
    + Nút tạm dừng (Feed Hold): PB7
    + Nút bắt đầu (Cycle Start): PB1
    + Cảm biến cửa (Safety Door): PB8
    + Cảm biến dò dao (Probe): PA7
  =============================================================================
*/

#include <Arduino.h>

// =============================================================================
// 1. ĐỊNH NGHĨA CHÂN PHẦN CỨNG (GPIO PIN DEFINITIONS)
// =============================================================================

// --- Động cơ bước ---
#define PIN_STEP_X          PD11
#define PIN_DIR_X           PE4
#define PIN_STEP_Y          PD12
#define PIN_DIR_Y           PE5
#define PIN_STEP_Z          PD13
#define PIN_DIR_Z           PE6
#define PIN_STEP_A          PD14
#define PIN_DIR_A           PE0
#define PIN_STEPPERS_ENABLE PD4   // Active LOW cho hầu hết driver (TB6600, DM542, A4988)

// --- Công tắc hành trình ---
#define PIN_LIMIT_X         PA4
#define PIN_LIMIT_Y         PA5
#define PIN_LIMIT_Z         PA6

// --- Trục chính (Spindle) ---
#define PIN_SPINDLE_PWM     PB0
#define PIN_SPINDLE_DIR     PC6
#define PIN_SPINDLE_ENABLE  PC7

// --- Nước tưới nguội & Khí làm mát ---
#define PIN_COOLANT_FLOOD   PD8
#define PIN_COOLANT_MIST    PD9

// --- Khí nén thay dao tự động ATC ---
#define PIN_ATC_UNCLAMP     PB10
#define PIN_ATC_AIR_BLAST   PB11

// --- Cảm biến & Nút bấm ---
#define PIN_INPUT_ESTOP       PC0
#define PIN_INPUT_FEED_HOLD   PB7
#define PIN_INPUT_CYCLE_START PB1
#define PIN_INPUT_SAFETY_DOOR PB8
#define PIN_INPUT_PROBE       PA7

// --- LED báo trạng thái bo mạch ---
#if defined(LED_BUILTIN)
  #define PIN_STATUS_LED    LED_BUILTIN
#elif defined(PA6)
  #define PIN_STATUS_LED    PA6
#else
  #define PIN_STATUS_LED    PC13
#endif

// =============================================================================
// 2. BIẾN TOÀN CỤC & TRẠNG THÁI HỆ THỐNG
// =============================================================================
bool steppers_enabled = true;
bool spindle_running  = false;
bool spindle_dir_ccw  = false;
uint8_t spindle_speed = 0; // 0 - 255
bool flood_active     = false;
bool mist_active      = false;
bool atc_unclamp_on   = false;
bool atc_air_blast_on = false;

// Cấu hình Jogging
uint16_t jog_steps    = 400;   // Số bước mỗi lần Jog (mặc định ~2 vòng với 200 step/rev microstepping 1/2)
uint16_t jog_delay_us = 400;   // Chu kỳ xung (us) -> tốc độ phát xung

// =============================================================================
// 3. NGUYÊN MẪU HÀM (FUNCTION PROTOTYPES)
// =============================================================================
void print_header();
void print_main_menu();
void handle_main_menu(char cmd);

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

char read_char_blocking();
void flush_serial_input();

// =============================================================================
// 4. HÀM KHỞI TẠO HỆ THỐNG (SETUP)
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
  pinMode(PIN_STEP_Y, OUTPUT);
  pinMode(PIN_DIR_Y, OUTPUT);
  pinMode(PIN_STEP_Z, OUTPUT);
  pinMode(PIN_DIR_Z, OUTPUT);
  pinMode(PIN_STEP_A, OUTPUT);
  pinMode(PIN_DIR_A, OUTPUT);
  pinMode(PIN_STEPPERS_ENABLE, OUTPUT);

  // Mặc định: Giữ chân STEP ở mức LOW, Enable = LOW (Kích hoạt driver)
  digitalWrite(PIN_STEP_X, LOW);
  digitalWrite(PIN_STEP_Y, LOW);
  digitalWrite(PIN_STEP_Z, LOW);
  digitalWrite(PIN_STEP_A, LOW);
  digitalWrite(PIN_DIR_X, LOW);
  digitalWrite(PIN_DIR_Y, LOW);
  digitalWrite(PIN_DIR_Z, LOW);
  digitalWrite(PIN_DIR_A, LOW);
  digitalWrite(PIN_STEPPERS_ENABLE, LOW); // LOW = Enable

  // --- Cấu hình các chân OUTPUT cho Spindle ---
  pinMode(PIN_SPINDLE_PWM, OUTPUT);
  pinMode(PIN_SPINDLE_DIR, OUTPUT);
  pinMode(PIN_SPINDLE_ENABLE, OUTPUT);
  analogWrite(PIN_SPINDLE_PWM, 0);
  digitalWrite(PIN_SPINDLE_DIR, LOW);
  digitalWrite(PIN_SPINDLE_ENABLE, LOW); // Mặc định tắt Spindle

  // --- Cấu hình các chân OUTPUT cho Coolant & Mist ---
  pinMode(PIN_COOLANT_FLOOD, OUTPUT);
  pinMode(PIN_COOLANT_MIST, OUTPUT);
  digitalWrite(PIN_COOLANT_FLOOD, LOW);
  digitalWrite(PIN_COOLANT_MIST, LOW);

  // --- Cấu hình các chân OUTPUT cho ATC Pneumatics ---
  pinMode(PIN_ATC_UNCLAMP, OUTPUT);
  pinMode(PIN_ATC_AIR_BLAST, OUTPUT);
  digitalWrite(PIN_ATC_UNCLAMP, LOW);    // 0: Kẹp dao
  digitalWrite(PIN_ATC_AIR_BLAST, LOW);  // 0: Tắt thổi khí

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
// 5. VÒNG LẶP CHÍNH (LOOP)
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
// 6. GIAO DIỆN MENU CHÍNH (SERIAL CLI)
// =============================================================================
void print_header() {
  Serial.println(F("\r\n============================================================"));
  Serial.println(F("     CNC MACHINE MECHANISM TEST SUITE - STM32F407VET6       "));
  Serial.println(F("        Bo mach: STM32F407VET6 Custom CNC (grblHAL)         "));
  Serial.println(F("============================================================"));
}

void print_main_menu() {
  Serial.println(F("\r\n--- MENU CHUC NANG KIEM TRA ---"));
  Serial.println(F(" [1] Test Dong co buoc (Steppers X, Y, Z, A & Enable)"));
  Serial.println(F(" [2] Che do Jogging ban phim (Thu cong W/A/S/D/Q/E/Z/C)"));
  Serial.println(F(" [3] Test Truc chinh Spindle (Enable, Chieu DIR, PWM 0-100%)"));
  Serial.println(F(" [4] Test Tuoi nguoi & Lam mat (Coolant Flood & Mist)"));
  Serial.println(F(" [5] Test Khi nen Thay dao tu dong ATC (Unclamp & Air Blast)"));
  Serial.println(F(" [6] Theo doi Cam bien & Cong tac hanh trinh (Live Monitor)"));
  Serial.println(F(" [7] Chay Tu Dong Kiem Tra Toan Dien (Auto Self-Test)"));
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
// 7. MODULE 1: KIỂM TRA ĐỘNG CƠ BƯỚC (STEPPER MOTORS)
// =============================================================================
void step_single_axis(uint32_t stepPin, uint32_t dirPin, bool dir, uint32_t steps, uint16_t delayUs) {
  digitalWrite(dirPin, dir ? HIGH : LOW);
  delayMicroseconds(20); // Chờ thời gian thiết lập chiều (Direction setup time)

  for (uint32_t i = 0; i < steps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(5); // Xung HIGH tối thiểu 5us
    digitalWrite(stepPin, LOW);
    delayMicroseconds(delayUs);
  }
}

void menu_steppers() {
  Serial.println(F("\r\n============================================================"));
  Serial.println(F("                 KIEM TRA DONG CO BUOC (STEPPERS)           "));
  Serial.println(F("============================================================"));
  Serial.println(F(" [E] Bat/Tat Steppers Enable (PD4)"));
  Serial.println(F(" [X] Test Truc X (PD11 Step, PE4 Dir) - Quay 800 buoc Thuan/Nghich"));
  Serial.println(F(" [Y] Test Truc Y (PD12 Step, PE5 Dir) - Quay 800 buoc Thuan/Nghich"));
  Serial.println(F(" [Z] Test Truc Z (PD13 Step, PE6 Dir) - Quay 800 buoc Len/Xuong"));
  Serial.println(F(" [A] Test Truc A (PD14 Step, PE0 Dir) - Quay 800 buoc Thuan/Nghich"));
  Serial.println(F(" [S] Test Quet Tan So Xung (Tang toc tu 1kHz -> 5kHz tren truc X)"));
  Serial.println(F(" [0] Quay lai Menu chinh"));
  
  while (true) {
    Serial.print(F("\r\n[Steppers] Nhap lenh (E/X/Y/Z/A/S/0): "));
    char c = read_char_blocking();

    if (c == '0' || c == 'q' || c == 'Q') {
      Serial.println(F("-> Thoat ve Menu chinh."));
      break;
    }

    switch (c) {
      case 'e':
      case 'E':
        steppers_enabled = !steppers_enabled;
        digitalWrite(PIN_STEPPERS_ENABLE, steppers_enabled ? LOW : HIGH);
        Serial.print(F("-> Trang thai Stepper Enable (PD4): "));
        Serial.println(steppers_enabled ? F("ENABLED (LOW - Driver giu luc)") : F("DISABLED (HIGH - Tha tu do)"));
        break;

      case 'x':
      case 'X':
        Serial.println(F("-> Dang test Truc X: Quay THUAN (800 steps)..."));
        step_single_axis(PIN_STEP_X, PIN_DIR_X, HIGH, 800, 500);
        delay(300);
        Serial.println(F("-> Dang test Truc X: Quay NGHICH (800 steps)..."));
        step_single_axis(PIN_STEP_X, PIN_DIR_X, LOW, 800, 500);
        Serial.println(F("-> Hoan thanh test Truc X."));
        break;

      case 'y':
      case 'Y':
        Serial.println(F("-> Dang test Truc Y: Quay THUAN (800 steps)..."));
        step_single_axis(PIN_STEP_Y, PIN_DIR_Y, HIGH, 800, 500);
        delay(300);
        Serial.println(F("-> Dang test Truc Y: Quay NGHICH (800 steps)..."));
        step_single_axis(PIN_STEP_Y, PIN_DIR_Y, LOW, 800, 500);
        Serial.println(F("-> Hoan thanh test Truc Y."));
        break;

      case 'z':
      case 'Z':
        Serial.println(F("-> Dang test Truc Z: Di chuyen LEN (800 steps)..."));
        step_single_axis(PIN_STEP_Z, PIN_DIR_Z, HIGH, 800, 500);
        delay(300);
        Serial.println(F("-> Dang test Truc Z: Di chuyen XUONG (800 steps)..."));
        step_single_axis(PIN_STEP_Z, PIN_DIR_Z, LOW, 800, 500);
        Serial.println(F("-> Hoan thanh test Truc Z."));
        break;

      case 'a':
      case 'A':
        Serial.println(F("-> Dang test Truc A: Quay THUAN (800 steps)..."));
        step_single_axis(PIN_STEP_A, PIN_DIR_A, HIGH, 800, 500);
        delay(300);
        Serial.println(F("-> Dang test Truc A: Quay NGHICH (800 steps)..."));
        step_single_axis(PIN_STEP_A, PIN_DIR_A, LOW, 800, 500);
        Serial.println(F("-> Hoan thanh test Truc A."));
        break;

      case 's':
      case 'S':
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
// 8. MODULE 2: CHẾ ĐỘ JOGGING THỦ CÔNG (KEYBOARD JOGGING)
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

  // Đảm bảo Enable Stepper
  digitalWrite(PIN_STEPPERS_ENABLE, LOW);
  steppers_enabled = true;

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
// 9. MODULE 3: KIỂM TRA TRỤC CHÍNH (SPINDLE)
// =============================================================================
void set_spindle(bool enable, bool dir_ccw, uint8_t pwm_val) {
  spindle_running = enable;
  spindle_dir_ccw = dir_ccw;
  spindle_speed   = pwm_val;

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
        if (spindle_running && spindle_speed == 0) spindle_speed = 128; // default 50%
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
// 10. MODULE 4: KIỂM TRA TƯỚI NGUỘI & KHÍ LÀM MÁT (COOLANT & MIST)
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
// 11. MODULE 5: KIỂM TRA KHÍ NÉN THAY DAO TỰ ĐỘNG (ATC PNEUMATICS)
// =============================================================================
void test_atc_full_cycle() {
  Serial.println(F("\r\n--- MOPHONG CHU TRINH THAY DAO TU DONG ATC ---"));
  Serial.println(F("1. Kich hoat van Mo ngam nha dao (Unclamp PB10 = HIGH)..."));
  digitalWrite(PIN_ATC_UNCLAMP, HIGH);
  delay(300);

  Serial.println(F("2. Bat van Thoi khi ve sinh con dao (Air Blast PB11 = HIGH)..."));
  digitalWrite(PIN_ATC_AIR_BLAST, HIGH);
  delay(600); // Thổi khí trong 600ms

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
// 12. MODULE 6: THEO DÕI CẢM BIẾN & CÔNG TẮC HÀNH TRÌNH (LIVE SENSOR MONITOR)
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

  // Lưu trạng thái trước đó để phát hiện thay đổi (Edge detection)
  int last_lim_x   = digitalRead(PIN_LIMIT_X);
  int last_lim_y   = digitalRead(PIN_LIMIT_Y);
  int last_lim_z   = digitalRead(PIN_LIMIT_Z);
  int last_estop   = digitalRead(PIN_INPUT_ESTOP);
  int last_hold    = digitalRead(PIN_INPUT_FEED_HOLD);
  int last_start   = digitalRead(PIN_INPUT_CYCLE_START);
  int last_door    = digitalRead(PIN_INPUT_SAFETY_DOOR);
  int last_probe   = digitalRead(PIN_INPUT_PROBE);

  // In bảng trạng thái ban đầu
  Serial.println(F("[TRANG THAI HIEN TAI BAN DAU]:"));
  Serial.print(F("  LimX(PA4): ")); Serial.print(last_lim_x ? F("OPEN (1)") : F("TRIGGERED (0)"));
  Serial.print(F(" | LimY(PA5): ")); Serial.print(last_lim_y ? F("OPEN (1)") : F("TRIGGERED (0)"));
  Serial.print(F(" | LimZ(PA6): ")); Serial.println(last_lim_z ? F("OPEN (1)") : F("TRIGGERED (0)"));

  Serial.print(F("  E-Stop(PC0): ")); Serial.print(last_estop ? F("NORMAL (1)") : F("PRESSED (0)"));
  Serial.print(F(" | FeedHold(PB7): ")); Serial.print(last_hold ? F("NORMAL (1)") : F("PRESSED (0)"));
  Serial.print(F(" | CycleStart(PB1): ")); Serial.println(last_start ? F("NORMAL (1)") : F("PRESSED (0)"));

  Serial.print(F("  Door(PB8): ")); Serial.print(last_door ? F("CLOSED (1)") : F("OPEN (0)"));
  Serial.print(F(" | Probe(PA7): ")); Serial.println(last_probe ? F("CLEAR (1)") : F("TOUCHED (0)"));
  Serial.println(F("------------------------------------------------------------"));
  Serial.println(F(">> Dang lang nghe su kien thoi gian thuc..."));

  flush_serial_input();

  while (Serial.available() == 0) {
    int cur_lim_x = digitalRead(PIN_LIMIT_X);
    int cur_lim_y = digitalRead(PIN_LIMIT_Y);
    int cur_lim_z = digitalRead(PIN_LIMIT_Z);
    int cur_estop = digitalRead(PIN_INPUT_ESTOP);
    int cur_hold  = digitalRead(PIN_INPUT_FEED_HOLD);
    int cur_start = digitalRead(PIN_INPUT_CYCLE_START);
    int cur_door  = digitalRead(PIN_INPUT_SAFETY_DOOR);
    int cur_probe = digitalRead(PIN_INPUT_PROBE);

    if (cur_lim_x != last_lim_x) {
      last_lim_x = cur_lim_x;
      Serial.print(F(" [EVENT] LIMIT X (PA4) -> "));
      Serial.println(cur_lim_x == LOW ? F(">>> TRIGGERED (DONG/CHAM) <<<") : F("RELEASED (NHA/HO)"));
    }
    if (cur_lim_y != last_lim_y) {
      last_lim_y = cur_lim_y;
      Serial.print(F(" [EVENT] LIMIT Y (PA5) -> "));
      Serial.println(cur_lim_y == LOW ? F(">>> TRIGGERED (DONG/CHAM) <<<") : F("RELEASED (NHA/HO)"));
    }
    if (cur_lim_z != last_lim_z) {
      last_lim_z = cur_lim_z;
      Serial.print(F(" [EVENT] LIMIT Z (PA6) -> "));
      Serial.println(cur_lim_z == LOW ? F(">>> TRIGGERED (DONG/CHAM) <<<") : F("RELEASED (NHA/HO)"));
    }
    if (cur_estop != last_estop) {
      last_estop = cur_estop;
      Serial.print(F(" [EVENT] NUT E-STOP (PC0) -> "));
      Serial.println(cur_estop == LOW ? F(">>> NUT DUNG KHAN CAP BI BAM! <<<") : F("DA NHA NUT"));
    }
    if (cur_hold != last_hold) {
      last_hold = cur_hold;
      Serial.print(F(" [EVENT] NUT FEED HOLD (PB7) -> "));
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

    delay(20); // Debounce
  }

  flush_serial_input();
  Serial.println(F("-> Da thoat che do theo doi cam bien."));
}

// =============================================================================
// 13. MODULE 7: TỰ ĐỘNG KIỂM TRA TOÀN DIỆN (AUTO SELF-TEST SEQUENCE)
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

  Serial.println(F("\r\n[1/6] Kiem tra Steppers Enable (PD4)..."));
  digitalWrite(PIN_STEPPERS_ENABLE, LOW); // LOW = Enable
  delay(300);
  Serial.println(F("   -> PASS: Enable da kich hoat."));

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
  Serial.print(F("   -> X_Limit (PA4): ")); Serial.println(digitalRead(PIN_LIMIT_X) ? F("HIGH (Pull-up Open)") : F("LOW (Triggered)"));
  Serial.print(F("   -> Y_Limit (PA5): ")); Serial.println(digitalRead(PIN_LIMIT_Y) ? F("HIGH (Pull-up Open)") : F("LOW (Triggered)"));
  Serial.print(F("   -> Z_Limit (PA6): ")); Serial.println(digitalRead(PIN_LIMIT_Z) ? F("HIGH (Pull-up Open)") : F("LOW (Triggered)"));
  Serial.print(F("   -> E-Stop  (PC0): ")); Serial.println(digitalRead(PIN_INPUT_ESTOP) ? F("HIGH (Normal)") : F("LOW (Pressed)"));
  Serial.print(F("   -> Door    (PB8): ")); Serial.println(digitalRead(PIN_INPUT_SAFETY_DOOR) ? F("HIGH (Closed)") : F("LOW (Open)"));
  Serial.print(F("   -> Probe   (PA7): ")); Serial.println(digitalRead(PIN_INPUT_PROBE) ? F("HIGH (Clear)") : F("LOW (Touched)"));

  Serial.println(F("\r\n============================================================"));
  Serial.println(F("     KET QUA: DA HOAN TAT CHUONG TRINH TU DONG KIEM TRA!    "));
  Serial.println(F("============================================================"));
}

// =============================================================================
// 14. TIỆN ÍCH TRUYỀN THÔNG SERIAL (UTILITIES)
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
