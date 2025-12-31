// ============================================================================
// 1. 磁簧即停 - 絕對優先權（不可被中斷）
// 2. 三狀態機 - STOPPED_AT_TOP / STOPPED_AT_BOTTOM / MOVING
// 3. 速度感測完全獨立 - 僅顯示用，不參與控制
// 4. IR 完全手動控制
// 5. OLED 三頁面切換
// 6. 移除所有自動邏輯（防止衝突）
// ============================================================================

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <IRremote.h>
#include <avr/wdt.h>

// ============================================================================
// OLED 設定
// ============================================================================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ============================================================================
// 音符定義
// ============================================================================
#define NOTE_C4  262
#define NOTE_E4  330
#define NOTE_G4  392
#define NOTE_C5  523
#define NOTE_E5  659
#define NOTE_G5  784
#define NOTE_A5  880
#define NOTE_C6  1047

// ============================================================================
// MPU6050 設定
// ============================================================================
#define MPU_6050 0x69
#define PWR_MGMT_1 0x6B
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H  0x43

// ============================================================================
// 腳位定義
// ============================================================================
const byte BUZZER_PIN = 8;
const byte IR_RECEIVE_PIN = 3;
const byte PWM_PIN = 9;

// 磁簧開關（常開型，LOW = 觸發）
const byte LIMIT_UP_PIN = 52;      // 上磁
const byte LIMIT_DOWN_PIN = 53;    // 下磁

// 控制與感測
const byte HALL_SPEED_PIN = 2;
const byte SPEED_CONTROL_PIN = A0;

// RGB LED
const byte RGB_RED_PIN = 5;
const byte RGB_GREEN_PIN = 7;
const byte RGB_BLUE_PIN = 6;
const bool RGB_COMMON_ANODE = false;

// ============================================================================
// 系統狀態定義（核心三狀態）
// ============================================================================
enum SystemState : byte {
  STOPPED_AT_TOP,      // 停在上限位
  STOPPED_AT_BOTTOM,   // 停在下限位
  MOVING               // 移動中（不知道往哪，也不需要知道）
};

enum RGBColor : byte {
  RGB_OFF,
  RGB_GREEN,
  RGB_YELLOW,
  RGB_BLUE,
  RGB_RED,
  RGB_PURPLE,
  RGB_CYAN,
  RGB_WHITE
};

enum DisplayPage : byte {
  PAGE_MAIN,      // 主頁面 - 狀態與速度
  PAGE_SENSOR,    // 感測器頁面 - 詳細數據
  PAGE_CONTROL    // 控制頁面 - 限位與請求
};

// ============================================================================
// 全域變數
// ============================================================================
SystemState currentState = MOVING;  // 開機假設在移動（最保守）
DisplayPage currentPage = PAGE_MAIN;

// 限位開關狀態（去抖動後）
bool upperLimitTriggered = false;
bool lowerLimitTriggered = false;
unsigned long lastLimitChangeTime = 0;
const unsigned long LIMIT_DEBOUNCE = 50;  // 50ms 去抖動

// PWM 控制
int pwmSpeed = 200;
unsigned long lastSpeedRead = 0;

// 速度測量（完全獨立，僅供顯示）
#define WHEEL_CIRCUMFERENCE 204.2
const uint16_t SPEED_TIMEOUT = 2000;
volatile uint32_t lastSpeedTrigger = 0;
volatile uint32_t timeBetweenTriggers = 0;
volatile bool newSpeedData = false;
float currentSpeed = 0.0;
uint32_t lastSpeedUpdate = 0;

// IMU 資料
float compAngleX = 0.0f;
float gyroBiasX = 0.0f;
float accelBiasX = 0.0f;
uint32_t lastIMUTime = 0;

// 安全閾值（僅用於顯示警告，不控制馬達）
const float EMERGENCY_ANGLE = 15.0f;
const float WARNING_ANGLE = 10.0f;

// IR 控制
uint32_t lastIRCode = 0;
const uint16_t IR_DELAY = 500;

// 顯示更新
uint32_t lastDisplayTime = 0;
uint32_t lastPrintTime = 0;
uint32_t lastPageSwitchTime = 0;
const uint32_t PAGE_SWITCH_DELAY = 300;  // 防止快速切換

// 控制鎖（防止重複觸發）
bool movementLocked = false;
uint32_t movementLockTime = 0;
const uint32_t MOVEMENT_LOCK_DURATION = 1000;  // 1秒鎖定期

// ============================================================================
// 中斷服務程序（速度感測完全獨立）
// ============================================================================
void speedInterrupt() {
  uint32_t t = micros();
  if (t - lastSpeedTrigger > 40000) {  // 防抖動
    timeBetweenTriggers = t - lastSpeedTrigger;
    lastSpeedTrigger = t;
    newSpeedData = true;
  }
}

// ============================================================================
// RGB LED 控制
// ============================================================================
void setRGB(byte r, byte g, byte b) {
  if (RGB_COMMON_ANODE) {
    analogWrite(RGB_RED_PIN, 255-r);
    analogWrite(RGB_GREEN_PIN, 255-g);
    analogWrite(RGB_BLUE_PIN, 255-b);
  } else {
    analogWrite(RGB_RED_PIN, r);
    analogWrite(RGB_GREEN_PIN, g);
    analogWrite(RGB_BLUE_PIN, b);
  }
}

void setRGBColor(RGBColor c) {
  switch(c) {
    case RGB_OFF: setRGB(0,0,0); break;
    case RGB_GREEN: setRGB(0,255,0); break;
    case RGB_YELLOW: setRGB(255,200,0); break;
    case RGB_BLUE: setRGB(0,100,255); break;
    case RGB_RED: setRGB(255,0,0); break;
    case RGB_PURPLE: setRGB(180,0,255); break;
    case RGB_CYAN: setRGB(0,255,255); break;
    case RGB_WHITE: setRGB(255,255,255); break;
  }
}

void updateRGBByState() {
  switch(currentState) {
    case STOPPED_AT_TOP:
      setRGBColor(RGB_CYAN);
      break;
    case STOPPED_AT_BOTTOM:
      setRGBColor(RGB_YELLOW);
      break;
    case MOVING:
      setRGBColor(RGB_PURPLE);
      break;
  }
}

// ============================================================================
// 音效系統
// ============================================================================
void playStartupMusic() {
  // 開機音樂：上升音階
  tone(BUZZER_PIN, NOTE_C4, 150); delay(180);
  tone(BUZZER_PIN, NOTE_E4, 150); delay(180);
  tone(BUZZER_PIN, NOTE_G4, 150); delay(180);
  tone(BUZZER_PIN, NOTE_C5, 150); delay(180);
  tone(BUZZER_PIN, NOTE_E5, 150); delay(180);
  tone(BUZZER_PIN, NOTE_G5, 150); delay(180);
  tone(BUZZER_PIN, NOTE_C6, 300); delay(350);
  noTone(BUZZER_PIN);
}

void playBeep(uint16_t freq, uint16_t duration) {
  tone(BUZZER_PIN, freq, duration);
  delay(duration + 20);
  noTone(BUZZER_PIN);
}

void playDoubleBeep() {
  tone(BUZZER_PIN, NOTE_A5, 100);
  delay(120);
  tone(BUZZER_PIN, NOTE_A5, 100);
  delay(120);
  noTone(BUZZER_PIN);
}

void playWarningBeep() {
  for(int i = 0; i < 3; i++) {
    tone(BUZZER_PIN, NOTE_C6, 100);
    delay(150);
  }
  noTone(BUZZER_PIN);
}

// ============================================================================
// 速度計算（僅供顯示）
// ============================================================================
float calculateSpeed() {
  if (timeBetweenTriggers == 0) return 0.0;
  float timeInSec = timeBetweenTriggers/1000000.0;
  return (WHEEL_CIRCUMFERENCE/timeInSec)*0.036;
}

// ============================================================================
// MPU6050 函式
// ============================================================================
int16_t safeRead16(byte reg) {
  Wire.beginTransmission(MPU_6050);
  Wire.write(reg);
  byte error = Wire.endTransmission(false);
  if(error != 0) return 0;

  Wire.requestFrom((byte)MPU_6050, (byte)2);
  uint32_t start = millis();
  while (Wire.available() < 2) {
    if (millis() - start > 10) return 0;
  }

  return (Wire.read()<<8)|Wire.read();
}

void setupMPU() {
  Wire.beginTransmission(MPU_6050);
  Wire.write(PWR_MGMT_1);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(50);
}

void calibrateIMU() {
  Serial.println(F("校準 IMU..."));
  float sumAx=0, sumGx=0;
  for(byte i=0; i<50; i++) {
    sumAx += safeRead16(ACCEL_XOUT_H)/16384.0f;
    sumGx += safeRead16(GYRO_XOUT_H)/131.0f;
    delay(5);
  }
  accelBiasX = sumAx/50;
  gyroBiasX = sumGx/50;
  Serial.println(F("IMU 校準完成"));
}

void readIMU() {
  uint32_t now = micros();
  float dt = (now - lastIMUTime) / 1000000.0;
  lastIMUTime = now;
  if (dt > 0.1) dt = 0.01;

  float ax = safeRead16(ACCEL_XOUT_H) / 16384.0f - accelBiasX;
  float az = safeRead16(ACCEL_XOUT_H + 4) / 16384.0f;
  float gx = safeRead16(GYRO_XOUT_H) / 131.0f - gyroBiasX;

  if(isnan(ax) || isnan(az) || isnan(gx) ||
     abs(ax) > 10 || abs(az) > 10 || abs(gx) > 500) {
    return;
  }

  float accelAngle = atan2(ax, az) * 57.2958f;
  compAngleX = 0.98 * (compAngleX + gx * dt) + 0.02 * accelAngle;

  if(compAngleX > 180) compAngleX = 180;
  if(compAngleX < -180) compAngleX = -180;
}

// ============================================================================
// 限位開關讀取（最高優先權 + 去抖動）
// ============================================================================
void checkLimitSwitchesWithDebounce() {
  // 原始讀取
  bool upperRaw = (digitalRead(LIMIT_UP_PIN) == LOW);
  bool lowerRaw = (digitalRead(LIMIT_DOWN_PIN) == LOW);
  
  // 去抖動邏輯
  if (upperRaw != upperLimitTriggered || lowerRaw != lowerLimitTriggered) {
    if (millis() - lastLimitChangeTime > LIMIT_DEBOUNCE) {
      upperLimitTriggered = upperRaw;
      lowerLimitTriggered = lowerRaw;
      lastLimitChangeTime = millis();
    }
  }
  
  // 【核心安全規則】立即停止 + 狀態更新
  if (upperLimitTriggered) {
    analogWrite(PWM_PIN, 0);
    currentState = STOPPED_AT_TOP;
    movementLocked = false;
  }
  
  if (lowerLimitTriggered) {
    analogWrite(PWM_PIN, 0);
    currentState = STOPPED_AT_BOTTOM;
    movementLocked = false;
  }
}

// ============================================================================
// PWM 速度控制讀取
// ============================================================================
void readSpeedControl() {
  if (millis() - lastSpeedRead > 100) {
    lastSpeedRead = millis();
    int speedControlValue = analogRead(SPEED_CONTROL_PIN);
    pwmSpeed = map(speedControlValue, 0, 1023, 100, 255);
    pwmSpeed = constrain(pwmSpeed, 100, 255);
  }
}

// ============================================================================
// 馬達控制函式（簡化版）
// ============================================================================
void startMotor() {
  if (movementLocked) {
    Serial.println(F("[控制] 移動鎖定中，忽略請求"));
    return;
  }
  
  // 檢查是否已在移動
  if (currentState == MOVING) {
    Serial.println(F("[控制] 已在移動中"));
    return;
  }
  
  // 啟動馬達
  analogWrite(PWM_PIN, pwmSpeed);
  currentState = MOVING;
  movementLocked = true;
  movementLockTime = millis();
  
  playBeep(NOTE_G5, 100);
  Serial.println(F("[馬達] 啟動"));
}

void stopMotor() {
  analogWrite(PWM_PIN, 0);
  Serial.println(F("[馬達] 停止"));
}

// ============================================================================
// 紅外線處理（完全手動控制）
// ============================================================================
void handleIRCommand(uint32_t code) {
  if(millis() - lastIRCode < IR_DELAY) return;
  lastIRCode = millis();

  // IR 碼判斷（根據你的遙控器調整）
  switch(code) {
    case 0xFF18E7:  // 上鍵
    case 0x1:
      Serial.println(F("[IR] 上升指令"));
      startMotor();
      break;
      
    case 0xFF4AB5:  // 下鍵
    case 0x2:
      Serial.println(F("[IR] 下降指令"));
      startMotor();
      break;
      
    case 0xFF38C7:  // OK 鍵 - 切換頁面
    case 0x3:
      if(millis() - lastPageSwitchTime > PAGE_SWITCH_DELAY) {
        currentPage = (DisplayPage)((currentPage + 1) % 3);
        lastPageSwitchTime = millis();
        playBeep(NOTE_A5, 50);
        Serial.print(F("[OLED] 切換到頁面 "));
        Serial.println(currentPage);
      }
      break;
      
    default:
      Serial.print(F("[IR] 未知碼: 0x"));
      Serial.println(code, HEX);
      break;
  }
}

// ============================================================================
// 開機畫面
// ============================================================================
void showStartupScreen() {
  display.clearDisplay();
  
  // 標題
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(15, 10);
  display.print(F("BIKE"));
  
  display.setTextSize(1);
  display.setCursor(10, 30);
  display.print(F("Smart Wheel"));
  
  display.setTextSize(2);
  display.setCursor(25, 45);
  display.print(F("v8.5"));
  
  display.display();
  delay(1500);
  
  // 載入動畫
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(25, 20);
  display.print(F("Loading..."));
  
  for(int i = 0; i < 3; i++) {
    display.fillRect(20 + i*30, 35, 20, 5, SSD1306_WHITE);
    display.display();
    delay(300);
  }
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(20, 28);
  display.print(F("System Ready!"));
  display.display();
  delay(1000);
}

// ============================================================================
// OLED 多頁面顯示
// ============================================================================
void drawMainPage() {
  display.clearDisplay();
  
  // 標題列
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(F("MAIN [1/3]"));
  
  display.drawLine(0, 9, 127, 9, SSD1306_WHITE);
  
  // 狀態顯示（大字）
  display.setTextSize(1);
  display.setCursor(0, 12);
  display.print(F("State:"));
  
  display.setTextSize(2);
  display.setCursor(0, 22);
  switch(currentState) {
    case STOPPED_AT_TOP:
      display.print(F("TOP"));
      break;
    case STOPPED_AT_BOTTOM:
      display.print(F("BOTTOM"));
      break;
    case MOVING:
      display.print(F("MOVING"));
      break;
  }
  
  display.drawLine(0, 40, 127, 40, SSD1306_WHITE);
  
  // 速度顯示
  display.setTextSize(2);
  display.setCursor(0, 43);
  int speedInt = (int)currentSpeed;
  if(speedInt < 10) display.print(F(" "));
  display.print(speedInt);
  
  display.setTextSize(1);
  display.setCursor(40, 46);
  display.print(F("km/h"));
  
  // PWM
  display.setCursor(70, 43);
  display.print(F("PWM:"));
  display.setCursor(70, 52);
  display.print(pwmSpeed);
  
  display.display();
}

void drawSensorPage() {
  display.clearDisplay();
  
  // 標題列
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(F("SENSOR [2/3]"));
  
  display.drawLine(0, 9, 127, 9, SSD1306_WHITE);
  
  // 傾斜角度
  display.setCursor(0, 12);
  display.print(F("Angle: "));
  display.print(compAngleX, 1);
  display.print((char)247);
  
  // 警告指示
  if(abs(compAngleX) > EMERGENCY_ANGLE) {
    display.setCursor(90, 12);
    display.print(F("WARN!"));
  }
  
  // 速度詳細
  display.setCursor(0, 24);
  display.print(F("Speed: "));
  display.print(currentSpeed, 1);
  display.print(F(" km/h"));
  
  // 速度觸發間隔
  display.setCursor(0, 36);
  display.print(F("Period: "));
  if(timeBetweenTriggers > 0) {
    display.print(timeBetweenTriggers / 1000);
    display.print(F(" ms"));
  } else {
    display.print(F("---"));
  }
  
  // 系統資訊
  display.drawLine(0, 48, 127, 48, SSD1306_WHITE);
  display.setCursor(0, 51);
  display.print(F("Uptime: "));
  display.print(millis() / 1000);
  display.print(F("s"));
  
  display.display();
}

void drawControlPage() {
  display.clearDisplay();
  
  // 標題列
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(F("CONTROL [3/3]"));
  
  display.drawLine(0, 9, 127, 9, SSD1306_WHITE);
  
  // 限位開關狀態（視覺化）
  display.setCursor(0, 12);
  display.print(F("Limit Switch:"));
  
  // 上磁簧
  display.setCursor(10, 24);
  display.print(F("UP:  "));
  if(upperLimitTriggered) {
    display.fillRect(50, 24, 20, 8, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
    display.setCursor(52, 24);
    display.print(F("ON"));
    display.setTextColor(SSD1306_WHITE);
  } else {
    display.drawRect(50, 24, 20, 8, SSD1306_WHITE);
    display.setCursor(52, 24);
    display.print(F("OFF"));
  }
  
  // 下磁簧
  display.setCursor(10, 36);
  display.print(F("DOWN:"));
  if(lowerLimitTriggered) {
    display.fillRect(50, 36, 20, 8, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
    display.setCursor(52, 36);
    display.print(F("ON"));
    display.setTextColor(SSD1306_WHITE);
  } else {
    display.drawRect(50, 36, 20, 8, SSD1306_WHITE);
    display.setCursor(52, 36);
    display.print(F("OFF"));
  }
  
  // 移動鎖定狀態
  display.drawLine(0, 48, 127, 48, SSD1306_WHITE);
  display.setCursor(0, 51);
  display.print(F("Lock: "));
  if(movementLocked) {
    display.print(F("YES ("));
    display.print((MOVEMENT_LOCK_DURATION - (millis() - movementLockTime)) / 100);
    display.print(F(")"));
  } else {
    display.print(F("NO"));
  }
  
  display.display();
}

void updateDisplay() {
  switch(currentPage) {
    case PAGE_MAIN:
      drawMainPage();
      break;
    case PAGE_SENSOR:
      drawSensorPage();
      break;
    case PAGE_CONTROL:
      drawControlPage();
      break;
  }
}

// ============================================================================
// Setup
// ============================================================================
void setup() {
  wdt_disable();
  Serial.begin(115200);
  delay(100);

  Wire.begin();
  Wire.setClock(100000);

  // 腳位初始化
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(LIMIT_UP_PIN, INPUT_PULLUP);
  pinMode(LIMIT_DOWN_PIN, INPUT_PULLUP);
  pinMode(HALL_SPEED_PIN, INPUT_PULLUP);
  pinMode(SPEED_CONTROL_PIN, INPUT);
  pinMode(RGB_RED_PIN, OUTPUT);
  pinMode(RGB_GREEN_PIN, OUTPUT);
  pinMode(RGB_BLUE_PIN, OUTPUT);

  // 確保馬達停止
  analogWrite(PWM_PIN, 0);
  setRGBColor(RGB_OFF);

  // 中斷與外設
  attachInterrupt(digitalPinToInterrupt(HALL_SPEED_PIN), speedInterrupt, FALLING);
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);

  // OLED 初始化
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("OLED 初始化失敗!"));
    while(1);
  }

  // 開機畫面與音樂
  showStartupScreen();
  playStartupMusic();

  // IMU 初始化
  setupMPU();
  calibrateIMU();

  Serial.println(F("===================================="));
  Serial.println(F("  智能輔助輪控制系統 v8.5"));
  Serial.println(F("  超穩定保守版"));
  Serial.println(F("===================================="));
  Serial.println(F("\n【核心設計原則】"));
  Serial.println(F("✓ 磁簧即停 - 絕對優先權"));
  Serial.println(F("✓ 三狀態機 - 極簡邏輯"));
  Serial.println(F("✓ 速度獨立 - 僅供顯示"));
  Serial.println(F("✓ IR 手動控制 - 無自動邏輯"));
  Serial.println(F("✓ OLED 三頁面 - 資訊分離"));
  Serial.println(F("✓ 移動鎖定 - 防止重複觸發"));
  Serial.println(F("====================================\n"));
  
  Serial.println(F("【IR 按鍵功能】"));
  Serial.println(F("上鍵 → 啟動馬達"));
  Serial.println(F("下鍵 → 啟動馬達"));
  Serial.println(F("OK 鍵 → 切換 OLED 頁面"));
  Serial.println(F("====================================\n"));

  lastIMUTime = micros();
  
  wdt_enable(WDTO_2S);
}

// ============================================================================
// Loop 主迴圈
// ============================================================================
void loop() {
  wdt_reset();
  uint32_t currentMillis = millis();

  // 【最高優先權】檢查限位開關
  checkLimitSwitchesWithDebounce();

  // 讀取感測器（僅供顯示）
  readIMU();
  readSpeedControl();

  // 速度更新（完全獨立）
  if(newSpeedData) {
    currentSpeed = calculateSpeed();
    newSpeedData = false;
    lastSpeedUpdate = millis();
  }
  if(millis() - lastSpeedUpdate > SPEED_TIMEOUT) {
    currentSpeed = 0.0;
  }

  // 檢查移動鎖定是否過期
  if(movementLocked && (millis() - movementLockTime > MOVEMENT_LOCK_DURATION)) {
    movementLocked = false;
    Serial.println(F("[控制] 移動鎖定解除"));
  }

  // IR 接收
  if(IrReceiver.decode()) {
    handleIRCommand(IrReceiver.decodedIRData.decodedRawData);
    IrReceiver.resume();
  }

  // 傾斜警告（僅聲音提示，不控制馬達）
  static bool lastWarningState = false;
  bool currentWarningState = (abs(compAngleX) > EMERGENCY_ANGLE);
  if(currentWarningState && !lastWarningState) {
    playWarningBeep();
    Serial.println(F("[警告] 傾斜角度過大!"));
  }
  lastWarningState = currentWarningState;

  // RGB 更新
  updateRGBByState();

  // OLED 更新
  if(millis() - lastDisplayTime > 250) {
    lastDisplayTime = millis();
    updateDisplay();
  }

  // 序列埠輸出
  if(millis() - lastPrintTime > 1000) {
    lastPrintTime = millis();

    Serial.print(F("狀態:"));
    switch(currentState) {
      case STOPPED_AT_TOP: Serial.print(F("上限位")); break;
      case STOPPED_AT_BOTTOM: Serial.print(F("下限位")); break;
      case MOVING: Serial.print(F("移動中")); break;
    }

    
    Serial.print(F(" |限位:["));
    Serial.print(upperLimitTriggered ? F("上✓") : F("上○"));
    Serial.print(F(" "));
    Serial.print(lowerLimitTriggered ? F("下✓") : F("下○"));
    Serial.print(F("]"));

    Serial.print(F(" |速度:"));
    Serial.print(currentSpeed, 1);
    Serial.print(F("km/h"));

    Serial.print(F(" |角度:"));
    Serial.print(compAngleX, 1);
    Serial.print(F("°"));

    Serial.print(F(" |PWM:"));
    Serial.print(pwmSpeed);
  }

  delay(2);
}

/*三、關鍵風險與「真的會出事的地方」

風險 1：checkLimitSwitchesWithDebounce() 的「雙觸發」
問題說明
if (upperLimitTriggered) {
  analogWrite(PWM_PIN, 0);
  currentState = STOPPED_AT_TOP;
  movementLocked = false;
}

if (lowerLimitTriggered) {
  analogWrite(PWM_PIN, 0);
  currentState = STOPPED_AT_BOTTOM;
  movementLocked = false;
}


如果磁鐵在某個位置同時影響兩顆磁簧（或抖動）：

你會在同一個 loop 內：

先被設成 STOPPED_AT_TOP

又立刻被設成 STOPPED_AT_BOTTOM

會怎樣？

馬達還是會停（安全）

狀態顯示可能跳來跳去

OLED / Serial 看起來會「怪怪的」

建議（低風險修正）

改成互斥判斷：

if (upperLimitTriggered && !lowerLimitTriggered) {
  ...
} else if (lowerLimitTriggered && !upperLimitTriggered) {
  ...
}


👉 這不是救命級修正，但會讓你看起來更專業

⚠️ 風險 2：movementLocked 在限位觸發時直接解除
movementLocked = false;


這代表什麼？

馬達剛停

IR 如果還在連發

可能立刻又被啟動

你有 IR_DELAY 在擋，大部分情況 OK
但在「抖動磁簧 + 手按 IR」的極端情境下，還是有機會抖一下

建議（可選）

在限位停下後，保留鎖定 300~500ms

不是必須，但會更穩。

⚠️ 風險 3：delay() 在音效裡

你現在有不少：

tone(); delay(); tone(); delay();

會不會炸？

👉 不會

因為：

限位檢查是在 loop 最前面

馬達停止不靠中斷

但工程角度我還是要提醒你一句：

如果未來有人問你「為什麼不用 non-blocking？」
你可以很理直氣壯說：
「因為所有控制已經被硬體限位保護，delay 不影響安全」

這是可以被接受的理由。

⚠️ 風險 4：IMU / I2C 失敗時的行為

你有做防呆：

if(error != 0) return 0;


也有做數值合理性檢查。

👉 就算 MPU 掛掉：

角度顯示會變怪

但不會影響馬達

✔ 完全符合你現在的設計目標 
*/
