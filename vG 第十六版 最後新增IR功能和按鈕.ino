// ============================================================================
// 智能輔助輪控制系統 v9.1 - 簡化版
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
#define NOTE_A5  880
#define NOTE_C6  1047

// ============================================================================
// MPU6050 設定(簡化版)
// ============================================================================
#define MPU_ADDR 0x69
#define PWR_MGMT_1 0x6B
#define ACCEL_XOUT_H 0x3B

// ============================================================================
// 腳位定義
// ============================================================================
const byte BUZZER_PIN = 8;
const byte IR_RECEIVE_PIN = 3;
const byte PWM_PIN = 9;

// 磁簧開關(常開型,LOW = 觸發)
const byte LIMIT_UP_PIN = 52;
const byte LIMIT_DOWN_PIN = 53;

// 速度感測
const byte HALL_SPEED_PIN = 2;
const byte SPEED_CONTROL_PIN = A0;

// RGB LED
const byte RGB_RED_PIN = 5;
const byte RGB_GREEN_PIN = 7;
const byte RGB_BLUE_PIN = 6;

// 【新增】實體按鈕 - 用於切換 OLED 頁面
const byte BUTTON_PIN = 12;

// ============================================================================
// 系統狀態(簡化成 4 個,知道方向)
// ============================================================================
enum SystemState : byte {
  AT_TOP,      // 停在上限位
  AT_BOTTOM,   // 停在下限位
  MOVING_UP,   // 正在上升
  MOVING_DOWN  // 正在下降(手動)
};

// ============================================================================
// 全域變數
// ============================================================================
// 開機預設在下方(最安全)
SystemState currentState = AT_BOTTOM;  

// 限位開關
bool upperLimit = false;
bool lowerLimit = false;
unsigned long lastLimitCheck = 0;
const unsigned long LIMIT_DEBOUNCE = 50;

// 系統鎖(統一管理)
bool systemLocked = false;
unsigned long lockStartTime = 0;
const unsigned long LOCK_DURATION = 500;

// PWM 控制
int pwmSpeed = 190;
unsigned long lastSpeedRead = 0;

// 【新增】PWM 手動開關 (IR 遙控器控制)
bool pwmManualOverride = false;  // false = 自動, true = 強制關閉

// 速度測量
#define WHEEL_CIRCUMFERENCE 204.2
volatile uint32_t lastSpeedTrigger = 0;
volatile uint32_t speedPulseInterval = 0;
volatile bool newSpeedData = false;
float currentSpeed = 0.0;
unsigned long lastSpeedUpdate = 0;
const unsigned long SPEED_TIMEOUT = 2000;

// 【核心功能】自動上升邏輯
const float AUTO_LIFT_SPEED = 15.0;
const unsigned long SPEED_HOLD_TIME = 1000;
unsigned long speedAboveThresholdTime = 0;
bool speedTimerRunning = false;
bool autoLiftDone = false;

// IR 控制
unsigned long lastIRTime = 0;
const unsigned long IR_DELAY = 500;

// 【新增】OLED 顯示 - 3 個頁面
byte currentPage = 0;  // 0=主頁, 1=除錯頁, 2=系統資訊頁
unsigned long lastDisplayUpdate = 0;

// 【新增】按鈕控制 - 切換頁面
bool lastButtonState = HIGH;
unsigned long lastButtonPress = 0;
const unsigned long BUTTON_DEBOUNCE = 200;

// 序列埠輸出
unsigned long lastSerialPrint = 0;

// IMU 資料(簡化版)
float tiltAngle = 0.0;
unsigned long lastIMURead = 0;
const unsigned long IMU_READ_INTERVAL = 100;
const float TILT_WARNING = 10.0;
const float TILT_DANGER = 15.0;
bool lastTiltWarning = false;

// ============================================================================
// 速度中斷(獨立測量,不干擾主邏輯)
// ============================================================================
void speedInterrupt() {
  uint32_t now = micros();
  uint32_t interval = now - lastSpeedTrigger;
  
  if (interval > 40000) {
    speedPulseInterval = interval;
    lastSpeedTrigger = now;
    newSpeedData = true;
  }
}

// ============================================================================
// RGB LED(簡化成 3 色)
// ============================================================================
void setRGB(byte r, byte g, byte b) {
  analogWrite(RGB_RED_PIN, r);
  analogWrite(RGB_GREEN_PIN, g);
  analogWrite(RGB_BLUE_PIN, b);
}

void updateRGBByState() {
  if (pwmManualOverride) {
    // PWM 被手動關閉 - 閃爍紅色警告
    static unsigned long lastBlink = 0;
    static bool blinkState = false;
    if (millis() - lastBlink > 300) {
      lastBlink = millis();
      blinkState = !blinkState;
      setRGB(blinkState ? 255 : 0, 0, 0);
    }
    return;
  }
  
  switch(currentState) {
    case AT_TOP:
      setRGB(0, 255, 255);  // 青色
      break;
    case AT_BOTTOM:
      setRGB(255, 200, 0);  // 黃色
      break;
    case MOVING_UP:
      setRGB(0, 255, 0);    // 綠色(上升中)
      break;
    case MOVING_DOWN:
      setRGB(255, 0, 0);    // 紅色(下降中)
      break;
  }
}

// ============================================================================
// 音效(簡化)
// ============================================================================
void playStartupMusic() {
  tone(BUZZER_PIN, NOTE_C4, 150); delay(180);
  tone(BUZZER_PIN, NOTE_E4, 150); delay(180);
  tone(BUZZER_PIN, NOTE_G4, 150); delay(180);
  tone(BUZZER_PIN, NOTE_C5, 300); delay(350);
  noTone(BUZZER_PIN);
}

void playBeep(uint16_t freq, uint16_t duration) {
  tone(BUZZER_PIN, freq, duration);
  delay(duration + 20);
  noTone(BUZZER_PIN);
}

void playWarning() {
  for(int i = 0; i < 3; i++) {
    tone(BUZZER_PIN, NOTE_C6, 100);
    delay(150);
  }
  noTone(BUZZER_PIN);
}

// ============================================================================
// IMU 簡化版(只讀角度)
// ============================================================================
void setupIMU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0x00);
  byte error = Wire.endTransmission();
  
  if (error == 0) {
    Serial.println(F("IMU 初始化成功"));
  } else {
    Serial.println(F("IMU 初始化失敗"));
  }
  delay(50);
}

void readIMU() {
  if (millis() - lastIMURead < IMU_READ_INTERVAL) {
    return;
  }
  lastIMURead = millis();
  
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_XOUT_H);
  byte error = Wire.endTransmission(false);
  
  if (error != 0) {
    tiltAngle = 0.0;
    return;
  }
  
  Wire.requestFrom((byte)MPU_ADDR, (byte)4);
  if (Wire.available() >= 4) {
    int16_t ax = (Wire.read() << 8) | Wire.read();
    int16_t ay = (Wire.read() << 8) | Wire.read();
    
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(ACCEL_XOUT_H + 4);
    Wire.endTransmission(false);
    Wire.requestFrom((byte)MPU_ADDR, (byte)2);
    int16_t az = (Wire.read() << 8) | Wire.read();
    
    float accelX = ax / 16384.0;
    float accelZ = az / 16384.0;
    
    tiltAngle = atan2(accelX, accelZ) * 57.2958;
    
    if (tiltAngle > 90) tiltAngle = 90;
    if (tiltAngle < -90) tiltAngle = -90;
  }
}

void checkTiltWarning() {
  float absTilt = abs(tiltAngle);
  bool currentWarning = (absTilt >= TILT_WARNING);
  
  if (currentWarning && !lastTiltWarning) {
    if (absTilt >= TILT_DANGER) {
      playWarning();
      Serial.print(F("[警告] 危險傾斜: "));
      Serial.println(tiltAngle, 1);
    } else {
      tone(BUZZER_PIN, NOTE_A5, 200);
      delay(220);
      noTone(BUZZER_PIN);
      Serial.print(F("[警告] 傾斜警告: "));
      Serial.println(tiltAngle, 1);
    }
  }
  
  lastTiltWarning = currentWarning;
}

// ============================================================================
// 速度計算
// ============================================================================
void updateSpeed() {
  if (newSpeedData) {
    if (speedPulseInterval > 0) {
      float timeInSec = speedPulseInterval / 1000000.0;
      currentSpeed = (WHEEL_CIRCUMFERENCE / timeInSec) * 0.036;
    }
    newSpeedData = false;
    lastSpeedUpdate = millis();
  }
  
  if (millis() - lastSpeedUpdate > SPEED_TIMEOUT) {
    currentSpeed = 0.0;
  }
}

// ============================================================================
// 【核心安全】限位開關檢查(最高優先權)
// ============================================================================
void checkLimitSwitches() {
  if (millis() - lastLimitCheck < LIMIT_DEBOUNCE) {
    return;
  }
  lastLimitCheck = millis();
  
  bool upperRaw = (digitalRead(LIMIT_UP_PIN) == LOW);
  bool lowerRaw = (digitalRead(LIMIT_DOWN_PIN) == LOW);
  
  if (upperRaw && !lowerRaw) {
    analogWrite(PWM_PIN, 0);
    currentState = AT_TOP;
    systemLocked = true;
    lockStartTime = millis();
    upperLimit = true;
    lowerLimit = false;
    
    autoLiftDone = false;
    speedTimerRunning = false;
    
    Serial.println(F("[限位] 到達上方"));
    
  } else if (lowerRaw && !upperRaw) {
    analogWrite(PWM_PIN, 0);
    currentState = AT_BOTTOM;
    systemLocked = true;
    lockStartTime = millis();
    upperLimit = false;
    lowerLimit = true;
    
    Serial.println(F("[限位] 到達下方"));
    
  } else if (upperRaw && lowerRaw) {
    analogWrite(PWM_PIN, 0);
    currentState = AT_TOP;
    systemLocked = true;
    lockStartTime = millis();
    
    Serial.println(F("[錯誤] 雙限位觸發!"));
    playWarning();
    
  } else {
    upperLimit = false;
    lowerLimit = false;
  }
}

// ============================================================================
// 馬達控制
// ============================================================================
void startMotorUp() {
  if (pwmManualOverride) {
    Serial.println(F("[控制] PWM 手動關閉中"));
    return;
  }
  
  if (systemLocked) {
    Serial.println(F("[控制] 系統鎖定中"));
    return;
  }
  
  if (currentState == MOVING_UP) {
    Serial.println(F("[控制] 已在上升中"));
    return;
  }
  
  if (currentState == AT_TOP) {
    Serial.println(F("[控制] 已在頂端"));
    return;
  }
  
  analogWrite(PWM_PIN, pwmSpeed);
  currentState = MOVING_UP;
  systemLocked = true;
  lockStartTime = millis();
  
  playBeep(NOTE_A5, 100);
  Serial.println(F("[馬達] 開始上升"));
}

void startMotorDown() {
  if (pwmManualOverride) {
    Serial.println(F("[控制] PWM 手動關閉中"));
    return;
  }
  
  if (systemLocked) {
    Serial.println(F("[控制] 系統鎖定中"));
    return;
  }
  
  if (currentState == MOVING_DOWN) {
    Serial.println(F("[控制] 已在下降中"));
    return;
  }
  
  if (currentState == AT_BOTTOM) {
    Serial.println(F("[控制] 已在底部"));
    return;
  }
  
  analogWrite(PWM_PIN, pwmSpeed);
  currentState = MOVING_DOWN;
  systemLocked = true;
  lockStartTime = millis();
  
  playBeep(NOTE_A5, 100);
  Serial.println(F("[馬達] 開始下降"));
}

void stopMotor() {
  analogWrite(PWM_PIN, 0);
  Serial.println(F("[馬達] 停止"));
}

// ============================================================================
// 【核心功能】自動上升邏輯
// ============================================================================
void checkAutoLift() {
  if (currentState != AT_BOTTOM) {
    speedTimerRunning = false;
    speedAboveThresholdTime = 0;
    return;
  }
  
  if (autoLiftDone) {
    return;
  }
  
  if (pwmManualOverride) {
    return;  // PWM 手動關閉時不執行自動上升
  }
  
  if (currentSpeed >= AUTO_LIFT_SPEED) {
    if (!speedTimerRunning) {
      speedTimerRunning = true;
      speedAboveThresholdTime = millis();
      Serial.println(F("[自動] 速度達標,開始計時"));
    } else {
      unsigned long elapsed = millis() - speedAboveThresholdTime;
      if (elapsed >= SPEED_HOLD_TIME) {
        Serial.println(F("[自動] 持續 1 秒,開始上升!"));
        startMotorUp();
        autoLiftDone = true;
        speedTimerRunning = false;
        playBeep(NOTE_C5, 200);
      }
    }
  } else {
    if (speedTimerRunning) {
      Serial.println(F("[自動] 速度下降,重置計時"));
    }
    speedTimerRunning = false;
    speedAboveThresholdTime = 0;
  }
}

// ============================================================================
// 【簡化】IR 控制 - 任意按鍵切換 PWM 開/關
// ============================================================================
void handleIR(uint32_t code) {
  if (millis() - lastIRTime < IR_DELAY) return;
  lastIRTime = millis();
  
  // 任意按鍵都執行相同功能
  if (code != 0 && code != 0xFFFFFFFF) {  // 排除重複碼和無效碼
    pwmManualOverride = !pwmManualOverride;
    
    if (pwmManualOverride) {
      // 關閉 PWM
      analogWrite(PWM_PIN, 0);
      playBeep(NOTE_C4, 200);
      Serial.println(F("[IR] PWM 手動關閉 (安全模式)"));
    } else {
      // 恢復自動模式
      playBeep(NOTE_C5, 200);
      Serial.println(F("[IR] PWM 恢復自動模式"));
    }
  }
}

// ============================================================================
// 【新增】實體按鈕 - 循環切換 OLED 頁面
// ============================================================================
void checkButton() {
  bool buttonState = digitalRead(BUTTON_PIN);
  
  // 偵測按下(HIGH → LOW)
  if (buttonState == LOW && lastButtonState == HIGH) {
    if (millis() - lastButtonPress > BUTTON_DEBOUNCE) {
      lastButtonPress = millis();
      
      // 循環切換頁面 0 → 1 → 2 → 0
      currentPage = (currentPage + 1) % 3;
      
      playBeep(NOTE_A5, 50);
      
      Serial.print(F("[按鈕] 切換到頁面 "));
      Serial.println(currentPage);
    }
  }
  
  lastButtonState = buttonState;
}

// ============================================================================
// 【新增】OLED 顯示 - 3 個頁面
// ============================================================================
void drawMainPage() {
  display.clearDisplay();
  
  // 標題
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(F("WHEEL v9.1"));
  
  if (pwmManualOverride) {
    display.setCursor(80, 0);
    display.print(F("[SAFE]"));
  }
  
  display.drawLine(0, 9, 127, 9, SSD1306_WHITE);
  
  // 狀態(大字)
  display.setTextSize(2);
  display.setCursor(0, 12);
  switch(currentState) {
    case AT_TOP:
      display.print(F("AT TOP"));
      break;
    case AT_BOTTOM:
      display.print(F("BOTTOM"));
      break;
    case MOVING_UP:
      display.print(F("UP"));
      break;
    case MOVING_DOWN:
      display.print(F("DOWN"));
      break;
  }
  
  display.drawLine(0, 28, 127, 28, SSD1306_WHITE);
  
  // 速度 + 角度(並排)
  display.setTextSize(1);
  display.setCursor(0, 31);
  display.print(F("Speed:"));
  
  display.setTextSize(2);
  display.setCursor(0, 40);
  int speedInt = (int)currentSpeed;
  if (speedInt < 10) display.print(F(" "));
  display.print(speedInt);
  
  display.setTextSize(1);
  display.setCursor(30, 44);
  display.print(F("km/h"));
  
  // 角度
  display.setCursor(70, 31);
  display.print(F("Angle:"));
  
  display.setTextSize(2);
  display.setCursor(70, 40);
  int angleInt = (int)tiltAngle;
  if (angleInt >= 0 && angleInt < 10) display.print(F(" "));
  display.print(angleInt);
  
  display.setTextSize(1);
  display.setCursor(100, 44);
  display.print((char)247);
  
  // 警告指示
  if (abs(tiltAngle) >= TILT_DANGER) {
    display.fillRect(110, 31, 18, 10, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
    display.setCursor(112, 32);
    display.print(F("!!"));
    display.setTextColor(SSD1306_WHITE);
  } else if (abs(tiltAngle) >= TILT_WARNING) {
    display.setCursor(112, 32);
    display.print(F("!"));
  }
  
  display.drawLine(0, 52, 127, 52, SSD1306_WHITE);
  
  // 自動上升指示 + PWM
  display.setCursor(0, 54);
  display.print(F("PWM:"));
  display.print(pwmSpeed);
  
  if (speedTimerRunning) {
    unsigned long elapsed = millis() - speedAboveThresholdTime;
    unsigned long remaining = (SPEED_HOLD_TIME - elapsed) / 100;
    
    display.setCursor(60, 54);
    display.print(F("AUTO:"));
    display.print(remaining / 10);
    display.print(F("."));
    display.print(remaining % 10);
  }
  
  display.display();
}

void drawDebugPage() {
  display.clearDisplay();
  
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(F("DEBUG PAGE"));
  
  display.drawLine(0, 9, 127, 9, SSD1306_WHITE);
  
  // 限位開關狀態
  display.setCursor(0, 12);
  display.print(F("Limit: ["));
  display.print(upperLimit ? F("U") : F("-"));
  display.print(F("]["));
  display.print(lowerLimit ? F("D") : F("-"));
  display.print(F("]"));
  
  // 系統鎖
  display.setCursor(0, 22);
  display.print(F("Lock: "));
  display.print(systemLocked ? F("YES") : F("NO"));
  
  // 自動上升狀態
  display.setCursor(0, 32);
  display.print(F("Auto: "));
  if (autoLiftDone) {
    display.print(F("DONE"));
  } else if (speedTimerRunning) {
    display.print(F("TIMING"));
  } else {
    display.print(F("READY"));
  }
  
  // 速度詳細
  display.setCursor(0, 42);
  display.print(F("Speed: "));
  display.print(currentSpeed, 1);
  display.print(F(" km/h"));
  
  // 角度詳細
  display.setCursor(0, 52);
  display.print(F("Tilt:  "));
  display.print(tiltAngle, 1);
  display.print((char)247);
  
  if (abs(tiltAngle) >= TILT_DANGER) {
    display.print(F(" DANGER"));
  } else if (abs(tiltAngle) >= TILT_WARNING) {
    display.print(F(" WARN"));
  }
  
  display.display();
}

void drawSystemInfoPage() {
  display.clearDisplay();
  
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(F("SYSTEM INFO"));
  
  display.drawLine(0, 9, 127, 9, SSD1306_WHITE);
  
  // PWM 手動模式狀態
  display.setCursor(0, 12);
  display.print(F("PWM Mode: "));
  if (pwmManualOverride) {
    display.print(F("MANUAL"));
    display.setCursor(0, 22);
    display.print(F("(Safety OFF)"));
  } else {
    display.print(F("AUTO"));
  }
  
  // 運行時間
  display.setCursor(0, 32);
  display.print(F("Uptime: "));
  unsigned long seconds = millis() / 1000;
  unsigned long minutes = seconds / 60;
  unsigned long hours = minutes / 60;
  display.print(hours);
  display.print(F("h "));
  display.print(minutes % 60);
  display.print(F("m"));
  
  // PWM 設定值
  display.setCursor(0, 42);
  display.print(F("PWM Value: "));
  display.print(pwmSpeed);
  
  // 版本資訊
  display.setCursor(0, 52);
  display.print(F("Version: 9.1"));
  
  display.display();
}

void updateDisplay() {
  if (millis() - lastDisplayUpdate < 250) return;
  lastDisplayUpdate = millis();
  
  switch(currentPage) {
    case 0:
      drawMainPage();
      break;
    case 1:
      drawDebugPage();
      break;
    case 2:
      drawSystemInfoPage();
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
  pinMode(BUTTON_PIN, INPUT_PULLUP);  // 按鈕
  
  // 確保馬達停止
  analogWrite(PWM_PIN, 0);
  setRGB(0, 0, 0);
  
  // 中斷與外設
  attachInterrupt(digitalPinToInterrupt(HALL_SPEED_PIN), speedInterrupt, FALLING);
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
  
  // OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("OLED 初始化失敗!"));
    while(1);
  }
  
  // 開機畫面
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(10, 10);
  display.print(F("WHEEL"));
  display.setTextSize(1);
  display.setCursor(10, 35);
  display.print(F("v9.1 Simple"));
  display.display();
  delay(1500);
  
  playStartupMusic();
  
  // IMU 初始化
  setupIMU();
  
  Serial.println(F("===================================="));
  Serial.println(F("  智能輔助輪 v9.1 簡化版"));
  Serial.println(F("===================================="));
  Serial.println(F("\n【核心功能】"));
  Serial.println(F("✓ 速度持續 1 秒 ≥ 15 km/h → 自動上升"));
  Serial.println(F("✓ 限位開關絕對優先"));
  Serial.println(F("✓ 傾斜 > 10° 警告 / > 15° 危險"));
  Serial.println(F("✓ 單一鎖機制"));
  Serial.println(F("✓ 3 頁 OLED(主頁/除錯/系統)"));
  Serial.println(F("===================================="));
  Serial.println(F("\n【控制方式】"));
  Serial.println(F("IR 遙控器 → 任意鍵切換 PWM 開/關"));
  Serial.println(F("實體按鈕(Pin12) → 切換 OLED 頁面"));
  Serial.println(F("====================================\n"));
  
  wdt_enable(WDTO_2S);
}

// ============================================================================
// Loop 主迴圈(清晰的執行順序)
// ============================================================================
void loop() {
  wdt_reset();
  
  // ========== 第一優先:安全檢查 ==========
  checkLimitSwitches();
  
  // 如果限位觸發,跳過所有邏輯
  if (upperLimit || lowerLimit) {
    updateDisplay();
    updateRGBByState();
    delay(10);
    return;
  }
  
  // ========== 第二優先:更新資料 ==========
  updateSpeed();
  readIMU();
  
  // 讀取 PWM 設定
  if (millis() - lastSpeedRead > 100) {
    lastSpeedRead = millis();
    int raw = analogRead(SPEED_CONTROL_PIN);
    pwmSpeed = map(raw, 0, 1023, 100, 255);
    pwmSpeed = constrain(pwmSpeed, 100, 255);
  }
  
  // ========== 第三優先:解鎖檢查 ==========
  if (systemLocked && millis() - lockStartTime > LOCK_DURATION) {
    systemLocked = false;
    Serial.println(F("[系統] 解鎖"));
  }
  
  // ========== 第四優先:自動上升 ==========
  checkAutoLift();
  
  // ========== 第五優先:IR 控制 ==========
  if (IrReceiver.decode()) {
    handleIR(IrReceiver.decodedIRData.decodedRawData);
    IrReceiver.resume();
  }
  
  // ========== 第六優先:實體按鈕 ==========
  checkButton();
  
  // ========== 第七優先:傾斜警告 ==========
  checkTiltWarning();
  
  // ========== 介面更新 ==========
  updateDisplay();
  updateRGBByState();
  
  // ========== 序列埠輸出(每秒一次) ==========
  if (millis() - lastSerialPrint > 1000) {
    lastSerialPrint = millis();
    
    Serial.print(F("狀態:"));
    switch(currentState) {
      case AT_TOP: Serial.print(F("上")); break;
      case AT_BOTTOM: Serial.print(F("下")); break;
      case MOVING_UP: Serial.print(F("↑")); break;
      case MOVING_DOWN: Serial.print(F("↓")); break;
    }
    
    Serial.print(F(" |限位:["));
    Serial.print(upperLimit ? F("上") : F("-"));
    Serial.print(F("]["));
    Serial.print(lowerLimit ? F("下") : F("-"));
    Serial.print(F("]"));
    
    Serial.print(F(" |速度:"));
    Serial.print(currentSpeed, 1);
    Serial.print(F("km/h"));
    
    Serial.print(F(" |角度:"));
    Serial.print(tiltAngle, 1);
    Serial.print((char)176);
    
    if (speedTimerRunning) {
      unsigned long elapsed = millis() - speedAboveThresholdTime;
      Serial.print(F(" |計時:"));
      Serial.print(elapsed / 100);
      Serial.print(F("/10"));
    }
    
    Serial.print(F(" |鎖:"));
    Serial.print(systemLocked ? F("是") : F("否"));
    
    if (pwmManualOverride) {
      Serial.print(F(" |PWM:手動關閉"));
    }
    
    Serial.print(F(" |頁:"));
    Serial.print(currentPage);
    
    Serial.println();
  }
  
  delay(5);
}
