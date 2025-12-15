// ============================================================================
// 自行車輔助輪智能控制系統 v3 - 完整修正版
// 修正：雙霍爾感測器、緊急停止按鈕、記憶體優化
// ============================================================================

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <IRremote.h>
#include <INA226_WE.h>

// ============================================================================
// 音符定義
// ============================================================================
#define NOTE_C5  523
#define NOTE_E5  659
#define NOTE_G5  784
#define NOTE_C6  1047
#define NOTE_A5  880

// ============================================================================
// OLED 設定（128x32 節省記憶體）
// ============================================================================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ============================================================================
// MPU6050 設定
// ============================================================================
#define MPU_6050 0x68
#define PWR_MGMT_1 0x6B
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H  0x43

// ============================================================================
// 數位腳位定義
// ============================================================================
const byte BUZZER_PIN = 8;
const byte RELAY_PIN = 11;
const byte IR_RECEIVE_PIN = 3;
const byte HALL_STOP_UP_PIN = 4;     // 上升端點霍爾（UP 位置磁鐵）
const byte HALL_STOP_DOWN_PIN = 10;  // 下降端點霍爾（DOWN 位置磁鐵）
const byte HALL_SPEED_PIN = 2;       // 速度霍爾
const byte EMERGENCY_STOP_PIN = 12;  // 緊急停止按鈕
const byte PWM_CONTROL_PIN = 9;
const byte SPEED_POT_PIN = A0;
// ============================================================================
// RGB LED 腳位定義
// ============================================================================
const byte RGB_RED_PIN = 5;
const byte RGB_GREEN_PIN = 7;
const byte RGB_BLUE_PIN = 6;
const bool RGB_COMMON_ANODE = false;  // 如果 RGB 顏色相反，改為 true

// ============================================================================
// RGB 顏色（使用 enum 節省記憶體）
// ============================================================================
enum RGBColor : byte {
  RGB_OFF = 0, RGB_GREEN, RGB_YELLOW, RGB_BLUE, RGB_RED, RGB_PURPLE
};

// ============================================================================
// INA226 電流監測
// ============================================================================
INA226_WE ina226;
const float MAX_CURRENT_A = 5.0;
const float BUZZER_CURRENT_THRESHOLD = 1.5;
float current_A = 0.0;

// ============================================================================
// PWM 週期控制
// ============================================================================
const uint16_t PWM_ON_DURATION = 1800;
const uint16_t PWM_OFF_DURATION = 1800;
uint32_t pwmPreviousMillis = 0;
bool pwmState = false;

// ============================================================================
// 速度測量（使用常數減少 SRAM）
// ============================================================================
#define WHEEL_CIRCUMFERENCE 204.2  // 65 * 3.14159
const float SPEED_THRESHOLD = 10.0;
const uint16_t SPEED_TIMEOUT = 2000;

volatile uint32_t lastSpeedTrigger = 0;
volatile uint32_t timeBetweenTriggers = 0;
volatile bool newSpeedData = false;
float currentSpeed = 0.0;
uint32_t lastSpeedUpdate = 0;

// ============================================================================
// IMU 濾波器
// ============================================================================
float compAngleX = 0.0f;
float gyroBiasX = 0.0f;
float accelBiasX = 0.0f;

// ============================================================================
// 傾斜安全設定
// ============================================================================
const float SAFE_ANGLE = 20.0f;
const float TILT_SAFE = 12.0f;
const float EMERGENCY_ANGLE = 35.0f;
const uint16_t TILT_TRIGGER_TIME = 500;
const uint16_t AUTO_UP_DELAY = 3000;

uint32_t tiltStartTime = 0;
bool tiltWarning = false;
uint32_t safeStartTime = 0;
bool emergencyStop = false;

// ============================================================================
// 系統狀態
// ============================================================================
enum SystemState : byte { RUNNING, HALTED, COOLING_DOWN };
SystemState currentSystemState = RUNNING;
uint32_t haltTime = 0;
const uint16_t RESTART_DELAY_MS = 5000;

// ============================================================================
// 霍爾停止邏輯（雙霍爾感測器）
// ============================================================================
bool hallStopLocked = false;
byte lastHallStopUpState = HIGH;     // 上升端點狀態
byte lastHallStopDownState = HIGH;   // 下降端點狀態

// ============================================================================
// 控制變數
// ============================================================================
bool wheelDown = false;
bool relayCmd = LOW;
byte ctrlSrc = 0;
bool manualMode = false;

// ============================================================================
// 顯示與動畫
// ============================================================================
uint32_t wheelAnimStartTime = 0;
bool showWheelAnim = false;
uint32_t lastPrintTime = 0;
uint32_t lastDisplayTime = 0;
uint32_t lastIRCode = 0;
const uint16_t IR_DELAY = 500;

// ============================================================================
// RGB LED 控制
// ============================================================================
void setRGB(byte r, byte g, byte b) {
  if (RGB_COMMON_ANODE) {
    // 共陽極：反轉邏輯
    analogWrite(RGB_RED_PIN, 255 - r);
    analogWrite(RGB_GREEN_PIN, 255 - g);
    analogWrite(RGB_BLUE_PIN, 255 - b);
  } else {
    // 共陰極：正常邏輯
    analogWrite(RGB_RED_PIN, r);
    analogWrite(RGB_GREEN_PIN, g);
    analogWrite(RGB_BLUE_PIN, b);
  }
}

void setRGBColor(RGBColor c) {
  switch (c) {
    case RGB_OFF: setRGB(0, 0, 0); break;
    case RGB_GREEN: setRGB(0, 255, 0); break;
    case RGB_YELLOW: setRGB(255, 255, 0); break;
    case RGB_BLUE: setRGB(0, 0, 255); break;
    case RGB_RED: setRGB(255, 0, 0); break;
    case RGB_PURPLE: setRGB(128, 0, 255); break;
  }
}

// ============================================================================
// 音效函式（簡化版）
// ============================================================================
void beep(uint16_t freq, byte duration) {
  tone(BUZZER_PIN, freq, duration);
  delay(duration);
  noTone(BUZZER_PIN);
}

void soundStartup() {
  beep(NOTE_C5, 100);
  beep(NOTE_G5, 150);
  beep(NOTE_C6, 200);
}

void soundDown() {
  beep(NOTE_G5, 60);
  beep(NOTE_C5, 100);
}

void soundUp() {
  beep(NOTE_C5, 60);
  beep(NOTE_G5, 100);
}

void soundLock() {
  beep(NOTE_C6, 80);
  beep(NOTE_E5, 80);
}

void soundEmergency() {
  for (byte i = 0; i < 3; i++) {
    beep(NOTE_C6, 100);
    delay(50);
    beep(NOTE_A5, 100);
    delay(50);
  }
}

// ============================================================================
// 速度中斷
// ============================================================================
void speedInterrupt() {
  uint32_t t = micros();
  if (t - lastSpeedTrigger > 50000) {
    timeBetweenTriggers = t - lastSpeedTrigger;
    lastSpeedTrigger = t;
    newSpeedData = true;
  }
}

// ============================================================================
// 速度計算（優化浮點運算）
// ============================================================================
float calculateSpeed() {
  if (timeBetweenTriggers == 0) return 0.0;
  float timeInSec = timeBetweenTriggers / 1000000.0;
  return (WHEEL_CIRCUMFERENCE / timeInSec) * 0.036;
}

// ============================================================================
// MPU6050 函式
// ============================================================================
int16_t read16(byte reg) {
  Wire.beginTransmission(MPU_6050);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((byte)MPU_6050, (byte)2);
  return (Wire.read() << 8) | Wire.read();
}

void setupMPU() {
  Wire.beginTransmission(MPU_6050);
  Wire.write(PWR_MGMT_1);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(50);
}

void calibrateIMU() {
  float sumAx = 0, sumGx = 0;
  for (byte i = 0; i < 50; i++) {
    sumAx += read16(ACCEL_XOUT_H) / 16384.0f;
    sumGx += read16(GYRO_XOUT_H) / 131.0f;
    delay(5);
  }
  accelBiasX = sumAx / 50;
  gyroBiasX = sumGx / 50;
}

void readIMU(float &ax, float &gx) {
  ax = read16(ACCEL_XOUT_H) / 16384.0f;
  float az = read16(ACCEL_XOUT_H + 4) / 16384.0f;
  gx = read16(GYRO_XOUT_H) / 131.0f;
  
  gx -= gyroBiasX;
  ax -= accelBiasX;
  
  float accelAngle = atan2(ax, az) * 57.2958f;
  compAngleX = 0.98 * (compAngleX + gx * 0.01) + 0.02 * accelAngle;
}

// ============================================================================
// 啟動動畫（簡化版）
// ============================================================================
void showStartup() {
  display.clearDisplay();
  
  // 閃爍標題
  for (byte i = 0; i < 2; i++) {
    display.fillRect(0, 0, 128, 32, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
    display.setTextSize(3);
    display.setCursor(14, 6);
    display.print(F("BIKE"));
    display.display();
    display.setTextColor(SSD1306_WHITE);
    delay(300);
    display.clearDisplay();
    display.display();
    delay(150);
  }
  
  // 載入條
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(28, 8);
  display.print(F("LOADING"));
  
  for (byte i = 0; i <= 100; i += 10) {
    display.fillRect(14, 20, i, 4, SSD1306_WHITE);
    display.display();
    if (i % 30 == 0) beep(500 + i * 3, 40);
    delay(50);
  }
  delay(200);
}

// ============================================================================
// OLED 主畫面（優化版）
// ============================================================================
void showStatus() {
  display.clearDisplay();
  display.drawRect(0, 0, 128, 32, SSD1306_WHITE);
  
  display.setTextSize(1);
  display.setCursor(4, 3);
  
  // 狀態顯示（使用 F() 節省 SRAM）
  if (emergencyStop) {
    display.print(F("EMERG"));
  } else if (currentSystemState == HALTED) {
    display.print(F("HALT"));
  } else if (currentSystemState == COOLING_DOWN) {
    display.print(F("COOL"));
  } else if (hallStopLocked) {
    display.print(F("LOCK"));
  } else if (ctrlSrc == 1) {
    display.print(F("HALL"));
  } else if (ctrlSrc == 2) {
    display.print(F("IR"));
  } else if (ctrlSrc == 3) {
    display.print(F("AUTO"));
  } else if (ctrlSrc == 4) {
    display.print(F("TILT"));
  } else {
    display.print(F("RUN"));
  }
  
  // 速度和角度
  display.print(F(" "));
  display.print((int)currentSpeed);
  display.print(F("k "));
  int angle = (int)compAngleX;
  if (angle >= 0 && angle < 10) display.print(F(" "));
  display.print(angle);
  display.print(F("d"));
  
  // 警告標示
  if (emergencyStop || current_A > BUZZER_CURRENT_THRESHOLD) {
    display.setCursor(114, 3);
    display.print(F("!!"));
  } else if (tiltWarning) {
    display.setCursor(120, 3);
    display.print(F("!"));
  }
  
  display.setTextSize(2);
  
  // 輔助輪狀態
  if (showWheelAnim && (millis() - wheelAnimStartTime < 600)) {
    byte frame = ((millis() - wheelAnimStartTime) / 150) % 2;
    
    if (wheelDown) {
      display.setCursor(12, 15);
      display.print(frame == 0 ? F("v DOWN") : F("V DOWN"));
      display.fillTriangle(108, 19, 112, 25, 116, 19, SSD1306_WHITE);
    } else {
      display.setCursor(12, 15);
      display.print(frame == 0 ? F("^ UP") : F("^ UP"));
      display.fillTriangle(108, 25, 112, 19, 116, 25, SSD1306_WHITE);
    }
  } else {
    showWheelAnim = false;
    
    if (wheelDown) {
      display.setCursor(12, 15);
      display.print(F("V DOWN"));
      display.fillTriangle(108, 19, 112, 25, 116, 19, SSD1306_WHITE);
      display.fillTriangle(108, 25, 112, 29, 116, 25, SSD1306_WHITE);
    } else {
      display.setCursor(24, 15);
      display.print(F("^ UP"));
      display.fillTriangle(108, 29, 112, 23, 116, 29, SSD1306_WHITE);
      display.fillTriangle(108, 23, 112, 19, 116, 23, SSD1306_WHITE);
    }
  }
  
  display.display();
}

// ============================================================================
// 紅外線遙控處理
// ============================================================================
void handleIRCommand(uint32_t code) {
  if (millis() - lastIRCode > IR_DELAY) {
    lastIRCode = millis();
    
    if (hallStopLocked) {
      hallStopLocked = false;
      Serial.println(F("IR unlock"));
    }
    
    wheelDown = !wheelDown;
    manualMode = true;
    ctrlSrc = 2;
    
    beep(NOTE_A5, 40);
    delay(30);
    if (wheelDown) soundDown();
    else soundUp();
    
    showWheelAnim = true;
    wheelAnimStartTime = millis();
  }
}

// ============================================================================
// Setup
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(100);
  
  Wire.begin();
  
  // 腳位初始化
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(HALL_STOP_UP_PIN, INPUT_PULLUP);     // 上升端點霍爾
  pinMode(HALL_STOP_DOWN_PIN, INPUT_PULLUP);   // 下降端點霍爾
  pinMode(HALL_SPEED_PIN, INPUT_PULLUP);
  pinMode(EMERGENCY_STOP_PIN, INPUT);          // 緊急停止按鈕（根據硬體選擇 INPUT 或 INPUT_PULLUP）
  // 如果按鈕按下時讀到 LOW，請改為：pinMode(EMERGENCY_STOP_PIN, INPUT_PULLUP);
  pinMode(PWM_CONTROL_PIN, OUTPUT);
  pinMode(RGB_RED_PIN, OUTPUT);
  pinMode(RGB_GREEN_PIN, OUTPUT);
  pinMode(RGB_BLUE_PIN, OUTPUT);
  
  digitalWrite(RELAY_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  analogWrite(PWM_CONTROL_PIN, 0);
  setRGBColor(RGB_OFF);
  
  // 初始化模組
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
  attachInterrupt(digitalPinToInterrupt(HALL_SPEED_PIN), speedInterrupt, FALLING);
  
  // OLED 初始化（關鍵修正）
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("OLED Fail"));
    setRGBColor(RGB_RED);
    beep(NOTE_C5, 500);
    while (1);
  }
  
  // INA226 初始化
  ina226.init();
  Serial.println(F("INA226 OK"));
  
  // MPU6050 初始化
  setupMPU();
  calibrateIMU();
  
  // 啟動動畫
  Serial.println(F("Testing RGB..."));
  // RGB 測試序列（確認接線正確）
  setRGBColor(RGB_RED);
  Serial.println(F("RED"));
  delay(500);
  setRGBColor(RGB_GREEN);
  Serial.println(F("GREEN"));
  delay(500);
  setRGBColor(RGB_BLUE);
  Serial.println(F("BLUE"));
  delay(500);
  setRGBColor(RGB_YELLOW);
  Serial.println(F("YELLOW"));
  delay(500);
  setRGBColor(RGB_PURPLE);
  Serial.println(F("PURPLE"));
  delay(500);
  setRGBColor(RGB_OFF);
  
  showStartup();
  soundStartup();
  
  Serial.println(F("=== READY ==="));
  setRGBColor(RGB_GREEN);
}

// ============================================================================
// Loop - 核心邏輯
// ============================================================================
void loop() {
  uint32_t currentMillis = millis();
  
  // ========================================================================
  // 【階段 1】讀取感測器
  // ========================================================================
  float ax, gx;
  readIMU(ax, gx);
  
  current_A = ina226.getCurrent_A();
  
  // 過電流蜂鳴器
  if (current_A > BUZZER_CURRENT_THRESHOLD) {
    tone(BUZZER_PIN, 1000);
  } else {
    noTone(BUZZER_PIN);
  }
  
  // 速度更新
  if (newSpeedData) {
    currentSpeed = calculateSpeed();
    newSpeedData = false;
    lastSpeedUpdate = millis();
    
    // 【移除速度鎖定機制】
    // 速度達標不再鎖定，而是由自動模式動態控制
  }
  if (millis() - lastSpeedUpdate > SPEED_TIMEOUT) {
    currentSpeed = 0.0;
  }
  
  // 紅外線接收
  if (IrReceiver.decode()) {
    handleIRCommand(IrReceiver.decodedIRData.decodedRawData);
    IrReceiver.resume();
  }
  
  // 霍爾停止檢測（雙端點版本）
  byte hallStopUp = digitalRead(HALL_STOP_UP_PIN);
  byte hallStopDown = digitalRead(HALL_STOP_DOWN_PIN);
  
  // 檢測上升端點霍爾
  if (hallStopUp != lastHallStopUpState) {
    lastHallStopUpState = hallStopUp;
    
    if (hallStopUp == LOW) {  // 碰到上升端點磁鐵
      hallStopLocked = true;
      manualMode = false;
      wheelDown = false;  // 鎖定在 UP 位置
      ctrlSrc = 1;
      
      soundLock();
      showWheelAnim = true;
      wheelAnimStartTime = millis();
      
      Serial.println(F("Hall UP lock"));
    }
  }
  
  // 檢測下降端點霍爾
  if (hallStopDown != lastHallStopDownState) {
    lastHallStopDownState = hallStopDown;
    
    if (hallStopDown == LOW) {  // 碰到下降端點磁鐵
      hallStopLocked = true;
      manualMode = false;
      wheelDown = true;  // 鎖定在 DOWN 位置
      ctrlSrc = 1;
      
      soundLock();
      showWheelAnim = true;
      wheelAnimStartTime = millis();
      
      Serial.println(F("Hall DOWN lock"));
    }
  }
  
  // ========================================================================
  // 【階段 2】PWM 週期控制
  // ========================================================================
  // 【修正】HALTED/COOLING 狀態也要輸出 PWM（讓線性致動器推下去）
  if (relayCmd == HIGH) {
    // 如果是緊急狀態（HALTED/COOLING），直接全速 PWM，不做週期
    if (currentSystemState == HALTED || currentSystemState == COOLING_DOWN) {
      int potValue = analogRead(SPEED_POT_PIN);
      int pwmSpeed = map(potValue, 0, 1023, 0, 255);
      analogWrite(PWM_CONTROL_PIN, pwmSpeed);
      pwmState = true;  // 標記為 ON
    }
    // 正常狀態且未被霍爾鎖定，才做週期控制
    else if (!hallStopLocked && currentSystemState == RUNNING) {
      uint16_t interval = pwmState ? PWM_ON_DURATION : PWM_OFF_DURATION;
      
      if (currentMillis - pwmPreviousMillis >= interval) {
        pwmPreviousMillis = currentMillis;
        pwmState = !pwmState;
        
        if (pwmState) {
          int potValue = analogRead(SPEED_POT_PIN);
          int pwmSpeed = map(potValue, 0, 1023, 0, 255);
          analogWrite(PWM_CONTROL_PIN, pwmSpeed);
        } else {
          analogWrite(PWM_CONTROL_PIN, 0);
        }
      }
    } else {
      analogWrite(PWM_CONTROL_PIN, 0);
      pwmState = false;
    }
  } else {
    // 繼電器 OFF 時，PWM 也關閉
    analogWrite(PWM_CONTROL_PIN, 0);
    pwmState = false;
  }
  
  // ========================================================================
  // 【階段 3】三層決策邏輯
  // ========================================================================
  
  relayCmd = LOW;
  bool allowLowerPriority = true;
  RGBColor targetRGB = RGB_GREEN;
  
  // 🔴 第一層：安全層
  bool isOverloaded = (current_A > MAX_CURRENT_A);
  bool emergencyButtonPressed = (digitalRead(EMERGENCY_STOP_PIN) == HIGH);  // 如果按鈕邏輯相反，改為 == LOW
  // 【修正】急停條件：過載 或 緊急按鈕
  bool haltConditionActive = isOverloaded || emergencyButtonPressed;
  
  switch (currentSystemState) {
    case RUNNING:
      if (haltConditionActive) {
        currentSystemState = HALTED;
        haltTime = currentMillis;
        
        // 【新增】緊急停止音效
        soundEmergency();
        
        Serial.println(F("HALTED"));
      }
      break;
      
    case HALTED:
      relayCmd = HIGH;
      wheelDown = true;
      targetRGB = RGB_RED;
      ctrlSrc = 6;
      manualMode = false;
      allowLowerPriority = false;
      
      if (!haltConditionActive) {
        currentSystemState = COOLING_DOWN;
        haltTime = currentMillis;
        Serial.println(F("COOLING"));
      }
      break;
      
    case COOLING_DOWN:
      relayCmd = HIGH;
      wheelDown = true;
      targetRGB = RGB_RED;
      ctrlSrc = 6;
      allowLowerPriority = false;
      
      if (haltConditionActive) {
        currentSystemState = HALTED;
      } else if (currentMillis - haltTime >= RESTART_DELAY_MS) {
        currentSystemState = RUNNING;
        manualMode = false;
        hallStopLocked = false;
        Serial.println(F("RUNNING"));
      }
      break;
  }
  
  // 緊急傾斜
  if (allowLowerPriority && abs(compAngleX) > EMERGENCY_ANGLE) {
    if (!emergencyStop) {
      emergencyStop = true;
      soundEmergency();
      showWheelAnim = true;
      wheelAnimStartTime = millis();
      Serial.println(F("EMERGENCY!"));
    }
    relayCmd = HIGH;
    wheelDown = true;
    targetRGB = RGB_RED;
    ctrlSrc = 5;
    manualMode = false;
    allowLowerPriority = false;
  } else if (emergencyStop && abs(compAngleX) < TILT_SAFE) {
    emergencyStop = false;
    manualMode = false;
    Serial.println(F("Emerg clear"));
  }
  
  // 🔵 第二層：傾斜警告
  if (allowLowerPriority) {
    if (abs(compAngleX) > SAFE_ANGLE) {
      if (tiltStartTime == 0) tiltStartTime = millis();
      
      if (!tiltWarning && (millis() - tiltStartTime >= TILT_TRIGGER_TIME)) {
        tiltWarning = true;
        wheelDown = true;
        ctrlSrc = 4;
        manualMode = false;
        beep(NOTE_G5, 150);
        delay(40);
        soundDown();
        showWheelAnim = true;
        wheelAnimStartTime = millis();
        safeStartTime = 0;
        Serial.println(F("TILT WARN"));
      }
    } else {
      tiltStartTime = 0;
      
      if (tiltWarning) {
        if (abs(compAngleX) < TILT_SAFE) {
          if (safeStartTime == 0) safeStartTime = millis();
          
          if (millis() - safeStartTime >= AUTO_UP_DELAY) {
            tiltWarning = false;
            wheelDown = false;
            ctrlSrc = 3;
            manualMode = false;
            showWheelAnim = true;
            wheelAnimStartTime = millis();
            soundUp();
            Serial.println(F("Tilt OK"));
            safeStartTime = 0;
          }
        } else {
          safeStartTime = 0;
        }
      }
    }
    
    if (tiltWarning) {
      relayCmd = HIGH;
      wheelDown = true;
      targetRGB = RGB_BLUE;
      allowLowerPriority = false;
    }
  }
  
  // 🟢 第三層：操作層
  if (allowLowerPriority) {
    // 霍爾停止鎖定（最高優先）
    if (hallStopLocked) {
      relayCmd = wheelDown ? HIGH : LOW;
      targetRGB = wheelDown ? RGB_YELLOW : RGB_GREEN;
    } 
    // 手動模式（IR 控制）
    else if (manualMode) {
      relayCmd = wheelDown ? HIGH : LOW;
      targetRGB = wheelDown ? RGB_YELLOW : RGB_GREEN;
    } 
    // 自動模式（速度控制）- 移除速度鎖定
    else {
      bool shouldWheelDown = (currentSpeed < SPEED_THRESHOLD);
      
      if (shouldWheelDown != wheelDown) {
        wheelDown = shouldWheelDown;
        ctrlSrc = 3;
        
        if (wheelDown) soundDown();
        else soundUp();
        
        showWheelAnim = true;
        wheelAnimStartTime = millis();
        
        Serial.print(F("AUTO: "));
        Serial.print(currentSpeed, 1);
        Serial.print(F("km/h -> "));
        Serial.println(wheelDown ? F("DOWN") : F("UP"));
      }
      
      relayCmd = wheelDown ? HIGH : LOW;
      targetRGB = (currentSpeed < SPEED_THRESHOLD) ? RGB_YELLOW : RGB_GREEN;
    }
  }
  
  // ========================================================================
  // 【階段 4】最終輸出
  // ========================================================================
  
  // 如果您的繼電器模組是「低電平觸發」，請將下一行改為：
  // digitalWrite(RELAY_PIN, !relayCmd);  // 反轉邏輯
  digitalWrite(RELAY_PIN, relayCmd);
  
  // 【修正】RGB LED 輸出（確保正確設定）
  setRGBColor(targetRGB);
  
  // ========================================================================
  // 【階段 5】顯示更新
  // ========================================================================
  
  if (millis() - lastDisplayTime > 50) {
    showStatus();
    lastDisplayTime = millis();
  }
  
  // 序列埠輸出（除錯版本）
  if (millis() - lastPrintTime > 500) {
    // 狀態顯示
    Serial.print(F("State:"));
    switch (currentSystemState) {
      case RUNNING: Serial.print(F("RUN")); break;
      case HALTED: Serial.print(F("HALT")); break;
      case COOLING_DOWN: Serial.print(F("COOL")); break;
    }
    
    Serial.print(F(" | S:"));
    Serial.print(currentSpeed, 1);
    Serial.print(F(" A:"));
    Serial.print((int)compAngleX);
    Serial.print(F(" C:"));
    Serial.print(current_A, 2);
    
    Serial.print(F(" | R:"));
    Serial.print(relayCmd);
    Serial.print(F(" W:"));
    Serial.print(wheelDown ? F("DN") : F("UP"));
    Serial.print(F(" PWM:"));
    Serial.print(pwmState ? F("ON") : F("OFF"));
    
    Serial.print(F(" | HallUp:"));
    Serial.print(digitalRead(HALL_STOP_UP_PIN));
    Serial.print(F(" HallDn:"));
    Serial.print(digitalRead(HALL_STOP_DOWN_PIN));
    Serial.print(F(" EmgBtn:"));
    Serial.print(digitalRead(EMERGENCY_STOP_PIN));
    
    Serial.print(F(" | Lock:"));
    Serial.print(hallStopLocked ? F("Y") : F("N"));
    Serial.print(F(" Src:"));
    Serial.println(ctrlSrc);
    
    lastPrintTime = millis();
  }
  
  delay(10);
}
