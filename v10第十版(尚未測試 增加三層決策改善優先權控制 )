// ============================================================================
// 自行車輔助輪智能控制系統 v3.1 - 速度控制修正版
// 修正：速度觸發邏輯、緊急按鈕、優先權調整、除錯輸出增強
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
// 速度測量（修正版）
// ============================================================================
#define WHEEL_CIRCUMFERENCE 204.2  // 65 * 3.14159 (單位：公分)
const float SPEED_THRESHOLD = 10.0;        // 速度閾值（km/h）
const uint16_t SPEED_TIMEOUT = 2000;       // 速度超時（毫秒）

volatile uint32_t lastSpeedTrigger = 0;
volatile uint32_t timeBetweenTriggers = 0;
volatile bool newSpeedData = false;
volatile uint32_t speedPulseCount = 0;     // 【新增】脈衝計數

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
byte lastHallStopUpState = HIGH;
byte lastHallStopDownState = HIGH;

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
    analogWrite(RGB_RED_PIN, 255 - r);
    analogWrite(RGB_GREEN_PIN, 255 - g);
    analogWrite(RGB_BLUE_PIN, 255 - b);
  } else {
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
// 音效函式
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
// 速度中斷（修正版 - 降低防彈跳時間）
// ============================================================================
void speedInterrupt() {
  uint32_t t = micros();
  // 【修正】降低防彈跳時間到 10ms，避免漏掉快速脈衝
  if (t - lastSpeedTrigger > 10000) {
    timeBetweenTriggers = t - lastSpeedTrigger;
    lastSpeedTrigger = t;
    newSpeedData = true;
    speedPulseCount++;  // 脈衝計數
  }
}

// ============================================================================
// 速度計算
// ============================================================================
float calculateSpeed() {
  if (timeBetweenTriggers == 0) return 0.0;
  float timeInSec = timeBetweenTriggers / 1000000.0;
  // 公式：(輪周長 cm / 100) / 時間(秒) * 3.6 = km/h
  return (WHEEL_CIRCUMFERENCE / 100.0 / timeInSec) * 3.6;
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
// 啟動動畫
// ============================================================================
void showStartup() {
  display.clearDisplay();
  
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
// OLED 主畫面
// ============================================================================
void showStatus() {
  display.clearDisplay();
  display.drawRect(0, 0, 128, 32, SSD1306_WHITE);
  
  display.setTextSize(1);
  display.setCursor(4, 3);
  
  if (emergencyStop) {
    display.print(F("EMERG"));
  } else if (currentSystemState == HALTED) {
    display.print(F("HALT"));
  } else if (currentSystemState == COOLING_DOWN) {
    display.print(F("COOL"));
  } else if (hallStopLocked) {
    display.print(F("LOCK"));
  } else if (manualMode) {
    display.print(F("MAN"));
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
  
  display.print(F(" "));
  display.print((int)currentSpeed);
  display.print(F("k "));
  int angle = (int)compAngleX;
  if (angle >= 0 && angle < 10) display.print(F(" "));
  display.print(angle);
  display.print(F("d"));
  
  if (emergencyStop || current_A > BUZZER_CURRENT_THRESHOLD) {
    display.setCursor(114, 3);
    display.print(F("!!"));
  } else if (tiltWarning) {
    display.setCursor(120, 3);
    display.print(F("!"));
  }
  
  display.setTextSize(2);
  
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
      Serial.println(F("IR unlock hall"));
    }
    
    if (manualMode) {
      wheelDown = !wheelDown;
    } else {
      manualMode = true;
      wheelDown = !wheelDown;
    }
    
    ctrlSrc = 2;
    
    beep(NOTE_A5, 40);
    delay(30);
    if (wheelDown) soundDown();
    else soundUp();
    
    showWheelAnim = true;
    wheelAnimStartTime = millis();
    
    Serial.print(F("IR: "));
    Serial.println(manualMode ? F("Manual") : F("Auto"));
  }
}

// ============================================================================
// Setup
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(100);
  
  Wire.begin();
  
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(HALL_STOP_UP_PIN, INPUT_PULLUP);
  pinMode(HALL_STOP_DOWN_PIN, INPUT_PULLUP);
  pinMode(HALL_SPEED_PIN, INPUT_PULLUP);
  pinMode(EMERGENCY_STOP_PIN, INPUT_PULLUP);  // 【修正】改為 INPUT_PULLUP
  pinMode(PWM_CONTROL_PIN, OUTPUT);
  pinMode(RGB_RED_PIN, OUTPUT);
  pinMode(RGB_GREEN_PIN, OUTPUT);
  pinMode(RGB_BLUE_PIN, OUTPUT);
  
  digitalWrite(RELAY_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  analogWrite(PWM_CONTROL_PIN, 0);
  setRGBColor(RGB_OFF);
  
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
  attachInterrupt(digitalPinToInterrupt(HALL_SPEED_PIN), speedInterrupt, FALLING);
  
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("OLED Fail"));
    setRGBColor(RGB_RED);
    beep(NOTE_C5, 500);
    while (1);
  }
  
  ina226.init();
  Serial.println(F("INA226 OK"));
  
  setupMPU();
  calibrateIMU();
  
  Serial.println(F("Testing RGB..."));
  setRGBColor(RGB_RED);
  delay(500);
  setRGBColor(RGB_GREEN);
  delay(500);
  setRGBColor(RGB_BLUE);
  delay(500);
  setRGBColor(RGB_YELLOW);
  delay(500);
  setRGBColor(RGB_PURPLE);
  delay(500);
  setRGBColor(RGB_OFF);
  
  showStartup();
  soundStartup();
  
  Serial.println(F("=== READY ==="));
  Serial.println(F("Speed threshold: 10 km/h"));
  Serial.println(F("Wheel UP when speed >= 10 km/h"));
  Serial.println(F("Wheel DOWN when speed < 10 km/h"));
  setRGBColor(RGB_GREEN);
}

// ============================================================================
// Loop - 核心邏輯（修正版）
// ============================================================================
void loop() {
  uint32_t currentMillis = millis();
  
  // ========================================================================
  // 【階段 1】讀取感測器
  // ========================================================================
  float ax, gx;
  readIMU(ax, gx);
  
  current_A = ina226.getCurrent_A();
  
  if (current_A > BUZZER_CURRENT_THRESHOLD) {
    tone(BUZZER_PIN, 1000);
  } else {
    noTone(BUZZER_PIN);
  }
  
  // 【修正】速度更新邏輯
  if (newSpeedData) {
    currentSpeed = calculateSpeed();
    newSpeedData = false;
    lastSpeedUpdate = millis();
    
    // 【除錯】輸出速度資訊
    Serial.print(F("[SPEED] Pulse #"));
    Serial.print(speedPulseCount);
    Serial.print(F(" | Speed: "));
    Serial.print(currentSpeed, 2);
    Serial.print(F(" km/h | Interval: "));
    Serial.print(timeBetweenTriggers / 1000.0, 2);
    Serial.println(F(" ms"));
  }
  
  if (millis() - lastSpeedUpdate > SPEED_TIMEOUT) {
    if (currentSpeed != 0.0) {
      Serial.println(F("[SPEED] Timeout - Speed = 0"));
    }
    currentSpeed = 0.0;
  }
  
  // 紅外線接收
  if (IrReceiver.decode()) {
    handleIRCommand(IrReceiver.decodedIRData.decodedRawData);
    IrReceiver.resume();
  }
  
  // 霍爾停止檢測
  byte hallStopUp = digitalRead(HALL_STOP_UP_PIN);
  byte hallStopDown = digitalRead(HALL_STOP_DOWN_PIN);
  
  if (hallStopUp != lastHallStopUpState) {
    lastHallStopUpState = hallStopUp;
    
    if (hallStopUp == LOW) {
      hallStopLocked = true;
      manualMode = false;
      wheelDown = false;
      ctrlSrc = 1;
      
      soundLock();
      showWheelAnim = true;
      wheelAnimStartTime = millis();
      
      Serial.println(F("Hall UP lock"));
    }
  }
  
  if (hallStopDown != lastHallStopDownState) {
    lastHallStopDownState = hallStopDown;
    
    if (hallStopDown == LOW) {
      hallStopLocked = true;
      manualMode = false;
      wheelDown = true;
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
  if (relayCmd == HIGH) {
    if (currentSystemState == HALTED || currentSystemState == COOLING_DOWN) {
      int potValue = analogRead(SPEED_POT_PIN);
      int pwmSpeed = map(potValue, 0, 1023, 0, 255);
      analogWrite(PWM_CONTROL_PIN, pwmSpeed);
      pwmState = true;
    }
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
    analogWrite(PWM_CONTROL_PIN, 0);
    pwmState = false;
  }
  
  // ========================================================================
  // 【階段 3】三層決策邏輯（修正優先權）
  // ========================================================================
  
  relayCmd = LOW;
  bool allowLowerPriority = true;
  RGBColor targetRGB = RGB_GREEN;
  
  // 🔴 第一層：安全層（最高優先）
  bool isOverloaded = (current_A > MAX_CURRENT_A);
  bool emergencyButtonPressed = (digitalRead(EMERGENCY_STOP_PIN) == LOW);  // 【修正】按下時為 LOW
  bool haltConditionActive = isOverloaded || emergencyButtonPressed;
  
  switch (currentSystemState) {
    case RUNNING:
      if (haltConditionActive) {
        currentSystemState = HALTED;
        haltTime = currentMillis;
        soundEmergency();
        Serial.println(F("HALTED - Emergency button or overload"));
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
      Serial.println(F("EMERGENCY TILT!"));
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
    Serial.println(F("Emergency tilt cleared"));
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
        Serial.println(F("TILT WARNING"));
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
            Serial.println(F("Tilt OK - Auto UP"));
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
  
  // 🟢 第三層：操作層（霍爾鎖定 > 手動 > 自動速度控制）
  if (allowLowerPriority) {
    // 1. 霍爾停止鎖定（最高優先）
    if (hallStopLocked) {
      relayCmd = wheelDown ? HIGH : LOW;
      targetRGB = wheelDown ? RGB_YELLOW : RGB_GREEN;
    } 
    // 2. 手動模式（IR 控制）
    else if (manualMode) {
      relayCmd = wheelDown ? HIGH : LOW;
      targetRGB = wheelDown ? RGB_YELLOW : RGB_GREEN;
    } 
    // 3. 【核心修正】自動速度控制
    else {
      // 速度邏輯：速度 < 10 km/h → 輔助輪 DOWN（繼電器 ON）
      //          速度 >= 10 km/h → 輔助輪 UP（繼電器 OFF）
      bool shouldWheelDown = (currentSpeed < SPEED_THRESHOLD);
      
      if (shouldWheelDown != wheelDown) {
        wheelDown = shouldWheelDown;
        ctrlSrc = 3;
        
        if (wheelDown) soundDown();
        else soundUp();
        
        showWheelAnim = true;
        wheelAnimStartTime = millis();
        
        Serial.print(F("AUTO SPEED: "));
        Serial.print(currentSpeed, 1);
        Serial.print(F(" km/h -> Wheel "));
        Serial.println(wheelDown ? F("DOWN") : F("UP"));
      }
      
      relayCmd = wheelDown ? HIGH : LOW;
      targetRGB = (currentSpeed < SPEED_THRESHOLD) ? RGB_YELLOW : RGB_GREEN;
    }
  }
  
  // ========================================================================
  // 【階段 4】最終輸出
  // ========================================================================
  
  // 繼電器輸出（如果是低電平觸發，請改為 !relayCmd）
  digitalWrite(RELAY_PIN, relayCmd);
  
  // RGB LED 輸出
  setRGBColor(targetRGB);
  
  // ========================================================================
  // 【階段 5】顯示更新
  // ========================================================================
  
  if (millis() - lastDisplayTime > 50) {
    showStatus();
    lastDisplayTime = millis();
  }
  
  // 序列埠輸出（增強版除錯）
  if (millis() - lastPrintTime > 500) {
    Serial.print(F("State:"));
    switch (currentSystemState) {
      case RUNNING: Serial.print(F("RUN")); break;
      case HALTED: Serial.print(F("HALT")); break;
      case COOLING_DOWN: Serial.print(F("COOL")); break;
    }
    
    Serial.print(F(" | Speed:"));
    Serial.print(currentSpeed, 1);
    Serial.print(F("km/h Angle:"));
    Serial.print((int)compAngleX);
    Serial.print(F("° Current:"));
    Serial.print(current_A, 2);
    Serial.print(F("A"));
    
    Serial.print(F(" | Relay:"));
    Serial.print(relayCmd ? F("ON") : F("OFF"));
    Serial.print(F(" Wheel:"));
    Serial.print(wheelDown ? F("DOWN") : F("UP"));
    Serial.print(F(" PWM:"));
    Serial.print(pwmState ? F("ON") : F("OFF"));
    
    Serial.print(F(" | HallUp:"));
    Serial.print(digitalRead(HALL_STOP_UP_PIN) == LOW ? F("TRIG") : F("---"));
    Serial.print(F(" HallDn:"));
    Serial.print(digitalRead(HALL_STOP_DOWN_PIN) == LOW ? F("TRIG") : F("---"));
    Serial.print(F(" EmgBtn:"));
    Serial.print(digitalRead(EMERGENCY_STOP_PIN) == LOW ? F("PRESS") : F("---"));
    
    Serial.print(F(" | Lock:"));
    Serial.print(hallStopLocked ? F("Y") : F("N"));
    Serial.print(F(" Manual:"));
    Serial.print(manualMode ? F("Y") : F("N"));
    Serial.print(F(" Src:"));
    Serial.print(ctrlSrc);
    Serial.print(F(" Pulses:"));
    Serial.println(speedPulseCount);
    
    lastPrintTime = millis();
  }
  
  delay(10);
}
