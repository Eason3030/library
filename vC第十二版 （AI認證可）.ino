// ============================================================================
// 自行車輔助輪智能控制系統 v4.6 - 優先權架構修正版 (Final)
//
// 【優先權設計】(由高到低，數字越小優先權越高)
// 0. 緊急停止 (PRIORITY_EMERGENCY) - 絕對最高
// 1. 安全保護 (PRIORITY_SAFETY_DOWN) - 傾斜/速度過低強制下降
// 2. 冷卻鎖定 (PRIORITY_COOLING) - 5秒恢復期
// 3. 手動控制 (PRIORITY_MANUAL) - 紅外線遙控器
// 4. 自動控制 (PRIORITY_AUTO) - 速度判斷
//
// 【架構原則】
// - 所有系統流程只在 loop() 執行一次
// - handleIRCommand() 只改變意圖，不執行動作
// - SystemState 記錄系統生命週期，Priority 控制決策權
// ============================================================================

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <IRremote.h>
#include <INA226_WE.h>

// ============================================================================
// OLED 螢幕設定
// ============================================================================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ============================================================================
// 系統設定
// ============================================================================
const bool BUTTON_IS_NC_TYPE = true;
const bool IGNORE_CURRENT_SENSOR = true;

// ============================================================================
// 音符定義
// ============================================================================
#define NOTE_C5  523
#define NOTE_E5  659
#define NOTE_G5  784
#define NOTE_C6  1047
#define NOTE_A5  880

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
const byte RELAY_PIN = 11;
const byte IR_RECEIVE_PIN = 3;
const byte LIMIT_UP_PIN = 4;
const byte LIMIT_DOWN_PIN = 10;
const byte HALL_SPEED_PIN = 2;
const byte EMERGENCY_STOP_PIN = 12;
const byte PWM_CONTROL_PIN = 9;
const byte SPEED_POT_PIN = A0;

const byte RGB_RED_PIN = 5;
const byte RGB_GREEN_PIN = 7;
const byte RGB_BLUE_PIN = 6;
const bool RGB_COMMON_ANODE = false;

// ============================================================================
// 狀態定義
// ============================================================================

// 推桿運動狀態
enum MotionState : byte {
  IDLE,
  MOVING_UP,
  MOVING_DOWN
};

// ⭐ 控制優先權 (數字越小 = 優先權越高)
enum ControlPriority : byte {
  PRIORITY_EMERGENCY = 0,      // 第0級:緊急停止 (最高)
  PRIORITY_SAFETY_DOWN = 1,    // 第1級:安全保護 (強制下降)
  PRIORITY_COOLING = 2,        // 第2級:冷卻鎖定
  PRIORITY_MANUAL = 3,         // 第3級:手動控制
  PRIORITY_AUTO = 4            // 第4級:自動控制 (最低)
};

// 系統生命週期狀態 (用於記錄歷史，不參與決策)
enum SystemState : byte {
  RUNNING,
  COOLING_DOWN,
  EMERGENCY
};

// RGB LED 顏色
enum RGBColor : byte {
  RGB_OFF,
  RGB_GREEN,
  RGB_YELLOW,
  RGB_BLUE,
  RGB_RED,
  RGB_PURPLE
};

// ============================================================================
// 全域變數
// ============================================================================

INA226_WE ina226;
const float MAX_CURRENT_A = 5.0;
float current_A = 0.0;

MotionState motionState = IDLE;
SystemState currentSystemState = RUNNING;
ControlPriority currentPriority = PRIORITY_AUTO;

// 速度測量
#define WHEEL_CIRCUMFERENCE 204.2
const float SPEED_THRESHOLD = 10.0;
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

// 傾斜保護
const float SAFE_ANGLE = 20.0f;
const float TILT_SAFE = 12.0f;
const float EMERGENCY_ANGLE = 35.0f;
const uint16_t TILT_TRIGGER_TIME = 500;
const uint16_t AUTO_UP_DELAY = 3000;
uint32_t tiltStartTime = 0;
bool tiltWarning = false;
uint32_t safeStartTime = 0;

// 速度防抖與遲滯
const byte SPEED_STABLE_COUNT = 3;        // 需要連續3次成立
byte speedLowCount = 0;                   // 速度過低計數
byte speedHighCount = 0;                  // 速度夠高計數
const float SPEED_UP_THRESHOLD = 12.0;    // 升起門檻 12 km/h
const float SPEED_DOWN_THRESHOLD = 8.0;   // 下降門檻 8 km/h

// 控制變數
bool wheelDown = false;
bool manualMode = false;
byte ctrlSrc = 0;

// 限位鎖定
bool limitUpLocked = false;
bool limitDownLocked = false;

// 時間控制
uint32_t haltTime = 0;
const uint16_t RESTART_DELAY_MS = 5000;
uint32_t lastPrintTime = 0;
uint32_t lastDisplayTime = 0;
uint32_t lastIRCode = 0;
const uint16_t IR_DELAY = 500;

// 非阻塞音效
uint32_t beepEndTime = 0;
bool isBeeping = false;

// 看門狗
uint32_t lastLoopTime = 0;
const uint32_t WATCHDOG_TIMEOUT = 2000;

// PWM 控制
int targetPWM = 0;

// 緊急停止狀態
bool emergencyStopPressed = false;

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
    case RGB_YELLOW: setRGB(255,255,0); break;
    case RGB_BLUE: setRGB(0,0,255); break;
    case RGB_RED: setRGB(255,0,0); break;
    case RGB_PURPLE: setRGB(128,0,255); break;
  }
}

RGBColor decideRGB() {
  switch(currentPriority) {
    case PRIORITY_EMERGENCY:
    case PRIORITY_COOLING:
      return RGB_RED;
    case PRIORITY_SAFETY_DOWN:
      return RGB_BLUE;
    case PRIORITY_MANUAL:
      return RGB_PURPLE;
    default:
      if (motionState == MOVING_DOWN) return RGB_YELLOW;
      if (motionState == MOVING_UP) return RGB_GREEN;
      return wheelDown ? RGB_YELLOW : RGB_GREEN;
  }
}

// ============================================================================
// 非阻塞音效
// ============================================================================

void beepStart(uint16_t freq, uint16_t duration) {
  tone(BUZZER_PIN, freq);
  beepEndTime = millis() + duration;
  isBeeping = true;
}

void beepUpdate() {
  if (isBeeping && millis() >= beepEndTime) {
    noTone(BUZZER_PIN);
    isBeeping = false;
  }
}

// ============================================================================
// 速度計算
// ============================================================================

void speedInterrupt() {
  uint32_t t = micros();
  if (t - lastSpeedTrigger > 50000) {
    timeBetweenTriggers = t - lastSpeedTrigger;
    lastSpeedTrigger = t;
    newSpeedData = true;
  }
}

float calculateSpeed() {
  if (timeBetweenTriggers == 0) return 0.0;
  float timeInSec = timeBetweenTriggers/1000000.0;
  return (WHEEL_CIRCUMFERENCE/timeInSec)*0.036;
}

// ============================================================================
// MPU6050 函式
// ============================================================================

int16_t read16(byte reg) {
  Wire.beginTransmission(MPU_6050);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((byte)MPU_6050,(byte)2);
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
  float sumAx=0, sumGx=0;
  for(byte i=0; i<50; i++) {
    sumAx += read16(ACCEL_XOUT_H)/16384.0f;
    sumGx += read16(GYRO_XOUT_H)/131.0f;
    delay(5);
  }
  accelBiasX = sumAx/50;
  gyroBiasX = sumGx/50;
}

void readIMU(float &ax, float &gx) {
  ax = read16(ACCEL_XOUT_H)/16384.0f;
  float az = read16(ACCEL_XOUT_H+4)/16384.0f;
  gx = read16(GYRO_XOUT_H)/131.0f;
  
  gx -= gyroBiasX;
  ax -= accelBiasX;
  
  float accelAngle = atan2(ax,az)*57.2958f;
  compAngleX = 0.98*(compAngleX+gx*0.01)+0.02*accelAngle;
}

// ============================================================================
// OLED 顯示
// ============================================================================

void showStartupPro() {
  display.clearDisplay();
  display.setTextSize(3);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(15,10);
  display.print(F("BIKE"));
  display.setCursor(10,40);
  display.setTextSize(2);
  display.print(F("v4.6"));
  display.display();
  delay(1000);
}

void drawDashboard() {
  display.clearDisplay();
  
  // 速度顯示
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(4);
  int speedInt = (int)currentSpeed;
  int xPos = (speedInt < 10) ? 40 : 10;
  display.setCursor(xPos, 5);
  display.print(speedInt);
  
  // 單位
  display.setTextSize(2);
  display.setCursor(85, 15);
  display.print(F("km"));
  display.setCursor(85, 30);
  display.print(F("/h"));
  
  // 底部資訊
  display.setTextSize(1);
  display.setCursor(2, 54);
  
  // 角度
  display.print(F("A:"));
  int angleInt = (int)abs(compAngleX);
  if(angleInt < 10) display.print(F(" "));
  display.print(angleInt);
  display.print(F("'"));
  
  if(compAngleX > 5) display.print(F("R"));
  else if(compAngleX < -5) display.print(F("L"));
  else display.print(F("-"));
  
  display.print(F(" |"));
  
  // 運動狀態
  if(motionState == MOVING_UP) {
    display.print(F(" ^UP"));
  } else if(motionState == MOVING_DOWN) {
    display.print(F(" vDN"));
  } else {
    display.print(F(" --"));
  }
  
  // ⭐ 優先權狀態顯示 (用於除錯與展示系統決策)
  display.setCursor(2, 45);
  switch(currentPriority) {
    case PRIORITY_EMERGENCY:
      display.print(F("EMERGENCY!"));
      break;
    case PRIORITY_SAFETY_DOWN:
      display.print(F("SAFETY"));
      break;
    case PRIORITY_COOLING:
      display.print(F("COOLING"));
      break;
    case PRIORITY_MANUAL:
      display.print(F("MANUAL"));
      break;
    case PRIORITY_AUTO:
      display.print(F("AUTO"));
      break;
  }
  
  display.display();
}

// ============================================================================
// ⭐ 紅外線遙控器處理 (修正版 - 只改變意圖，不執行動作)
// ============================================================================

/**
 * 處理紅外線遙控器指令
 * ⚠️ 架構原則:只設定意圖,不執行系統流程
 * 所有動作(executeMotion, RGB, OLED)都在 loop() 統一執行
 */
void handleIRCommand(uint32_t code) {
  // 防抖動
  if(millis() - lastIRCode < IR_DELAY) {
    return;
  }
  lastIRCode = millis();
  
  // ⭐ 優先權檢查:只有在 MANUAL 或 AUTO 優先權時才允許控制
  if(currentPriority < PRIORITY_MANUAL) {
    Serial.println(F("IR: Blocked by higher priority"));
    beepStart(NOTE_C5, 100);  // 短促警告音
    return;
  }
  
  // 解除限位鎖定
  if(limitUpLocked || limitDownLocked) {
    limitUpLocked = false;
    limitDownLocked = false;
    Serial.println(F("IR: Unlock limits"));
  }
  
  // ⭐ 只改變意圖變數
  wheelDown = !wheelDown;
  manualMode = true;
  ctrlSrc = 2;
  
  beepStart(NOTE_A5, 50);
  Serial.print(F("IR: Target "));
  Serial.println(wheelDown ? F("DOWN") : F("UP"));
  
  // ❌ 刪除所有執行動作的程式碼:
  // - executeMotion()
  // - setRGBColor()
  // - drawDashboard()
  // - Serial.print(大量資訊)
  // - delay()
  // 這些都在 loop() 統一處理
}

// ============================================================================
// ⭐ 優先權決策核心
// ============================================================================

/**
 * 決定當前的控制優先權
 * 這是整個系統的「指揮官」
 */
void decidePriority() {
  
  // 【第0級】緊急停止 - 絕對最高
  if (emergencyStopPressed || 
      (!IGNORE_CURRENT_SENSOR && current_A > MAX_CURRENT_A)) {
    currentPriority = PRIORITY_EMERGENCY;
    return;
  }
  
  // 【第0級】極端傾斜 - 視為緊急
  if (abs(compAngleX) > EMERGENCY_ANGLE) {
    currentPriority = PRIORITY_EMERGENCY;
    return;
  }
  
  // 【第1級】安全保護 - 傾斜或速度過低
  if (tiltWarning || 
      (currentSpeed < SPEED_DOWN_THRESHOLD && speedLowCount >= SPEED_STABLE_COUNT)) {
    currentPriority = PRIORITY_SAFETY_DOWN;
    return;
  }
  
  // 【第2級】冷卻鎖定
  if (currentSystemState == COOLING_DOWN) {
    currentPriority = PRIORITY_COOLING;
    return;
  }
  
  // 【第3級】手動控制
  if (manualMode) {
    currentPriority = PRIORITY_MANUAL;
    return;
  }
  
  // 【第4級】自動控制(預設)
  currentPriority = PRIORITY_AUTO;
}

// ============================================================================
// ⭐ 根據優先權執行動作
// ============================================================================

/**
 * 根據當前優先權決定輔助輪的目標狀態
 * ⚠️ 職責清晰:只設定 wheelDown 等控制變數
 */
void executeByPriority() {
  
  switch(currentPriority) {
    
    case PRIORITY_EMERGENCY:
      // 緊急狀態:強制下降,忽略所有限位
      wheelDown = true;
      limitUpLocked = false;
      limitDownLocked = false;
      manualMode = false;
      ctrlSrc = 0;
      
      // 進入冷卻
      if(currentSystemState != COOLING_DOWN && currentSystemState != EMERGENCY) {
        currentSystemState = COOLING_DOWN;
        haltTime = millis();
        beepStart(NOTE_C6, 300);
        Serial.println(F("!!! EMERGENCY TRIGGERED !!!"));
      }
      break;
      
    case PRIORITY_SAFETY_DOWN:
      // 安全保護:強制下降
      wheelDown = true;
      manualMode = false;  // 取消手動模式
      
      if(limitDownLocked) {
        limitDownLocked = false;  // 解除下限鎖定,允許下降
      }
      
      ctrlSrc = 4;  // 標記為安全保護控制
      break;
      
    case PRIORITY_COOLING:
      // ⭐ 修正:冷卻期間明確定義行為
      wheelDown = true;      // 維持安全狀態(輔助輪在下)
      manualMode = false;    // 禁止手動介入
      ctrlSrc = 0;           // 無控制來源
      // 不改變限位鎖定,讓馬達保持停止
      break;
      
    case PRIORITY_MANUAL:
      // 手動控制:由 IR 決定,這裡不覆蓋 wheelDown
      ctrlSrc = 2;
      break;
      
    case PRIORITY_AUTO:
      // 自動控制:根據速度決定
      
      // 速度遲滯:上升需要更高門檻
      if(currentSpeed >= SPEED_UP_THRESHOLD && speedHighCount >= SPEED_STABLE_COUNT) {
        if(wheelDown && !limitUpLocked) {
          wheelDown = false;
          ctrlSrc = 3;
          limitUpLocked = false;
          Serial.println(F("Auto: UP (speed >= 12)"));
        }
      } 
      else if(currentSpeed < SPEED_DOWN_THRESHOLD && speedLowCount >= SPEED_STABLE_COUNT) {
        if(!wheelDown && !limitDownLocked) {
          wheelDown = true;
          ctrlSrc = 3;
          limitDownLocked = false;
          Serial.println(F("Auto: DOWN (speed < 8)"));
        }
      }
      break;
  }
}

// ============================================================================
// 運動控制核心
// ============================================================================

/**
 * 執行推桿運動控制
 * ⚠️ 修正:不再檢查 priority,只根據 motionState 和 wheelDown 運作
 */
void executeMotion() {
  bool limitUpTriggered = (digitalRead(LIMIT_UP_PIN) == LOW);
  bool limitDownTriggered = (digitalRead(LIMIT_DOWN_PIN) == LOW);
  
  // 限位觸發檢查
  if(limitUpTriggered && !limitUpLocked && motionState == MOVING_UP) {
    limitUpLocked = true;
    motionState = IDLE;
    ctrlSrc = 1;
    beepStart(NOTE_C6, 50);
    Serial.println(F("<<< UP LIMIT"));
  }
  
  if(limitDownTriggered && !limitDownLocked && motionState == MOVING_DOWN) {
    limitDownLocked = true;
    motionState = IDLE;
    ctrlSrc = 1;
    beepStart(NOTE_G5, 50);
    Serial.println(F("<<< DOWN LIMIT"));
  }
  
  // 狀態機
  switch(motionState) {
    case IDLE:
      // 只在 RUNNING 狀態才允許啟動
      if(currentSystemState == RUNNING) {
        
        if(wheelDown && !limitDownLocked) {
          digitalWrite(RELAY_PIN, LOW);
          motionState = MOVING_DOWN;
          limitDownLocked = false;
          Serial.println(F(">>> START DOWN"));
        }
        else if(!wheelDown && !limitUpLocked) {
          digitalWrite(RELAY_PIN, HIGH);
          motionState = MOVING_UP;
          limitUpLocked = false;
          Serial.println(F(">>> START UP"));
        }
      }
      targetPWM = 0;
      break;
      
    case MOVING_UP:
      if(limitUpLocked) {
        motionState = IDLE;
        targetPWM = 0;
        Serial.println(F("<<< UP COMPLETE"));
      }
      else if(wheelDown) {
        motionState = IDLE;
        targetPWM = 0;
        Serial.println(F("<<< UP ABORT"));
      }
      else {
        int potValue = analogRead(SPEED_POT_PIN);
        targetPWM = map(potValue, 0, 1023, 120, 255);
      }
      break;
      
    case MOVING_DOWN:
      if(limitDownLocked) {
        motionState = IDLE;
        targetPWM = 0;
        Serial.println(F("<<< DOWN COMPLETE"));
      }
      else if(!wheelDown) {
        motionState = IDLE;
        targetPWM = 0;
        Serial.println(F("<<< DOWN ABORT"));
      }
      else {
        int potValue = analogRead(SPEED_POT_PIN);
        targetPWM = map(potValue, 0, 1023, 120, 255);
      }
      break;
  }
  
  // ⭐ 修正:冷卻/緊急時透過 SystemState 控制,不再檢查 priority
  if(currentSystemState != RUNNING) {
    motionState = IDLE;
    targetPWM = 0;
  }
  
  // 輸出 PWM
  analogWrite(PWM_CONTROL_PIN, targetPWM);
}

// ============================================================================
// Setup
// ============================================================================

void setup() {
  Serial.begin(115200);
  delay(100);
  
  Wire.begin();
  Wire.setClock(100000);
  
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(LIMIT_UP_PIN, INPUT_PULLUP);
  pinMode(LIMIT_DOWN_PIN, INPUT_PULLUP);
  pinMode(HALL_SPEED_PIN, INPUT_PULLUP);
  pinMode(EMERGENCY_STOP_PIN, INPUT_PULLUP);
  pinMode(PWM_CONTROL_PIN, OUTPUT);
  pinMode(RGB_RED_PIN, OUTPUT);
  pinMode(RGB_GREEN_PIN, OUTPUT);
  pinMode(RGB_BLUE_PIN, OUTPUT);
  
  digitalWrite(RELAY_PIN, LOW);
  analogWrite(PWM_CONTROL_PIN, 0);
  setRGBColor(RGB_OFF);
  
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
  attachInterrupt(digitalPinToInterrupt(HALL_SPEED_PIN), speedInterrupt, FALLING);
  
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("OLED Fail"));
    while(1);
  }
  
  ina226.init();
  setupMPU();
  calibrateIMU();
  
  showStartupPro();
  beepStart(NOTE_C6, 100);
  
  Serial.println(F("============================"));
  Serial.println(F("  BIKE ASSIST v4.6 Final"));
  Serial.println(F("  優先權架構修正版"));
  Serial.println(F("============================"));
  
  lastLoopTime = millis();
}

// ============================================================================
// Loop 主迴圈 (系統流程只執行一次)
// ============================================================================

void loop() {
  uint32_t currentMillis = millis();
  
  // ========== 看門狗 ==========
  if(currentMillis - lastLoopTime > WATCHDOG_TIMEOUT) {
    Serial.println(F("!!! WATCHDOG RESET !!!"));
    motionState = IDLE;
    currentSystemState = RUNNING;
    currentPriority = PRIORITY_AUTO;
    targetPWM = 0;
    analogWrite(PWM_CONTROL_PIN, 0);
    digitalWrite(RELAY_PIN, LOW);
    tiltWarning = false;
    manualMode = false;
    beepStart(NOTE_A5, 300);
  }
  lastLoopTime = currentMillis;
  
  // ========== 非阻塞音效更新 ==========
  beepUpdate();
  
  // ========== 讀取感測器 ==========
  float ax, gx;
  readIMU(ax, gx);
  
  current_A = ina226.getCurrent_A();
  if(isnan(current_A)) current_A = 0.0;
  
  if(newSpeedData) {
    currentSpeed = calculateSpeed();
    newSpeedData = false;
    lastSpeedUpdate = millis();
  }
  
  if(millis() - lastSpeedUpdate > SPEED_TIMEOUT) {
    currentSpeed = 0.0;
  }
  
  // ========== 速度防抖計數 ==========
  if(currentSpeed < SPEED_DOWN_THRESHOLD) {
    speedLowCount = min(speedLowCount + 1, SPEED_STABLE_COUNT);
    speedHighCount = 0;
  } else if(currentSpeed >= SPEED_UP_THRESHOLD) {
    speedHighCount = min(speedHighCount + 1, SPEED_STABLE_COUNT);
    speedLowCount = 0;
  } else {
    // 在遲滯區間,計數器遞減
    speedLowCount = (speedLowCount > 0) ? speedLowCount - 1 : 0;
    speedHighCount = (speedHighCount > 0) ? speedHighCount - 1 : 0;
  }
  
  // ========== 紅外線接收 ==========
  if(IrReceiver.decode()) {
    handleIRCommand(IrReceiver.decodedIRData.decodedRawData);
    IrReceiver.resume();
  }
  
  // ========== 緊急按鈕檢查 ==========
  emergencyStopPressed = (digitalRead(EMERGENCY_STOP_PIN) == LOW);
  
  // ========== 傾斜檢測 ==========
  if(abs(compAngleX) > SAFE_ANGLE) {
    if(tiltStartTime == 0) tiltStartTime = millis();
    if(!tiltWarning && (millis() - tiltStartTime >= TILT_TRIGGER_TIME)) {
      tiltWarning = true;
      Serial.println(F(">>> TILT WARN"));
    }
  } else {
    tiltStartTime = 0;
    if(tiltWarning && abs(compAngleX) < TILT_SAFE) {
      if(safeStartTime == 0) safeStartTime = millis();
      if(millis() - safeStartTime >= AUTO_UP_DELAY) {
        tiltWarning = false;
        if(!manualMode && currentPriority == PRIORITY_AUTO) {
          limitUpLocked = false;
        }
        Serial.println(F("<<< TILT OK"));
        safeStartTime = 0;
      }
    } else {
      safeStartTime = 0;
    }
  }
  
  // ========== ⭐⭐⭐ 核心:優先權決策 ==========
  decidePriority();
  
  // ========== ⭐⭐⭐ 核心:根據優先權執行動作 ==========
  executeByPriority();
  
  // ========== 冷卻恢復檢查 ==========
  if(currentSystemState == COOLING_DOWN) {
    if(currentMillis - haltTime >= RESTART_DELAY_MS) {
      currentSystemState = RUNNING;
      currentPriority = PRIORITY_AUTO;
      limitUpLocked = false;
      limitDownLocked = false;
      manualMode = false;
      emergencyStopPressed = false;
      Serial.println(F("<<< RECOVERED >>>"));
      beepStart(NOTE_C5, 100);
    }
  }
  
  // ========== 執行運動控制 ==========
  executeMotion();
  
  // ========== RGB LED 狀態顯示 ==========
  setRGBColor(decideRGB());
  
  // ========== OLED 螢幕更新(每 0.2 秒) ==========
  if(millis() - lastDisplayTime > 200) {
    lastDisplayTime = millis();
    drawDashboard();
  }
  
  // ========== 序列埠輸出(每 1 秒) ==========
  if(millis() - lastPrintTime > 1000) {
    lastPrintTime = millis();
    
    // ⭐ 優先權狀態顯示
    Serial.print(F("Priority:"));
    switch(currentPriority) {
      case PRIORITY_EMERGENCY: Serial.print(F("EMRG")); break;
      case PRIORITY_SAFETY_DOWN: Serial.print(F("SAFE")); break;
      case PRIORITY_COOLING: Serial.print(F("COOL")); break;
      case PRIORITY_MANUAL: Serial.print(F("MANU")); break;
      case PRIORITY_AUTO: Serial.print(F("AUTO")); break;
    }
    
    Serial.print(F(" |Sys:"));
    if(currentSystemState == RUNNING) 
      Serial.print(F("RUN"));
    else if(currentSystemState == COOLING_DOWN) 
      Serial.print(F("COOL"));
    else 
      Serial.print(F("EMRG"));
    
    Serial.print(F(" |Motion:"));
    if(motionState == MOVING_UP) 
      Serial.print(F("^UP^"));
    else if(motionState == MOVING_DOWN) 
      Serial.print(F("vDNv"));
    else 
      Serial.print(F("IDLE"));
    
    Serial.print(F(" |Target:"));
    Serial.print(wheelDown ? F("DN") : F("UP"));
    
    Serial.print(F(" |LimUP:"));
    if(limitUpLocked) 
      Serial.print(F("LOCK"));
    else 
      Serial.print(digitalRead(LIMIT_UP_PIN) == LOW ? F("ON") : F("--"));
    
    Serial.print(F(" |LimDN:"));
    if(limitDownLocked) 
      Serial.print(F("LOCK"));
    else 
      Serial.print(digitalRead(LIMIT_DOWN_PIN) == LOW ? F("ON") : F("--"));
    
    Serial.print(F(" |Spd:"));
    Serial.print(currentSpeed, 1);
    Serial.print(F("km/h"));
    
    Serial.print(F(" |Ang:"));
    Serial.print(compAngleX, 1);
    Serial.print(F("'"));
    
    Serial.print(F(" |PWM:"));
    Serial.print(targetPWM);
    
    Serial.print(F(" |I:"));
    Serial.print(current_A, 2);
    Serial.print(F("A"));
    
    // ⭐ 速度防抖計數顯示
    Serial.print(F(" |SpdCnt:L"));
    Serial.print(speedLowCount);
    Serial.print(F("/H"));
    Serial.print(speedHighCount);
    
    // 警告標記
    if(tiltWarning) 
      Serial.print(F(" [TILT!]"));
    if(manualMode) 
      Serial.print(F(" [MANUAL]"));
    if(emergencyStopPressed)
      Serial.print(F(" [EMRG-BTN]"));
    
    // 控制來源
    Serial.print(F(" Src:"));
    switch(ctrlSrc) {
      case 0: Serial.print(F("--")); break;
      case 1: Serial.print(F("LIM")); break;
      case 2: Serial.print(F("IR")); break;
      case 3: Serial.print(F("AUTO")); break;
      case 4: Serial.print(F("TILT")); break;
    }
    
    Serial.println();
  }
  
  // ========== 小延遲防止 CPU 滿載 ==========
  delay(5);
}
