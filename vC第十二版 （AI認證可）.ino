// ============================================================================
// 自行車輔助輪智能控制系統 v4.7 - 穩定融合版
// 
// 【融合特性】
// ✅ v4.6 完整優先權邏輯
// ✅ v4.6.1 硬體保護機制（看門狗、I2C 超時、軟啟動）
// ✅ I2C 健康檢查與自動恢復
// ✅ 馬達軟啟動防止電壓突降
// ============================================================================

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <IRremote.h>
#include <INA226_WE.h>
#include <avr/wdt.h> // 硬體看門狗

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

enum MotionState : byte {
  IDLE,
  MOVING_UP,
  MOVING_DOWN
};

enum ControlPriority : byte {
  PRIORITY_EMERGENCY = 0,      // 第0級:緊急停止
  PRIORITY_SAFETY_DOWN = 1,    // 第1級:安全保護
  PRIORITY_COOLING = 2,        // 第2級:冷卻鎖定
  PRIORITY_MANUAL = 3,         // 第3級:手動控制
  PRIORITY_AUTO = 4            // 第4級:自動控制
};

enum SystemState : byte {
  RUNNING,
  COOLING_DOWN,
  EMERGENCY
};

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
const uint16_t SPEED_TIMEOUT = 2000;
volatile uint32_t lastSpeedTrigger = 0;
volatile uint32_t timeBetweenTriggers = 0;
volatile bool newSpeedData = false;
float currentSpeed = 0.0;
uint32_t lastSpeedUpdate = 0;

// IMU 資料與健康狀態
float compAngleX = 0.0f;
float gyroBiasX = 0.0f;
float accelBiasX = 0.0f;
uint32_t lastIMUTime = 0;
bool i2cHealthy = true;
uint32_t lastI2CSuccess = 0;
const uint16_t I2C_TIMEOUT = 5000;

// 傾斜保護
const float SAFE_ANGLE = 10.0f;
const float TILT_SAFE = 5.0f;
const float EMERGENCY_ANGLE = 15.0f;
const uint16_t TILT_TRIGGER_TIME = 500;
const uint16_t AUTO_UP_DELAY = 3000;
uint32_t tiltStartTime = 0;
bool tiltWarning = false;
uint32_t safeStartTime = 0;

// 速度防抖與遲滯
const byte SPEED_STABLE_COUNT = 3;
byte speedLowCount = 0;
byte speedHighCount = 0;
const float SPEED_UP_THRESHOLD = 12.0;
const float SPEED_DOWN_THRESHOLD = 8.0;

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

// PWM 軟啟動
int targetPWM = 0;
int currentPWM = 0;

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
  if (t - lastSpeedTrigger > 40000) {  // 防抖門檻
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
// MPU6050 函式 - 安全版本
// ============================================================================

int16_t safeRead16(byte reg) {
  Wire.beginTransmission(MPU_6050);
  Wire.write(reg);
  byte error = Wire.endTransmission(false);

  if(error != 0) {
    i2cHealthy = false;
    return 0;
  }

  Wire.requestFrom((byte)MPU_6050, (byte)2);
  uint32_t start = millis();
  while (Wire.available() < 2) {
    if (millis() - start > 10) {  // 10ms 超時保護
      i2cHealthy = false;
      return 0;
    }
  }

  i2cHealthy = true;
  lastI2CSuccess = millis();
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
    sumAx += safeRead16(ACCEL_XOUT_H)/16384.0f;
    sumGx += safeRead16(GYRO_XOUT_H)/131.0f;
    delay(5);
  }
  accelBiasX = sumAx/50;
  gyroBiasX = sumGx/50;
}

void readIMU() {
  // 如果 I2C 不健康，使用上次的值
  if(!i2cHealthy) {
    return;
  }

  uint32_t now = micros();
  float dt = (now - lastIMUTime) / 1000000.0;
  lastIMUTime = now;
  if (dt > 0.1) dt = 0.01;  // 防止第一次讀取異常

  float ax = safeRead16(ACCEL_XOUT_H) / 16384.0f - accelBiasX;
  float az = safeRead16(ACCEL_XOUT_H + 4) / 16384.0f;
  float gx = safeRead16(GYRO_XOUT_H) / 131.0f - gyroBiasX;

  // 檢查數據合理性
  if(isnan(ax) || isnan(az) || isnan(gx) ||
     abs(ax) > 10 || abs(az) > 10 || abs(gx) > 500) {
    i2cHealthy = false;
    return;
  }

  float accelAngle = atan2(ax, az) * 57.2958f;
  compAngleX = 0.98 * (compAngleX + gx * dt) + 0.02 * accelAngle;

  // 限制角度範圍
  if(compAngleX > 180) compAngleX = 180;
  if(compAngleX < -180) compAngleX = -180;
}

// ============================================================================
// PWM 軟啟動
// ============================================================================

void updatePWM() {
  if (currentPWM < targetPWM) {
    currentPWM = min(currentPWM + 5, targetPWM);  // 慢慢加速
  } else if (currentPWM > targetPWM) {
    currentPWM = max(currentPWM - 10, targetPWM);  // 減速可以快一點
  }
  analogWrite(PWM_CONTROL_PIN, currentPWM);
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
  display.print(F("v4.7"));
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
    display.print(F(" –"));
  }

  // 優先權狀態顯示
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
// 紅外線遙控器處理
// ============================================================================

void handleIRCommand(uint32_t code) {
  // 防抖動
  if(millis() - lastIRCode < IR_DELAY) {
    return;
  }
  lastIRCode = millis();

  // 優先權檢查
  if(currentPriority < PRIORITY_MANUAL) {
    Serial.println(F("IR: Blocked by higher priority"));
    beepStart(NOTE_C5, 100);
    return;
  }

  // 解除限位鎖定
  if(limitUpLocked || limitDownLocked) {
    limitUpLocked = false;
    limitDownLocked = false;
    Serial.println(F("IR: Unlock limits"));
  }

  // 只改變意圖變數
  wheelDown = !wheelDown;
  manualMode = true;
  ctrlSrc = 2;

  beepStart(NOTE_A5, 50);
  Serial.print(F("IR: Target "));
  Serial.println(wheelDown ? F("DOWN") : F("UP"));
}

// ============================================================================
// 優先權決策核心
// ============================================================================

void decidePriority() {
  // 【第0級】緊急停止
  if (emergencyStopPressed ||
      (!IGNORE_CURRENT_SENSOR && current_A > MAX_CURRENT_A)) {
    currentPriority = PRIORITY_EMERGENCY;
    return;
  }

  // 【第0級】極端傾斜
  if (abs(compAngleX) > EMERGENCY_ANGLE) {
    currentPriority = PRIORITY_EMERGENCY;
    return;
  }

  // 【第1級】安全保護
  // ⭐ 修正：只有在輔助輪已經升起(wheelDown=false)時，才檢查速度過低
  if (tiltWarning) {
    currentPriority = PRIORITY_SAFETY_DOWN;
    return;
  }
  
  // 速度過低保護：只在輔助輪升起時才觸發
  if (!wheelDown && 
      currentSpeed < SPEED_DOWN_THRESHOLD && 
      speedLowCount >= SPEED_STABLE_COUNT) {
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

  // 【第4級】自動控制
  currentPriority = PRIORITY_AUTO;
}

// ============================================================================
// 根據優先權執行動作
// ============================================================================

void executeByPriority() {
  switch(currentPriority) {
    
    case PRIORITY_EMERGENCY:
      wheelDown = true;
      limitUpLocked = false;
      limitDownLocked = false;
      manualMode = false;
      ctrlSrc = 0;
      
      if(currentSystemState != COOLING_DOWN && currentSystemState != EMERGENCY) {
        currentSystemState = COOLING_DOWN;
        haltTime = millis();
        beepStart(NOTE_C6, 300);
        Serial.println(F("!!! EMERGENCY TRIGGERED !!!"));
      }
      break;
      
    case PRIORITY_SAFETY_DOWN:
      wheelDown = true;
      manualMode = false;
      
      if(limitDownLocked) {
        limitDownLocked = false;
      }
      
      ctrlSrc = 4;
      break;
      
    case PRIORITY_COOLING:
      wheelDown = true;
      manualMode = false;
      ctrlSrc = 0;
      break;
      
    case PRIORITY_MANUAL:
      ctrlSrc = 2;
      break;
      
    case PRIORITY_AUTO:
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

  // 冷卻/緊急時停止
  if(currentSystemState != RUNNING) {
    motionState = IDLE;
    targetPWM = 0;
  }

  // 軟啟動 PWM 輸出
  updatePWM();
}

// ============================================================================
// Setup
// ============================================================================

void setup() {
  wdt_disable();  // 初始化期間先關閉看門狗
  Serial.begin(115200);
  delay(100);

  Wire.begin();
  Wire.setClock(100000);
  
  #if defined(ARDUINO_ARCH_AVR)
    Wire.setWireTimeout(3000, true);  // 防止 I2C 線路卡死
  #endif

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
  Serial.println(F("  BIKE ASSIST v4.7"));
  Serial.println(F("  穩定融合版"));
  Serial.println(F("============================"));

  // 診斷資訊
  Serial.println(F("\n=== 系統診斷 ==="));
  Serial.print(F("Emergency Pin ("));
  Serial.print(EMERGENCY_STOP_PIN);
  Serial.print(F("): "));
  Serial.println(digitalRead(EMERGENCY_STOP_PIN) ? "HIGH" : "LOW");
  Serial.print(F("Button Type: "));
  Serial.println(BUTTON_IS_NC_TYPE ? "NC (常閉)" : "NO (常開)");

  Serial.println(F("\n=== IMU 狀態檢查 ==="));
  readIMU();
  Serial.print(F("Initial Angle: "));
  Serial.print(compAngleX, 2);
  Serial.println(F(" degrees"));
  Serial.print(F("I2C Health: "));
  Serial.println(i2cHealthy ? "OK" : "FAIL");

  if(abs(compAngleX) > 90) {
    Serial.println(F("⚠️  WARNING: IMU angle abnormal!"));
    Serial.println(F("⚠️  請檢查 MPU6050 接線或 I2C 位址"));
    compAngleX = 0;
  }

  Serial.println(F("================\n"));

  lastI2CSuccess = millis();
  lastLoopTime = millis();
  lastIMUTime = micros();
  
  wdt_enable(WDTO_2S);  // 啟動硬體看門狗，2秒超時
}

// ============================================================================
// Loop 主迴圈
// ============================================================================

void loop() {
  wdt_reset();  // 餵狗
  uint32_t currentMillis = millis();

  // ========== 看門狗 ==========
  if(currentMillis - lastLoopTime > WATCHDOG_TIMEOUT) {
    Serial.println(F("!!! WATCHDOG RESET !!!"));
    
    motionState = IDLE;
    currentSystemState = RUNNING;
    currentPriority = PRIORITY_AUTO;
    targetPWM = 0;
    currentPWM = 0;
    analogWrite(PWM_CONTROL_PIN, 0);
    digitalWrite(RELAY_PIN, LOW);
    tiltWarning = false;
    manualMode = false;
    speedLowCount = 0;
    speedHighCount = 0;
    tiltStartTime = 0;
    safeStartTime = 0;
    limitUpLocked = false;
    limitDownLocked = false;
    
    beepStart(NOTE_A5, 300);
  }
  lastLoopTime = currentMillis;

  // ========== I2C 健康檢查與恢復 ==========
  if(!i2cHealthy && (millis() - lastI2CSuccess > I2C_TIMEOUT)) {
    Serial.println(F("!!! I2C TIMEOUT - RESET !!!"));
    
    Wire.end();
    delay(100);
    Wire.begin();
    Wire.setClock(100000);
    setupMPU();
    
    compAngleX = 0;
    i2cHealthy = true;
    lastI2CSuccess = millis();
    
    beepStart(NOTE_C5, 200);
  }

  // ========== 非阻塞音效更新 ==========
  beepUpdate();

  // ========== 讀取感測器 ==========
  readIMU();

  // 安全讀取電流
  if(IGNORE_CURRENT_SENSOR) {
    current_A = 0.0;
  } else {
    float tempCurrent = ina226.getCurrent_A();
    if(isnan(tempCurrent) || tempCurrent < -1.0 || tempCurrent > 10.0) {
      current_A = 0.0;
    } else {
      current_A = tempCurrent;
    }
  }

  // 速度更新
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
    speedLowCount = (speedLowCount > 0) ? speedLowCount - 1 : 0;
    speedHighCount = (speedHighCount > 0) ? speedHighCount - 1 : 0;
  }

  // ========== 紅外線接收 ==========
  if(IrReceiver.decode()) {
    handleIRCommand(IrReceiver.decodedIRData.decodedRawData);
    IrReceiver.resume();
  }

  // ========== 緊急按鈕檢查 ==========
  if(BUTTON_IS_NC_TYPE) {
    emergencyStopPressed = (digitalRead(EMERGENCY_STOP_PIN) == HIGH);
  } else {
    emergencyStopPressed = (digitalRead(EMERGENCY_STOP_PIN) == LOW);
  }

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

  // ========== 優先權決策 ==========
  decidePriority();

  // ========== 根據優先權執行動作 ==========
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

  // ========== OLED 螢幕更新 ==========
  if(millis() - lastDisplayTime > 250) {
    lastDisplayTime = millis();
    drawDashboard();
  }

  // ========== 序列埠輸出 ==========
  if(millis() - lastPrintTime > 1000) {
    lastPrintTime = millis();

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
    Serial.print(currentPWM);
    Serial.print(F("/"));
    Serial.print(targetPWM);

    Serial.print(F(" |I:"));
    Serial.print(current_A, 2);
    Serial.print(F("A"));

    Serial.print(F(" |SpdCnt:L"));
    Serial.print(speedLowCount);
    Serial.print(F("/H"));
    Serial.print(speedHighCount);

    if(tiltWarning) 
      Serial.print(F(" [TILT!]"));
    if(manualMode) 
      Serial.print(F(" [MANUAL]"));
    if(emergencyStopPressed)
      Serial.print(F(" [EMRG-BTN]"));
    if(!i2cHealthy)
      Serial.print(F(" [I2C-ERR]"));

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

  delay(2);  // 給予系統緩衝
}
