// ============================================================================
// 輔助輪 v9.7 - 中文顯示版
// 改進項目：
// 1. ✓ 改用 U8g2 庫支援中文顯示
// 2. ✓ 混合版面：中文狀態描述 + 英文數據
// 3. ✓ 優化版面配置適應中文字型
// 4. ✓ 保留所有原有功能
// ============================================================================
#include <Wire.h>
#include <U8g2lib.h>
#include <IRremote.h>
#include <avr/wdt.h>

// ============================================================================
// OLED 設定
// ============================================================================
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// 音符定義
#define NOTE_C4  262
#define NOTE_E4  330
#define NOTE_G4  392
#define NOTE_C5  523
#define NOTE_A5  880
#define NOTE_C6  1047

// MPU6050 設定
#define MPU_ADDR 0x69
#define PWR_MGMT_1 0x6B
#define ACCEL_XOUT_H 0x3B

// 腳位定義
namespace Pins {
  const byte BUZZER = 8;
  const byte IR_RECEIVE = 3;
  const byte PWM = 9;
  const byte LIMIT_UP = 52;
  const byte LIMIT_DOWN = 53;
  const byte HALL_SPEED = 2;
  const byte SPEED_CONTROL = A0;
  const byte RGB_RED = 5;
  const byte RGB_GREEN = 7;
  const byte RGB_BLUE = 6;
  const byte BUTTON = 12;
}

// RGB 顏色常數
namespace RGB {
  const byte OFF = 0;
  const byte FULL = 255;
  const byte MEDIUM = 200;
}

// PWM 控制常數
namespace PWMControl {
  const int HOMING_SPEED = 180;
  const int NORMAL_SPEED = 190;
  const int INCREMENT_STEP = 10;
  const int DECREMENT_STEP = 20;
  const int STOPPED = 0;
}

// 時間常數
namespace Timing {
  const unsigned long HOMING_TIMEOUT_MS = 8000;
  const unsigned long SYSTEM_LOCK_DURATION_MS = 500;
  const unsigned long LIMIT_DEBOUNCE_MS = 50;
  const unsigned long IR_DELAY_MS = 500;
  const unsigned long BUTTON_DEBOUNCE_MS = 200;
  const unsigned long SPEED_TIMEOUT_MS = 2000;
  const unsigned long IMU_READ_INTERVAL_MS = 100;
  const unsigned long DISPLAY_UPDATE_INTERVAL_MS = 100;
  const unsigned long STATUS_PRINT_INTERVAL_MS = 1000;
  const unsigned long RGB_BLINK_INTERVAL_MS = 300;
  const unsigned long HOMING_BLINK_INTERVAL_MS = 500;
  const unsigned long AUTO_LIFT_SPEED_HOLD_MS = 1000;
}

// 速度相關常數
namespace Speed {
  const float WHEEL_CIRCUMFERENCE_MM = 204.2;
  const float AUTO_LIFT_THRESHOLD_KMH = 15.0;
  const float MAX_REASONABLE_SPEED_KMH = 50.0;
  const uint32_t MIN_PULSE_INTERVAL_US = 40000;
}

// IMU 相關常數
namespace IMU {
  const float ACCEL_SCALE = 16384.0;
  const float RAD_TO_DEG_CONV = 57.2958;  // 避免與 Arduino.h 的 RAD_TO_DEG 衝突
  const float TILT_WARNING_DEGREES = 10.0;
  const float TILT_DANGER_DEGREES = 15.0;
  const int MAX_READ_FAILURES = 5;
}

// I2C 錯誤碼說明
namespace I2C {
  const byte SUCCESS = 0;
  const byte DATA_TOO_LONG = 1;
  const byte NACK_ON_ADDRESS = 2;
  const byte NACK_ON_DATA = 3;
  const byte OTHER_ERROR = 4;
  const byte TIMEOUT = 5;
}

// ============================================================================
// 系統狀態定義
// ============================================================================
enum SystemState : byte {
  HOMING,
  AT_TOP,
  AT_BOTTOM,
  MOVING_UP,
  MOVING_DOWN
};

enum HomingPhase : byte {
  NOT_STARTED,
  MOVING,
  COMPLETED,
  FAILED
};

enum HomingFailureReason : byte {
  HOMING_UNKNOWN = 0,          // 避免與 IRremote 的 UNKNOWN 衝突
  HOMING_TIMEOUT = 1,
  HOMING_LIMIT_SWITCH_ERROR = 2
};

// ============================================================================
// 資料結構
// ============================================================================

struct HomingState {
  HomingPhase phase = NOT_STARTED;
  unsigned long startTime = 0;
  bool enabled = true;
  HomingFailureReason failureReason = HOMING_UNKNOWN;

  void reset() {
    phase = NOT_STARTED;
    startTime = 0;
    enabled = true;
    failureReason = HOMING_UNKNOWN;
  }
};

struct MotorState {
  int targetPWM = PWMControl::STOPPED;
  int currentPWM = PWMControl::STOPPED;
  int normalSpeed = PWMControl::NORMAL_SPEED;
  bool manualOverride = false;

  void setTarget(int pwm) {
    targetPWM = constrain(pwm, PWMControl::STOPPED, 255);
  }

  void smoothUpdate() {
    if (currentPWM < targetPWM) {
      currentPWM = min(currentPWM + PWMControl::INCREMENT_STEP, targetPWM);
    } else if (currentPWM > targetPWM) {
      currentPWM = max(currentPWM - PWMControl::DECREMENT_STEP, targetPWM);
    }
  }
};

struct LimitSwitchState {
  bool upperTriggered = false;
  bool lowerTriggered = false;
  unsigned long lastCheckTime = 0;

  bool needsDebounce() {
    return (millis() - lastCheckTime) < Timing::LIMIT_DEBOUNCE_MS;
  }

  void updateCheckTime() {
    lastCheckTime = millis();
  }
};

struct SystemLockState {
  bool locked = false;
  unsigned long lockStartTime = 0;

  void lock() {
    locked = true;
    lockStartTime = millis();
  }

  void unlock() {
    locked = false;
  }

  bool shouldUnlock() {
    return locked && (millis() - lockStartTime > Timing::SYSTEM_LOCK_DURATION_MS);
  }
};

struct SpeedMeasurement {
  volatile uint32_t lastTriggerTime = 0;
  volatile uint32_t pulseInterval = 0;
  volatile bool hasNewData = false;
  float currentSpeed = 0.0;
  unsigned long lastUpdateTime = 0;

  bool isTimedOut() {
    return (millis() - lastUpdateTime) > Timing::SPEED_TIMEOUT_MS;
  }

  void markUpdated() {
    lastUpdateTime = millis();
  }
};

struct AutoLiftState {
  unsigned long speedAboveThresholdTime = 0;
  bool timerRunning = false;
  bool liftCompleted = false;

  void reset() {
    timerRunning = false;
    speedAboveThresholdTime = 0;
  }

  unsigned long getElapsedTime() {
    return timerRunning ? (millis() - speedAboveThresholdTime) : 0;
  }
};

struct IMUState {
  float tiltAngle = 0.0;
  unsigned long lastReadTime = 0;
  bool lastWarningState = false;
  int consecutiveReadFailures = 0;

  bool needsRead() {
    return (millis() - lastReadTime) >= Timing::IMU_READ_INTERVAL_MS;
  }

  void markRead() {
    lastReadTime = millis();
  }

  void recordFailure() {
    consecutiveReadFailures++;
  }

  void recordSuccess() {
    consecutiveReadFailures = 0;
  }

  bool hasExcessiveFailures() {
    return consecutiveReadFailures >= IMU::MAX_READ_FAILURES;
  }
};

struct DisplayState {
  byte currentPage = 0;
  unsigned long lastUpdateTime = 0;

  bool needsUpdate() {
    return (millis() - lastUpdateTime) >= Timing::DISPLAY_UPDATE_INTERVAL_MS;
  }

  void markUpdated() {
    lastUpdateTime = millis();
  }

  void nextPage() {
    currentPage = (currentPage + 1) % 3;
  }
};

// ============================================================================
// 全域物件
// ============================================================================
SystemState g_systemState = HOMING;
bool g_testModeEnabled = false;

HomingState g_homing;
MotorState g_motor;
LimitSwitchState g_limits;
SystemLockState g_systemLock;
SpeedMeasurement g_speed;
AutoLiftState g_autoLift;
IMUState g_imu;
DisplayState g_display;

struct {
  uint32_t irCode = 0;
  bool irDataAvailable = false;
  unsigned long lastIRTime = 0;
  bool lastButtonState = HIGH;
  unsigned long lastButtonPress = 0;
} g_input;

unsigned long g_lastStatusPrint = 0;

// ============================================================================
// 輔助函數：I2C 錯誤訊息
// ============================================================================
void printI2CError(byte errorCode) {
  Serial.print(F("I2C 錯誤碼 "));
  Serial.print(errorCode);
  Serial.print(F(": "));

  switch(errorCode) {
    case I2C::SUCCESS:
      Serial.println(F("成功"));
      break;
    case I2C::DATA_TOO_LONG:
      Serial.println(F("資料過長"));
      break;
    case I2C::NACK_ON_ADDRESS:
      Serial.println(F("位址無回應 (裝置未連接?)"));
      break;
    case I2C::NACK_ON_DATA:
      Serial.println(F("資料傳輸失敗"));
      break;
    case I2C::OTHER_ERROR:
      Serial.println(F("其他錯誤"));
      break;
    case I2C::TIMEOUT:
      Serial.println(F("逾時"));
      break;
    default:
      Serial.println(F("未知錯誤"));
      break;
  }
}

// ============================================================================
// 速度中斷
// ============================================================================
void speedInterrupt() {
  uint32_t now = micros();
  uint32_t interval = now - g_speed.lastTriggerTime;
  if (interval > Speed::MIN_PULSE_INTERVAL_US) {
    g_speed.pulseInterval = interval;
    g_speed.lastTriggerTime = now;
    g_speed.hasNewData = true;
  }
}

// ============================================================================
// RGB LED 控制
// ============================================================================
void setRGB(byte red, byte green, byte blue) {
  analogWrite(Pins::RGB_RED, red);
  analogWrite(Pins::RGB_GREEN, green);
  analogWrite(Pins::RGB_BLUE, blue);
}

struct RGBColor {
  byte r, g, b;
};

const RGBColor RGB_COLORS[] = {
  {RGB::OFF, RGB::FULL, RGB::FULL},
  {RGB::FULL, RGB::MEDIUM, RGB::OFF},
  {RGB::OFF, RGB::FULL, RGB::OFF},
  {RGB::FULL, RGB::OFF, RGB::OFF}
};

void updateRGBByState() {
  if (g_motor.manualOverride) {
    static unsigned long lastBlink = 0;
    static bool blinkState = false;
    if (millis() - lastBlink > Timing::RGB_BLINK_INTERVAL_MS) {
      lastBlink = millis();
      blinkState = !blinkState;
      setRGB(blinkState ? RGB::FULL : RGB::OFF, RGB::OFF, RGB::OFF);
    }
    return;
  }

  if (g_systemState == HOMING) {
    static unsigned long lastBlink = 0;
    static bool blinkState = false;
    if (millis() - lastBlink > Timing::HOMING_BLINK_INTERVAL_MS) {
      lastBlink = millis();
      blinkState = !blinkState;
      setRGB(blinkState ? RGB::FULL : RGB::OFF,
             RGB::OFF,
             blinkState ? RGB::FULL : RGB::OFF);
    }
    return;
  }

  if (g_systemState >= AT_TOP && g_systemState <= MOVING_DOWN) {
    int index = g_systemState - AT_TOP;
    const RGBColor& color = RGB_COLORS[index];
    setRGB(color.r, color.g, color.b);
  }
}

// ============================================================================
// 音效函數
// ============================================================================
void playTone(uint16_t frequency, uint16_t durationMs) {
  tone(Pins::BUZZER, frequency, durationMs);
  delay(durationMs + 20);
  noTone(Pins::BUZZER);
}

void playStartupMusic() {
  playTone(NOTE_C4, 150);
  playTone(NOTE_E4, 150);
  playTone(NOTE_G4, 150);
  playTone(NOTE_C5, 300);
}

void playWarning() {
  for(int i = 0; i < 3; i++) {
    tone(Pins::BUZZER, NOTE_C6, 100);
    delay(150);
  }
  noTone(Pins::BUZZER);
}

void playHomingComplete() {
  playTone(NOTE_G4, 150);
  playTone(NOTE_C5, 200);
}

// ============================================================================
// IMU 函數
// ============================================================================
bool initializeIMU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0x00);
  byte error = Wire.endTransmission();

  if (error == I2C::SUCCESS) {
    Serial.println(F("IMU 初始化成功"));
    g_imu.recordSuccess();
    return true;
  } else {
    Serial.print(F("IMU 初始化失敗 - "));
    printI2CError(error);
    g_imu.recordFailure();
    return false;
  }
}

bool readIMUData(int16_t& accelX, int16_t& accelY, int16_t& accelZ) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_XOUT_H);
  byte error = Wire.endTransmission(false);

  if (error != I2C::SUCCESS) {
    Serial.print(F("[IMU] 讀取失敗 - "));
    printI2CError(error);
    return false;
  }

  Wire.requestFrom((byte)MPU_ADDR, (byte)6);
  if (Wire.available() < 6) {
    Serial.println(F("[IMU] 資料不足"));
    return false;
  }

  accelX = (Wire.read() << 8) | Wire.read();
  accelY = (Wire.read() << 8) | Wire.read();
  accelZ = (Wire.read() << 8) | Wire.read();

  return true;
}

float calculateTiltAngle(int16_t accelX, int16_t accelZ) {
  float ax = accelX / IMU::ACCEL_SCALE;
  float az = accelZ / IMU::ACCEL_SCALE;
  float angle = atan2(ax, az) * IMU::RAD_TO_DEG_CONV;
  return constrain(angle, -90.0, 90.0);
}

void updateIMUReading() {
  if (!g_imu.needsRead()) return;
  g_imu.markRead();

  int16_t accelX, accelY, accelZ;

  if (readIMUData(accelX, accelY, accelZ)) {
    g_imu.tiltAngle = calculateTiltAngle(accelX, accelZ);
    g_imu.recordSuccess();
  } else {
    g_imu.recordFailure();
    g_imu.tiltAngle = 0.0;

    if (g_imu.hasExcessiveFailures()) {
      Serial.println(F("[錯誤] IMU 連續讀取失敗，嘗試重新初始化"));
      initializeIMU();
    }
  }
}

void checkTiltWarning() {
  if (g_systemState == HOMING) return;

  float absoluteTilt = abs(g_imu.tiltAngle);
  bool currentWarning = (absoluteTilt >= IMU::TILT_WARNING_DEGREES);

  if (currentWarning && !g_imu.lastWarningState) {
    if (absoluteTilt >= IMU::TILT_DANGER_DEGREES) {
      playWarning();
      Serial.print(F("[警告] 危險傾斜: "));
      Serial.println(g_imu.tiltAngle, 1);
      g_motor.setTarget(g_motor.normalSpeed);
      g_systemState = MOVING_DOWN;
    } else {
      playTone(NOTE_A5, 200);
      Serial.print(F("[警告] 傾斜警告: "));
      Serial.println(g_imu.tiltAngle, 1);
    }
  }
  g_imu.lastWarningState = currentWarning;
}

// ============================================================================
// 速度測量
// ============================================================================
float calculateSpeedFromInterval(uint32_t intervalMicros) {
  float timeInSeconds = intervalMicros / 1000000.0;
  return (Speed::WHEEL_CIRCUMFERENCE_MM / timeInSeconds) * 0.036;
}

bool isSpeedReasonable(float speed) {
  return (speed >= 0 && speed <= Speed::MAX_REASONABLE_SPEED_KMH);
}

void updateSpeedMeasurement() {
  if (g_speed.hasNewData) {
    if (g_speed.pulseInterval > 0) {
      float rawSpeed = calculateSpeedFromInterval(g_speed.pulseInterval);

      if (isSpeedReasonable(rawSpeed)) {
        g_speed.currentSpeed = rawSpeed;
      }
    }
    g_speed.hasNewData = false;
    g_speed.markUpdated();
  }

  if (g_speed.isTimedOut()) {
    g_speed.currentSpeed = 0.0;
  }
}

// ============================================================================
// 限位開關處理
// ============================================================================
void updateLimitSwitchesAndSystemState() {
  if (g_testModeEnabled) return;
  if (g_limits.needsDebounce()) return;
  g_limits.updateCheckTime();

  bool upperRaw = (digitalRead(Pins::LIMIT_UP) == LOW);
  bool lowerRaw = (digitalRead(Pins::LIMIT_DOWN) == LOW);

  static bool lastUpperRaw = false;
  static bool lastLowerRaw = false;
  if (upperRaw != lastUpperRaw || lowerRaw != lastLowerRaw) {
    Serial.print(F("[限位] 上:"));
    Serial.print(upperRaw);
    Serial.print(F(" 下:"));
    Serial.println(lowerRaw);
  }
  lastUpperRaw = upperRaw;
  lastLowerRaw = lowerRaw;

  if (upperRaw && lowerRaw) {
    handleBothLimitsSwitchesTriggered();
    return;
  }

  if (upperRaw && !g_limits.upperTriggered) {
    handleUpperLimitTriggered();
  }
  else if (lowerRaw && !g_limits.lowerTriggered) {
    handleLowerLimitTriggered();
  }
  else if (!upperRaw && !lowerRaw) {
    g_limits.upperTriggered = false;
    g_limits.lowerTriggered = false;
  }
}

void handleBothLimitsSwitchesTriggered() {
  g_motor.setTarget(PWMControl::STOPPED);
  g_systemState = AT_TOP;
  g_systemLock.lock();

  if (g_homing.phase == MOVING) {
    g_homing.phase = FAILED;
    g_homing.failureReason = HOMING_LIMIT_SWITCH_ERROR;
  }

  Serial.println(F("[錯誤] 兩個限位同時觸發！"));
  playWarning();
}

void handleUpperLimitTriggered() {
  g_motor.setTarget(PWMControl::STOPPED);

  if (g_systemState == HOMING && g_homing.phase == MOVING) {
    g_homing.phase = COMPLETED;
    g_systemState = AT_TOP;
    playHomingComplete();
    Serial.println(F("[Homing] 完成 - 到達上限 (非預期)"));
  } else {
    g_systemState = AT_TOP;
    Serial.println(F("[限位] 到達上限"));
  }

  g_systemLock.lock();
  g_limits.upperTriggered = true;
  g_limits.lowerTriggered = false;
  g_autoLift.liftCompleted = false;
  g_autoLift.reset();
}

void handleLowerLimitTriggered() {
  g_motor.setTarget(PWMControl::STOPPED);

  if (g_systemState == HOMING && g_homing.phase == MOVING) {
    g_homing.phase = COMPLETED;
    g_systemState = AT_BOTTOM;
    playHomingComplete();
    Serial.println(F("[Homing] 完成 - 到達下限"));
  } else {
    g_systemState = AT_BOTTOM;
    Serial.println(F("[限位] 到達下限"));
  }

  g_systemLock.lock();
  g_limits.upperTriggered = false;
  g_limits.lowerTriggered = true;
}

// ============================================================================
// Homing 流程
// ============================================================================
void startHomingSequence() {
  Serial.println(F("\n[Homing] 開始歸位流程"));
  Serial.println(F("  策略: 向下移動至下限位"));
  Serial.print(F("  PWM: "));
  Serial.println(PWMControl::HOMING_SPEED);
  Serial.print(F("  逾時: "));
  Serial.print(Timing::HOMING_TIMEOUT_MS / 1000);
  Serial.println(F("秒"));

  bool upperNow = (digitalRead(Pins::LIMIT_UP) == LOW);
  bool lowerNow = (digitalRead(Pins::LIMIT_DOWN) == LOW);

  if (lowerNow && !upperNow) {
    g_homing.phase = COMPLETED;
    g_systemState = AT_BOTTOM;
    playHomingComplete();
    Serial.println(F("[Homing] 已在下限,直接完成"));
    return;
  }

  if (upperNow && lowerNow) {
    g_homing.phase = FAILED;
    g_homing.failureReason = HOMING_LIMIT_SWITCH_ERROR;
    g_systemState = AT_BOTTOM;
    Serial.println(F("[Homing] 失敗 - 限位開關異常"));
    playWarning();
    return;
  }

  g_motor.setTarget(PWMControl::HOMING_SPEED);
  g_homing.phase = MOVING;
  g_homing.startTime = millis();
  Serial.println(F("[Homing] 馬達已啟動"));
}

void updateHomingProgress() {
  unsigned long elapsedTime = millis() - g_homing.startTime;

  static unsigned long lastProgressPrint = 0;
  if (millis() - lastProgressPrint > Timing::STATUS_PRINT_INTERVAL_MS) {
    lastProgressPrint = millis();
    Serial.print(F("[Homing] 進行中... "));
    Serial.print(elapsedTime / 1000);
    Serial.print(F("s / "));
    Serial.print(Timing::HOMING_TIMEOUT_MS / 1000);
    Serial.println(F("s"));
  }
}

void checkHomingTimeout() {
  unsigned long elapsedTime = millis() - g_homing.startTime;

  if (elapsedTime > Timing::HOMING_TIMEOUT_MS) {
    g_motor.setTarget(PWMControl::STOPPED);
    g_homing.phase = FAILED;
    g_homing.failureReason = HOMING_TIMEOUT;
    g_systemState = AT_BOTTOM;
    Serial.println(F("[Homing] 失敗 - 逾時未觸發限位"));
    Serial.println(F("  可能原因: 機構卡住/限位開關故障"));
    playWarning();
  }
}

void performHoming() {
  if (g_homing.phase == COMPLETED || g_homing.phase == FAILED) {
    return;
  }

  if (!g_homing.enabled) {
    g_homing.phase = COMPLETED;
    g_systemState = AT_BOTTOM;
    Serial.println(F("[Homing] 已跳過 (使用者選擇)"));
    return;
  }

  if (g_homing.phase == NOT_STARTED) {
    startHomingSequence();
  }

  if (g_homing.phase == MOVING) {
    updateHomingProgress();
    checkHomingTimeout();
  }
}

// ============================================================================
// 自動升降
// ============================================================================
void checkAutoLiftConditions() {
  if (g_systemState == HOMING) return;
  if (g_homing.phase != COMPLETED) return;
  if (g_systemState != AT_BOTTOM) {
    g_autoLift.reset();
    return;
  }
  if (g_autoLift.liftCompleted || g_motor.manualOverride) {
    return;
  }

  if (g_speed.currentSpeed >= Speed::AUTO_LIFT_THRESHOLD_KMH) {
    if (!g_autoLift.timerRunning) {
      g_autoLift.timerRunning = true;
      g_autoLift.speedAboveThresholdTime = millis();
      Serial.println(F("[自動] 速度達標,開始計時"));
    } else {
      if (g_autoLift.getElapsedTime() >= Timing::AUTO_LIFT_SPEED_HOLD_MS) {
        startAutoLiftAndUpdateState();
      }
    }
  } else {
    if (g_autoLift.timerRunning) {
      Serial.println(F("[自動] 速度降低,重置計時"));
    }
    g_autoLift.reset();
  }
}

void startAutoLiftAndUpdateState() {
  Serial.println(F("[自動] 持續1秒,啟動升降"));

  if (!g_systemLock.locked) {
    g_motor.setTarget(g_motor.normalSpeed);
    g_systemState = MOVING_UP;
    g_autoLift.liftCompleted = true;
    g_autoLift.reset();
    playTone(NOTE_C5, 200);
    Serial.print(F("[馬達] 自動上升 PWM="));
    Serial.println(g_motor.normalSpeed);
  }
}

// ============================================================================
// 輸入處理
// ============================================================================
void readIRReceiver() {
  if (IrReceiver.decode()) {
    g_input.irCode = IrReceiver.decodedIRData.decodedRawData;
    if (g_input.irCode != 0 && g_input.irCode != 0xFFFFFFFF) {
      g_input.irDataAvailable = true;
    }
    IrReceiver.resume();
  }
}

void handleIRCommand() {
  if (!g_input.irDataAvailable) return;
  if (millis() - g_input.lastIRTime < Timing::IR_DELAY_MS) {
    g_input.irDataAvailable = false;
    return;
  }

  g_input.lastIRTime = millis();
  g_input.irDataAvailable = false;

  if (g_systemState == HOMING && g_homing.phase == MOVING) {
    skipHomingSequence();
    return;
  }

  toggleManualOverride();
}

void skipHomingSequence() {
  g_motor.setTarget(PWMControl::STOPPED);
  g_homing.enabled = false;
  g_homing.phase = COMPLETED;
  g_systemState = AT_BOTTOM;
  playTone(NOTE_C4, 200);
  Serial.println(F("[紅外] 使用者跳過 Homing"));
  Serial.println(F("  系統假設當前在下限位置"));
}

void toggleManualOverride() {
  g_motor.manualOverride = !g_motor.manualOverride;

  if (g_motor.manualOverride) {
    g_motor.setTarget(PWMControl::STOPPED);
    playTone(NOTE_C4, 200);
    Serial.println(F("[紅外] PWM 手動關閉 (安全模式)"));
  } else {
    playTone(NOTE_C5, 200);
    Serial.println(F("[紅外] PWM 自動模式恢復"));
  }
}

void checkButtonPress() {
  bool buttonState = digitalRead(Pins::BUTTON);

  if (buttonState == LOW && g_input.lastButtonState == HIGH) {
    if (millis() - g_input.lastButtonPress > Timing::BUTTON_DEBOUNCE_MS) {
      g_input.lastButtonPress = millis();
      g_display.nextPage();
      playTone(NOTE_A5, 50);
      Serial.print(F("[按鈕] 切換至頁面 "));
      Serial.println(g_display.currentPage);
    }
  }
  g_input.lastButtonState = buttonState;
}

// ============================================================================
// 馬達控制核心
// ============================================================================
void executeMotorControl() {
  if (g_systemState == HOMING && g_homing.phase == MOVING) {
    g_motor.setTarget(PWMControl::HOMING_SPEED);
  } else if (g_systemState == HOMING) {
    g_motor.setTarget(PWMControl::STOPPED);
  } else if (g_motor.manualOverride) {
    g_motor.setTarget(PWMControl::STOPPED);
  } else if (g_systemState == AT_TOP || g_systemState == AT_BOTTOM) {
    g_motor.setTarget(PWMControl::STOPPED);
  } else if (g_systemState == MOVING_UP || g_systemState == MOVING_DOWN) {
    g_motor.setTarget(g_motor.normalSpeed);

    if (g_limits.upperTriggered && g_systemState == MOVING_UP) {
      g_motor.setTarget(PWMControl::STOPPED);
      g_systemState = AT_TOP;
    }
    if (g_limits.lowerTriggered && g_systemState == MOVING_DOWN) {
      g_motor.setTarget(PWMControl::STOPPED);
      g_systemState = AT_BOTTOM;
    }
  }

  g_motor.smoothUpdate();
  analogWrite(Pins::PWM, g_motor.currentPWM);

  static int lastOutputPWM = -1;
  if (g_motor.currentPWM != lastOutputPWM) {
    Serial.print(F("[PWM] 輸出: "));
    Serial.print(g_motor.currentPWM);
    Serial.print(F(" (目標: "));
    Serial.print(g_motor.targetPWM);
    Serial.println(F(")"));
    lastOutputPWM = g_motor.currentPWM;
  }
}

// ============================================================================
// OLED 顯示 - 中文版
// ============================================================================
void drawMainPage() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_unifont_t_chinese1);

  // 標題列
  u8g2.setCursor(0, 10);
  u8g2.print("腳踏車 v9.7");

  if (g_motor.manualOverride) {
    u8g2.setCursor(90, 10);
    u8g2.setFont(u8g2_font_6x10_tr);
    u8g2.print("[SAFE]");
    u8g2.setFont(u8g2_font_unifont_t_chinese1);
  }

  u8g2.drawLine(0, 12, 127, 12);

  // Homing 狀態
  if (g_systemState == HOMING) {
    u8g2.setCursor(20, 30);
    u8g2.print("歸位中");

    u8g2.setFont(u8g2_font_6x10_tr);
    u8g2.setCursor(30, 45);

    switch(g_homing.phase) {
      case NOT_STARTED:
        u8g2.print("Waiting...");
        break;
      case MOVING:
        {
          unsigned long elapsed = millis() - g_homing.startTime;
          u8g2.print("Moving ");
          u8g2.print(elapsed / 1000);
          u8g2.print("s");
        }
        break;
      case COMPLETED:
        u8g2.print("Completed!");
        break;
      case FAILED:
        u8g2.print("FAILED!");
        break;
    }

    u8g2.sendBuffer();
    return;
  }

  // 系統狀態
  u8g2.setCursor(0, 28);
  switch(g_systemState) {
    case AT_TOP: u8g2.print("位於上限"); break;
    case AT_BOTTOM: u8g2.print("位於下限"); break;
    case MOVING_UP: u8g2.print("上升中"); break;
    case MOVING_DOWN: u8g2.print("下降中"); break;
    default: break;
  }

  u8g2.drawLine(0, 30, 127, 30);

  // 速度和角度
  u8g2.setFont(u8g2_font_6x10_tr);

  u8g2.setCursor(0, 42);
  u8g2.print("Speed:");
  u8g2.setFont(u8g2_font_10x20_tr);
  u8g2.setCursor(40, 45);
  u8g2.print((int)g_speed.currentSpeed);
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.setCursor(70, 42);
  u8g2.print("km/h");

  u8g2.setCursor(0, 56);
  u8g2.print("Angle:");
  u8g2.setFont(u8g2_font_10x20_tr);
  u8g2.setCursor(40, 59);
  u8g2.print((int)g_imu.tiltAngle);
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.setCursor(70, 56);
  u8g2.print("deg");

  // PWM
  u8g2.setCursor(90, 56);
  u8g2.print("P:");
  u8g2.print(g_motor.currentPWM);

  u8g2.sendBuffer();
}

void drawDebugPage() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_unifont_t_chinese1);

  u8g2.setCursor(0, 10);
  u8g2.print("除錯資訊");
  u8g2.drawLine(0, 12, 127, 12);

  u8g2.setFont(u8g2_font_6x10_tr);

  u8g2.setCursor(0, 24);
  u8g2.print("State: ");
  u8g2.print(g_systemState);

  u8g2.setCursor(0, 36);
  u8g2.print("Homing: ");
  u8g2.print(g_homing.phase);

  u8g2.setCursor(0, 48);
  u8g2.print("PWM: ");
  u8g2.print(g_motor.currentPWM);

  u8g2.setCursor(70, 24);
  u8g2.print("Lock:");
  u8g2.print(g_systemLock.locked ? "Y" : "N");

  u8g2.setCursor(70, 36);
  u8g2.print("IMU:");
  u8g2.print(g_imu.consecutiveReadFailures);

  u8g2.setCursor(70, 48);
  u8g2.print("Man:");
  u8g2.print(g_motor.manualOverride ? "Y" : "N");

  u8g2.sendBuffer();
}

// ============================================================================
// Setup
// ============================================================================
void initializePins() {
  pinMode(Pins::PWM, OUTPUT);
  pinMode(Pins::LIMIT_UP, INPUT_PULLUP);
  pinMode(Pins::LIMIT_DOWN, INPUT_PULLUP);
  pinMode(Pins::HALL_SPEED, INPUT_PULLUP);
  pinMode(Pins::BUTTON, INPUT_PULLUP);
  pinMode(Pins::RGB_RED, OUTPUT);
  pinMode(Pins::RGB_GREEN, OUTPUT);
  pinMode(Pins::RGB_BLUE, OUTPUT);
  pinMode(Pins::BUZZER, OUTPUT);

  analogWrite(Pins::PWM, PWMControl::STOPPED);
  Serial.println(F("[初始化] 所有腳位已設定"));
}

void initializeDisplay() {
  u8g2.begin();
  u8g2.enableUTF8Print();  // 啟用 UTF-8 支援！

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_unifont_t_chinese1);
  u8g2.setCursor(0, 20);
  u8g2.print("腳踏車系統");
  u8g2.setCursor(0, 40);
  u8g2.print("版本 9.7");
  u8g2.sendBuffer();

  Serial.println(F("[初始化] OLED 已啟動 (中文模式)"));
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  Serial.println(F("\n========================================"));
  Serial.println(F("輔助輪系統 v9.7 啟動（中文版）"));
  Serial.println(F("========================================"));

  initializePins();

  Wire.begin();
  Wire.setClock(400000);
  Serial.println(F("[初始化] I2C 已啟動"));

  initializeDisplay();
  initializeIMU();

  IrReceiver.begin(Pins::IR_RECEIVE, ENABLE_LED_FEEDBACK);
  Serial.println(F("[初始化] IR 接收器已啟動"));

  attachInterrupt(digitalPinToInterrupt(Pins::HALL_SPEED), speedInterrupt, FALLING);
  Serial.println(F("[初始化] 霍爾感測器中斷已設定"));

  if (digitalRead(Pins::BUTTON) == LOW) {
    g_testModeEnabled = true;
    Serial.println(F("\n[系統] 測試模式啟動！"));
    playTone(NOTE_C5, 100);
    delay(100);
    playTone(NOTE_C5, 100);
  }

  playStartupMusic();

  setRGB(RGB::FULL, RGB::OFF, RGB::OFF); delay(200);
  setRGB(RGB::OFF, RGB::FULL, RGB::OFF); delay(200);
  setRGB(RGB::OFF, RGB::OFF, RGB::FULL); delay(200);
  setRGB(RGB::OFF, RGB::OFF, RGB::OFF);

  Serial.println(F("[初始化] 完成！\n"));
}

// ============================================================================
// Loop
// ============================================================================
void readAllSensors() {
  readIRReceiver();
  checkButtonPress();
  updateSpeedMeasurement();
  updateIMUReading();
}

void updateSystemStates() {
  if (g_systemLock.shouldUnlock()) {
    g_systemLock.unlock();
    Serial.println(F("[系統] 解除鎖定"));
  }
}

void processDecisionLogic() {
  performHoming();
  updateLimitSwitchesAndSystemState();
  checkAutoLiftConditions();
  handleIRCommand();
  checkTiltWarning();
}

void executeControlActions() {
  executeMotorControl();
}

void updateDisplayAndFeedback() {
  updateRGBByState();

  if (g_display.needsUpdate()) {
    g_display.markUpdated();
    if (g_display.currentPage == 0) {
      drawMainPage();
    } else {
      drawDebugPage();
    }
  }
}

void printStatusIfNeeded() {
  if (millis() - g_lastStatusPrint > Timing::STATUS_PRINT_INTERVAL_MS) {
    g_lastStatusPrint = millis();
    Serial.print(F("[狀態] State:"));
    Serial.print(g_systemState);
    Serial.print(F(" PWM:"));
    Serial.print(g_motor.currentPWM);
    Serial.print(F(" Speed:"));
    Serial.print(g_speed.currentSpeed, 1);
    Serial.print(F(" Tilt:"));
    Serial.print(g_imu.tiltAngle, 1);
    Serial.print(F(" Lock:"));
    Serial.println(g_systemLock.locked ? F("Y") : F("N"));
  }
}

void loop() {
  readAllSensors();
  updateSystemStates();
  processDecisionLogic();
  executeControlActions();
  updateDisplayAndFeedback();
  printStatusIfNeeded();
}
