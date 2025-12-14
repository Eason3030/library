// ============================================================================
// 自行車輔助輪智能控制系統 v2 - 邏輯修正版
// 基於 ok_2.ino，整合電流監測，修正狀態機邏輯
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
// OLED 顯示器設定
// ============================================================================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ============================================================================
// MPU6050 陀螺儀/加速度計設定
// ============================================================================
#define MPU_6050 0x68
#define PWR_MGMT_1 0x6B
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H  0x43

// ============================================================================
// 數位腳位定義
// ============================================================================
const int BUZZER_PIN = 8;
const int RELAY_PIN  = 11;
const int IR_RECEIVE_PIN = 3;
const int HALL_SENSOR_PIN = 4;
const int HALL_SPEED_PIN = 2;
const int PWM_CONTROL_PIN = 9;
const int SPEED_POT_PIN = A0;

// ============================================================================
// RGB LED 腳位定義
// ============================================================================
const int RGB_RED_PIN = 5;
const int RGB_GREEN_PIN = 7;
const int RGB_BLUE_PIN = 6;
const bool RGB_COMMON_ANODE = false;

// ============================================================================
// RGB 顏色定義
// ============================================================================
enum RGBColor {
  RGB_OFF = 0,
  RGB_GREEN = 1,    // 正常運作/UP狀態
  RGB_YELLOW = 2,   // 輔助輪DOWN/低速
  RGB_BLUE = 3,     // 傾斜警告
  RGB_RED = 4,      // 緊急停止/危險
  RGB_PURPLE = 5    // 測試/初始化
};

// ============================================================================
// INA226 電流監測
// ============================================================================
INA226_WE ina226;
const float MAX_CURRENT_A = 5.0;
const float BUZZER_CURRENT_THRESHOLD = 1.5;
float current_A = 0.0;
float busVoltage_V = 0.0;

// ============================================================================
// 速度測量參數
// ============================================================================
const float WHEEL_DIAMETER = 65.0;
const float WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * 3.14159;
const float SPEED_THRESHOLD = 10.0;
const unsigned long SPEED_TIMEOUT = 2000;

volatile unsigned long lastSpeedTrigger = 0;
volatile unsigned long timeBetweenTriggers = 0;
volatile bool newSpeedData = false;
float currentSpeed = 0.0;
unsigned long lastSpeedUpdate = 0;

// ============================================================================
// IMU 濾波器參數
// ============================================================================
const float dt = 0.01f;
const float alpha = 0.98f;
float compAngleX = 0.0f;
float gyroBiasX = 0.0f;
float accelBiasX = 0.0f;

// ============================================================================
// 傾斜安全設定
// ============================================================================
const float SAFE_ANGLE = 20.0f;
const float TILT_SAFE = 12.0f;
const float EMERGENCY_ANGLE = 35.0f;
const unsigned long TILT_TRIGGER_TIME = 500;
const unsigned long AUTO_UP_DELAY = 3000;

unsigned long tiltStartTime = 0;
bool tiltWarning = false;
unsigned long safeStartTime = 0;
bool emergencyStop = false;

// ============================================================================
// 系統狀態定義
// ============================================================================
enum SystemState {
  RUNNING,
  HALTED,
  COOLING_DOWN
};

SystemState currentSystemState = RUNNING;
unsigned long haltTime = 0;
const unsigned long RESTART_DELAY_MS = 5000;

// ============================================================================
// 控制變數（核心）
// ============================================================================
bool wheelDown = false;          // 當前輔助輪狀態（事實）
bool relayCmd = LOW;             // 最終繼電器命令（唯一輸出點）
byte ctrlSrc = 0;                // 控制來源 0=INIT, 1=HALL, 2=IR, 3=AUTO, 4=TILT, 5=EMERG, 6=HALT
bool manualMode = false;         // 手動模式旗標

// ============================================================================
// 顯示與動畫
// ============================================================================
unsigned long wheelAnimStartTime = 0;
bool showWheelAnim = false;
int lastHallState = HIGH;
unsigned long lastPrintTime = 0;
unsigned long lastDisplayTime = 0;
unsigned long lastIRCode = 0;
const unsigned long IR_DELAY = 500;

// ============================================================================
// RGB LED 控制函式
// ============================================================================
void setRGB(int red, int green, int blue) {
  if (RGB_COMMON_ANODE) {
    analogWrite(RGB_RED_PIN, 255 - red);
    analogWrite(RGB_GREEN_PIN, 255 - green);
    analogWrite(RGB_BLUE_PIN, 255 - blue);
  } else {
    analogWrite(RGB_RED_PIN, red);
    analogWrite(RGB_GREEN_PIN, green);
    analogWrite(RGB_BLUE_PIN, blue);
  }
}

void setRGBColor(RGBColor color) {
  switch (color) {
    case RGB_OFF: setRGB(0, 0, 0); break;
    case RGB_GREEN: setRGB(0, 255, 0); break;
    case RGB_YELLOW: setRGB(255, 255, 0); break;
    case RGB_BLUE: setRGB(0, 0, 255); break;
    case RGB_RED: setRGB(255, 0, 0); break;
    case RGB_PURPLE: setRGB(128, 0, 255); break;
  }
}

void rgbBlink(RGBColor color, int times, int delayMs) {
  for (int i = 0; i < times; i++) {
    setRGBColor(color);
    delay(delayMs);
    setRGBColor(RGB_OFF);
    delay(delayMs);
  }
}

// ============================================================================
// 音效函式
// ============================================================================
void beep(int freq, int duration) {
  tone(BUZZER_PIN, freq, duration);
  delay(duration);
  noTone(BUZZER_PIN);
}

void soundStartup() {
  beep(NOTE_C5, 100);
  beep(NOTE_E5, 100);
  beep(NOTE_G5, 150);
  delay(100);
  beep(NOTE_C6, 200);
}

void soundDown() {
  beep(NOTE_G5, 60);
  beep(NOTE_E5, 60);
  beep(NOTE_C5, 100);
}

void soundUp() {
  beep(NOTE_C5, 60);
  beep(NOTE_E5, 60);
  beep(NOTE_G5, 100);
}

void soundClick() {
  beep(NOTE_C6, 50);
}

void soundComplete() {
  beep(NOTE_C5, 80);
  beep(NOTE_E5, 80);
  beep(NOTE_G5, 80);
  beep(NOTE_C6, 150);
}

void soundEmergency() {
  for (int i = 0; i < 3; i++) {
    beep(NOTE_C6, 100);
    delay(50);
    beep(NOTE_A5, 100);
    delay(50);
  }
}

// ============================================================================
// 速度中斷服務函式
// ============================================================================
void speedInterrupt() {
  unsigned long currentTime = micros();
  
  if (currentTime - lastSpeedTrigger > 50000) {
    timeBetweenTriggers = currentTime - lastSpeedTrigger;
    lastSpeedTrigger = currentTime;
    newSpeedData = true;
  }
}

// ============================================================================
// 速度計算
// ============================================================================
float calculateSpeed() {
  if (timeBetweenTriggers == 0) return 0.0;
  
  float distancePerTrigger = WHEEL_CIRCUMFERENCE;
  float timeInSeconds = timeBetweenTriggers / 1000000.0;
  float speedCmPerSec = distancePerTrigger / timeInSeconds;
  float speedKmPerHour = speedCmPerSec * 0.036;
  
  return speedKmPerHour;
}

// ============================================================================
// MPU6050 函式
// ============================================================================
void mpuReadBytes(uint8_t reg, uint8_t count, uint8_t* dest) {
  Wire.beginTransmission(MPU_6050);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_6050, count);
  for (uint8_t i = 0; i < count; i++) {
    dest[i] = Wire.read();
  }
}

int16_t read16(uint8_t reg) {
  uint8_t buf[2];
  mpuReadBytes(reg, 2, buf);
  return (int16_t)((buf[0] << 8) | buf[1]);
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
  gyroBiasX  = sumGx / 50;
}

void readIMU(float &ax, float &gx) {
  ax = read16(ACCEL_XOUT_H) / 16384.0f;
  float az = read16(ACCEL_XOUT_H + 4) / 16384.0f;
  gx = read16(GYRO_XOUT_H) / 131.0f;

  gx -= gyroBiasX;
  ax -= accelBiasX;

  float accelAngle = atan2(ax, az) * 57.2958f;
  compAngleX = alpha * (compAngleX + gx * dt) + (1 - alpha) * accelAngle;
}

// ============================================================================
// 啟動動畫
// ============================================================================
void startupRGB() {
  RGBColor colors[] = {RGB_RED, RGB_YELLOW, RGB_GREEN, RGB_BLUE, RGB_PURPLE};
  
  for (int i = 0; i < 5; i++) {
    setRGBColor(colors[i]);
    beep(400 + i * 100, 60);
    delay(200);
  }
  
  for (int i = 0; i < 3; i++) {
    setRGBColor(RGB_PURPLE);
    beep(NOTE_C6, 80);
    delay(100);
    setRGBColor(RGB_OFF);
    delay(100);
  }
}

void showStartup() {
  display.clearDisplay();

  for (int i = 0; i < 2; i++) {
    display.fillRect(0, 0, 128, 32, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
    display.setTextSize(3);
    display.setCursor(14, 6);
    display.print("BIKE");
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
  display.print("LOADING");

  for (int i = 0; i <= 100; i += 10) {
    display.fillRect(14, 20, i, 4, SSD1306_WHITE);
    display.display();
    if (i % 30 == 0) beep(500 + i * 3, 40);
    delay(50);
  }
  delay(200);
}

// ============================================================================
// OLED 主畫面顯示
// ============================================================================
void showStatus() {
  display.clearDisplay();

  display.setTextSize(1);
  display.drawRect(0, 0, 128, 10, SSD1306_WHITE);
  
  display.setCursor(2, 2);
  
  // 狀態顯示
  if (emergencyStop) {
    display.print("EMERG");
  } else if (currentSystemState == HALTED) {
    display.print("HALT");
  } else if (currentSystemState == COOLING_DOWN) {
    display.print("COOL");
  } else if (ctrlSrc == 1) {
    display.print("HALL");
  } else if (ctrlSrc == 2) {
    display.print("IR");
  } else if (ctrlSrc == 3) {
    display.print("AUTO");
  } else if (ctrlSrc == 4) {
    display.print("TILT");
  } else {
    display.print("RUN");
  }
  
  // 速度顯示
  display.print(" ");
  display.print((int)currentSpeed);
  display.print("k");
  
  // 角度顯示
  display.print(" ");
  int angle = (int)compAngleX;
  if (angle >= 0 && angle < 10) display.print(" ");
  display.print(angle);
  display.print("d");

  // 警告標示
  if (emergencyStop || current_A > BUZZER_CURRENT_THRESHOLD) {
    display.setCursor(118, 2);
    display.print("!!");
  } else if (tiltWarning) {
    display.setCursor(122, 2);
    display.print("!");
  }

  display.setTextSize(2);

  // 輔助輪狀態顯示
  if (showWheelAnim && (millis() - wheelAnimStartTime < 600)) {
    byte frame = ((millis() - wheelAnimStartTime) / 150) % 2;

    if (wheelDown) {
      display.setCursor(10, 14);
      display.print(frame == 0 ? "v DOWN" : "V DOWN");
      display.fillTriangle(110, 18, 114, 24, 118, 18, SSD1306_WHITE);
    } else {
      display.setCursor(10, 14);
      display.print(frame == 0 ? "^ UP" : "^ UP");
      display.fillTriangle(110, 24, 114, 18, 118, 24, SSD1306_WHITE);
    }

  } else {
    showWheelAnim = false;

    if (wheelDown) {
      display.setCursor(10, 14);
      display.print("V DOWN");
      display.fillTriangle(110, 18, 114, 24, 118, 18, SSD1306_WHITE);
      display.fillTriangle(110, 24, 114, 28, 118, 24, SSD1306_WHITE);
    } else {
      display.setCursor(22, 14);
      display.print("^ UP");
      display.fillTriangle(110, 28, 114, 22, 118, 28, SSD1306_WHITE);
      display.fillTriangle(110, 22, 114, 18, 118, 22, SSD1306_WHITE);
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
  pinMode(HALL_SENSOR_PIN, INPUT);
  pinMode(HALL_SPEED_PIN, INPUT_PULLUP);
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

  // OLED 初始化
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED Fail");
    setRGBColor(RGB_RED);
    beep(NOTE_C5, 500);
    while (1);
  }

  // INA226 初始化
  ina226.init();
  if (ina226.getBusVoltage_V() == 0.0 && ina226.getCurrent_A() == 0.0) {
    Serial.println("INA226 init failed");
  } else {
    Serial.println("INA226 OK");
  }

  // MPU6050 初始化
  setupMPU();
  calibrateIMU();

  // 啟動動畫
  startupRGB();
  showStartup();
  soundStartup();

  Serial.println("=== SYSTEM READY ===");
  setRGBColor(RGB_GREEN);
}

// ============================================================================
// Loop - 三層決策邏輯（核心重點）
// ============================================================================
void loop() {
  unsigned long currentMillis = millis();
  
  // ========================================================================
  // 【階段 1】讀取所有感測器數據
  // ========================================================================
  float ax, gx;
  readIMU(ax, gx);
  
  busVoltage_V = ina226.getBusVoltage_V();
  current_A = ina226.getCurrent_A();

  // 過電流蜂鳴器（獨立於主邏輯）
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
  }
  if (millis() - lastSpeedUpdate > SPEED_TIMEOUT) {
    currentSpeed = 0.0;
  }

  // 紅外線接收
  if (IrReceiver.decode()) {
    uint32_t code = IrReceiver.decodedIRData.decodedRawData;
    Serial.print("IR: 0x");
    Serial.println(code, HEX);
    handleIRCommand(code);
    IrReceiver.resume();
  }

  // 霍爾感測器變化檢測
  int hall = digitalRead(HALL_SENSOR_PIN);
  if (hall != lastHallState) {
    lastHallState = hall;
    wheelDown = (hall == LOW);
    ctrlSrc = 1;
    manualMode = true;

    if (wheelDown) soundDown();
    else soundUp();

    showWheelAnim = true;
    wheelAnimStartTime = millis();
    Serial.println("HALL control");
  }

  // ========================================================================
  // 【階段 2】三層決策邏輯（核心）
  // ========================================================================
  
  // 預設值
  relayCmd = LOW;
  bool allowLowerPriority = true;  // 是否允許低優先級邏輯執行
  RGBColor targetRGB = RGB_GREEN;  // 預設 RGB

  // ------------------------------------------------------------------------
  // 🔴【第一層】安全層（最高優先）- 不能被覆蓋
  // ------------------------------------------------------------------------
  
  // 檢查急停條件
  bool isOverloaded = (current_A > MAX_CURRENT_A);
  bool haltConditionActive = 
    (digitalRead(HALL_SENSOR_PIN) == LOW) ||  // 注意：這個是急停霍爾，不是手動控制
    isOverloaded;

  // 狀態機邏輯
  switch (currentSystemState) {
    case RUNNING:
      if (haltConditionActive) {
        currentSystemState = HALTED;
        haltTime = currentMillis;
        Serial.println("!!! HALTED !!!");
      }
      break;

    case HALTED:
      relayCmd = HIGH;  // 強制 DOWN
      wheelDown = true;
      targetRGB = RGB_RED;
      ctrlSrc = 6;
      manualMode = false;  // 清除手動模式
      allowLowerPriority = false;  // 鎖死，不允許其他邏輯
      
      if (!haltConditionActive) {
        currentSystemState = COOLING_DOWN;
        haltTime = currentMillis;
        Serial.println(">>> COOLING_DOWN <<<");
      }
      break;

    case COOLING_DOWN:
      relayCmd = HIGH;  // 冷卻期間保持 DOWN
      wheelDown = true;
      targetRGB = RGB_RED;
      ctrlSrc = 6;
      allowLowerPriority = false;
      
      if (haltConditionActive) {
        currentSystemState = HALTED;
        Serial.println("!!! Back to HALTED !!!");
      } else if (currentMillis - haltTime >= RESTART_DELAY_MS) {
        currentSystemState = RUNNING;
        manualMode = false;  // 清除手動模式
        Serial.println("<<< RUNNING >>>");
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
      Serial.println("EMERGENCY STOP!");
    }
    relayCmd = HIGH;
    wheelDown = true;
    targetRGB = RGB_RED;
    ctrlSrc = 5;
    manualMode = false;  // 清除手動模式
    allowLowerPriority = false;
  } else if (emergencyStop && abs(compAngleX) < TILT_SAFE) {
    emergencyStop = false;
    manualMode = false;  // 清除手動模式
    Serial.println("Emergency cleared");
  }

  // ------------------------------------------------------------------------
  // 🟡【第二層】傾斜警告（第二優先）
  // ------------------------------------------------------------------------
  if (allowLowerPriority) {
    if (abs(compAngleX) > SAFE_ANGLE) {
      if (tiltStartTime == 0) tiltStartTime = millis();
      
      if (!tiltWarning && (millis() - tiltStartTime >= TILT_TRIGGER_TIME)) {
        tiltWarning = true;
        wheelDown = true;
        ctrlSrc = 4;
        manualMode = false;  // 清除手動模式
        beep(NOTE_G5, 150);
        delay(40);
        soundDown();
        showWheelAnim = true;
        wheelAnimStartTime = millis();
        safeStartTime = 0;
        Serial.println("TILT WARNING");
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
            manualMode = false;  // 清除手動模式
            showWheelAnim = true;
            wheelAnimStartTime = millis();
            soundUp();
            Serial.println("TILT recovered");
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

  // ------------------------------------------------------------------------
  // 🟢【第三層】操作層（最低優先）
  // ------------------------------------------------------------------------
  if (allowLowerPriority) {
    // 手動模式（霍爾 / IR）
    if (manualMode) {
      relayCmd = wheelDown ? HIGH : LOW;
      targetRGB = wheelDown ? RGB_YELLOW : RGB_GREEN;
    }
    // 自動模式
    else {
      bool shouldWheelDown = (currentSpeed < SPEED_THRESHOLD);
      
      if (shouldWheelDown != wheelDown) {
        wheelDown = shouldWheelDown;
        ctrlSrc = 3;
        
        if (wheelDown) soundDown();
        else soundUp();
        
        showWheelAnim = true;
        wheelAnimStartTime = millis();
        
        Serial.print("AUTO: Speed ");
        Serial.print(currentSpeed, 1);
        Serial.print(" -> ");
        Serial.println(wheelDown ? "DOWN" : "UP");
      }
      
      relayCmd = wheelDown ? HIGH : LOW;
      targetRGB = wheelDown ? RGB_YELLOW : RGB_GREEN;
    }
  }

  // ========================================================================
  // 【階段 3】最終輸出（唯一控制點）
  // ========================================================================
  
  digitalWrite(RELAY_PIN, relayCmd);
  
  // PWM 控制（只在 DOWN 時有效）
  if (relayCmd == HIGH) {
    int potValue = analogRead(SPEED_POT_PIN);
    int pwmSpeed = map(potValue, 0, 1023, 0, 255);
    analogWrite(PWM_CONTROL_PIN, pwmSpeed);
  } else {
    analogWrite(PWM_CONTROL_PIN, 0);
  }
  
  // RGB LED 輸出
  setRGBColor(targetRGB);

  // ========================================================================
  // 【階段 4】顯示更新
  // ========================================================================
  
  if (millis() - lastDisplayTime > 50) {
    showStatus();
    lastDisplayTime = millis();
  }

  // 序列埠輸出
  if (millis() - lastPrintTime > 500) {
    Serial.print("State:");
    switch (currentSystemState) {
      case RUNNING: Serial.print("RUN"); break;
      case HALTED: Serial.print("HALT"); break;
      case COOLING_DOWN: Serial.print("COOL"); break;
    }
    Serial.print(" | Speed:");
    Serial.print(currentSpeed, 1);
    Serial.print("km/h");
    Serial.print(" | Angle:");
    Serial.print(compAngleX, 1);
    Serial.print("deg");
    Serial.print(" | Curr:");
    Serial.print(current_A, 2);
    Serial.print("A");
    Serial.print(" | Relay:");
    Serial.print(relayCmd ? "ON" : "OFF");
    Serial.print(" | Wheel:");
    Serial.print(wheelDown ? "DN" : "UP");
    
    if (relayCmd == HIGH) {
      int potValue = analogRead(SPEED_POT_PIN);
      int pwmSpeed = map(potValue, 0, 1023, 0, 255);
      Serial.print(" | PWM:");
      Serial.print(pwmSpeed);
    }
    
    Serial.print(" | Ctrl:");
    switch(ctrlSrc) {
      case 0: Serial.print("INIT"); break;
      case 1: Serial.print("HALL"); break;
      case 2: Serial.print("IR"); break;
      case 3: Serial.print("AUTO"); break;
      case 4: Serial.print("TILT"); break;
      case 5: Serial.print("EMERG"); break;
      case 6: Serial.print("HALT"); break;
    }
    
   Serial.println();
    lastPrintTime = millis();
  }

  delay(10);
}
