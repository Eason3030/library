#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <IRremote.h>

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

// ============================================================================
// RGB LED 腳位定義
// ============================================================================
const int RGB_RED_PIN = 5;
const int RGB_GREEN_PIN = 6;
const int RGB_BLUE_PIN = 7;

// 設定 RGB 類型：true = 共陽極（HIGH=滅），false = 共陰極（HIGH=亮）
const bool RGB_COMMON_ANODE = false;

// ============================================================================
// RGB 顏色定義（5種狀態顏色）
// ============================================================================
enum RGBColor {
  RGB_OFF = 0,      // 關閉（執行完畢後）
  RGB_GREEN = 1,    // 綠色 - 正常運作/UP狀態
  RGB_YELLOW = 2,   // 黃色 - 輔助輪DOWN/低速
  RGB_BLUE = 3,     // 藍色 - 傾斜警告
  RGB_RED = 4,      // 紅色 - 緊急停止/危險
  RGB_PURPLE = 5    // 紫色 - 測試/初始化
};

// ============================================================================
// 紅外線處理
// ============================================================================
unsigned long lastIRCode = 0;
const unsigned long IR_DELAY = 500;

// ============================================================================
// 速度測量參數
// ============================================================================
const float WHEEL_DIAMETER = 65.0;
const float WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * 3.14159;
const float SPEED_THRESHOLD = 10.0;
const unsigned long SPEED_TIMEOUT = 2000;

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
const unsigned long TILT_TRIGGER_TIME = 500;
const unsigned long AUTO_UP_DELAY = 3000;
unsigned long tiltStartTime = 0;
bool tiltWarning = false;
unsigned long safeStartTime = 0;

// ============================================================================
// 緊急停止設定
// ============================================================================
bool emergencyStop = false;
const float EMERGENCY_ANGLE = 35.0f;

// ============================================================================
// 系統狀態變數
// ============================================================================
bool wheelDown = false;
unsigned long wheelAnimStartTime = 0;
bool showWheelAnim = false;
byte ctrlSrc = 0;

int lastHallState = HIGH;
unsigned long lastPrintTime = 0;
unsigned long lastDisplayTime = 0;
bool manualMode = false;

// ============================================================================
// 速度測量變數（中斷使用）
// ============================================================================
volatile unsigned long lastSpeedTrigger = 0;
volatile unsigned long timeBetweenTriggers = 0;
volatile bool newSpeedData = false;
float currentSpeed = 0.0;
unsigned long lastSpeedUpdate = 0;

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
    case RGB_OFF:
      setRGB(0, 0, 0);
      break;
    case RGB_GREEN:
      setRGB(0, 255, 0);
      break;
    case RGB_YELLOW:
      setRGB(255, 255, 0);
      break;
    case RGB_BLUE:
      setRGB(0, 0, 255);
      break;
    case RGB_RED:
      setRGB(255, 0, 0);
      break;
    case RGB_PURPLE:
      setRGB(128, 0, 255);
      break;
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
// 中斷服務函式
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
  compAngleX = alpha * (compAngleX + gx * dt)
             + (1 - alpha) * accelAngle;
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

// ============================================================================
// OLED 啟動畫面
// ============================================================================
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
  
  if (emergencyStop) {
    display.print("EMERG");
  } else if (ctrlSrc == 1) {
    display.print("HALL");
  } else if (ctrlSrc == 2) {
    display.print("IR-REM");
  } else if (ctrlSrc == 3) {
    display.print("AUTO");
  } else if (ctrlSrc == 4) {
    display.print("TILT");
  } else {
    display.print("INIT");
  }
  
  display.print(" | ");
  display.print(currentSpeed, 0);
  display.print("km");
  display.print(" | ");
  
  int angle = (int)compAngleX;
  if (angle >= 0) display.print(" ");
  display.print(angle);
  display.print("d");

  if (emergencyStop) {
    display.setCursor(110, 2);
    display.print("!!");
  } else if (tiltWarning) {
    display.setCursor(110, 2);
    display.print("!");
  }

  display.setTextSize(2);

  if (showWheelAnim && (millis() - wheelAnimStartTime < 600)) {
    byte frame = ((millis() - wheelAnimStartTime) / 100) % 3;

    if (wheelDown) {
      display.setCursor(4, 14);
      if (frame == 0) display.print("|");
      else if (frame == 1) display.print("v");
      else display.print("V");

      display.setCursor(20, 14);
      display.print("DOWN");

      int arrowY = 16 + frame * 3;
      display.fillTriangle(108, arrowY, 112, arrowY+4, 116, arrowY, SSD1306_WHITE);
      display.fillTriangle(108, arrowY+5, 112, arrowY+9, 116, arrowY+5, SSD1306_WHITE);

    } else {
      display.setCursor(4, 14);
      if (frame == 0) display.print("|");
      else if (frame == 1) display.print("^");
      else display.print("^");

      display.setCursor(20, 14);
      display.print("UP");

      int arrowY = 24 - frame * 3;
      display.fillTriangle(108, arrowY+9, 112, arrowY+5, 116, arrowY+9, SSD1306_WHITE);
      display.fillTriangle(108, arrowY+4, 112, arrowY, 116, arrowY+4, SSD1306_WHITE);
    }

  } else {
    showWheelAnim = false;

    if (wheelDown) {
      display.setCursor(4, 14);
      display.print("V DOWN");
      display.fillRect(0, 30, 128, 2, SSD1306_WHITE);
      display.fillTriangle(110, 16, 114, 20, 118, 16, SSD1306_WHITE);
      display.fillTriangle(110, 22, 114, 26, 118, 22, SSD1306_WHITE);

    } else {
      display.setCursor(16, 14);
      display.print("^ UP");

      for (int x = 0; x < 128; x += 6) {
        display.drawFastHLine(x, 12, 3, SSD1306_WHITE);
      }

      display.fillTriangle(110, 26, 114, 22, 118, 26, SSD1306_WHITE);
      display.fillTriangle(110, 20, 114, 16, 118, 20, SSD1306_WHITE);
    }
  }

  display.display();
}

// ============================================================================
// 模組測試
// ============================================================================
void displayTestResult(const char* result) {
  display.setTextSize(2);
  display.setCursor(95, 12);
  display.print(result);
  display.display();
  delay(600);
}

bool testIR() {
  unsigned long startWait = millis();
  while (millis() - startWait < 1000) {
    if (IrReceiver.decode()) {
      IrReceiver.resume();
      return true;
    }
  }
  return false;
}

void testMPU() {
  float ax, gx;
  readIMU(ax, gx);
  delay(200);
}

bool testHallSensor() {
  return digitalRead(HALL_SENSOR_PIN) == LOW;
}

bool testSpeedSensor() {
  unsigned long startWait = millis();
  while (millis() - startWait < 1000) {
    if (newSpeedData) {
      newSpeedData = false;
      return true;
    }
  }
  return false;
}

void testModules() {
  display.clearDisplay();
  display.fillRect(0, 0, 128, 32, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setTextSize(2);
  display.setCursor(20, 8);
  display.print("TEST");
  display.display();
  display.setTextColor(SSD1306_WHITE);
  
  setRGBColor(RGB_PURPLE);
  beep(NOTE_C5, 150);
  delay(850);

  const char* testNames[] = {"OLED", "IR", "MPU", "HALL", "SPEED"};
  RGBColor testColors[] = {RGB_GREEN, RGB_BLUE, RGB_YELLOW, RGB_PURPLE, RGB_RED};

  for (int i = 0; i < 5; i++) {
    setRGBColor(testColors[i]);
    
    display.clearDisplay();
    display.setTextSize(1);
    display.drawRect(0, 0, 128, 9, SSD1306_WHITE);
    display.setCursor(2, 1);
    display.print("TEST");

    display.setTextSize(2);
    int textWidth = strlen(testNames[i]) * 12;
    int xPos = (128 - textWidth) / 2;
    display.setCursor(xPos, 12);
    display.print(testNames[i]);
    display.display();

    soundClick();

    bool testResult = true;
    if (i == 0) {
      delay(600);
      displayTestResult("V");
      beep(NOTE_E5, 100);
    } else if (i == 1) {
      testResult = testIR();
      displayTestResult(testResult ? "V" : "X");
      beep(testResult ? NOTE_E5 : NOTE_C5, 100);
    } else if (i == 2) {
      testMPU();
      displayTestResult("V");
      beep(NOTE_E5, 100);
    } else if (i == 3) {
      testResult = testHallSensor();
      displayTestResult(testResult ? "V" : "X");
      beep(testResult ? NOTE_E5 : NOTE_C5, 100);
    } else if (i == 4) {
      testResult = testSpeedSensor();
      displayTestResult(testResult ? "V" : "X");
      beep(testResult ? NOTE_E5 : NOTE_C5, 100);
    }

    delay(200);
  }

  display.clearDisplay();
  display.fillRect(0, 0, 128, 32, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setTextSize(2);
  display.setCursor(30, 8);
  display.print("DONE");
  display.display();
  display.setTextColor(SSD1306_WHITE);

  soundComplete();
  rgbBlink(RGB_GREEN, 3, 150);

  delay(1000);
  setRGBColor(RGB_OFF);
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

    digitalWrite(RELAY_PIN, wheelDown ? HIGH : LOW);
    setRGBColor(wheelDown ? RGB_YELLOW : RGB_GREEN);

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

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(HALL_SENSOR_PIN, INPUT);
  pinMode(HALL_SPEED_PIN, INPUT_PULLUP);

  pinMode(RGB_RED_PIN, OUTPUT);
  pinMode(RGB_GREEN_PIN, OUTPUT);
  pinMode(RGB_BLUE_PIN, OUTPUT);

  digitalWrite(RELAY_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  setRGBColor(RGB_OFF);

  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
  attachInterrupt(digitalPinToInterrupt(HALL_SPEED_PIN), speedInterrupt, FALLING);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED Fail");
    setRGBColor(RGB_RED);
    beep(NOTE_C5, 500);
    while (1);
  }

  setupMPU();
  calibrateIMU();

  startupRGB();
  showStartup();
  soundStartup();
  testModules();

  Serial.println("=== READY ===");
  setRGBColor(RGB_GREEN);
}

// ============================================================================
// Loop - 完整控制邏輯
// ============================================================================
void loop() {
  float ax, gx;
  readIMU(ax, gx);

  // ====== 1. 緊急停止檢測（最高優先級）======
  if (abs(compAngleX) > EMERGENCY_ANGLE) {
    if (!emergencyStop) {
      emergencyStop = true;
      wheelDown = true;
      ctrlSrc = 5;
      manualMode = false;
      digitalWrite(RELAY_PIN, HIGH);
      setRGBColor(RGB_RED);
      soundEmergency();
      showWheelAnim = true;
      wheelAnimStartTime = millis();
      Serial.println("EMERGENCY STOP!");
    }
  } else if (emergencyStop && abs(compAngleX) < TILT_SAFE) {
    emergencyStop = false;
    Serial.println("Emergency cleared");
  }

  // ====== 2. 傾斜偵測（第二優先級）======
  if (!emergencyStop) {
    if (abs(compAngleX) > SAFE_ANGLE) {
      if (tiltStartTime == 0) tiltStartTime = millis();
      
      if (!tiltWarning && (millis() - tiltStartTime >= TILT_TRIGGER_TIME)) {
        tiltWarning = true;
        wheelDown = true;
        ctrlSrc = 4;
        manualMode = false;
        digitalWrite(RELAY_PIN, HIGH);
        setRGBColor(RGB_BLUE);
        showWheelAnim = true;
        wheelAnimStartTime = millis();
        beep(NOTE_G5, 150);
        delay(40);
        soundDown();
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
            digitalWrite(RELAY_PIN, LOW);
            setRGBColor(RGB_GREEN);
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
  }

  // ====== 3. 速度測量更新 ======
  if (newSpeedData) {
    currentSpeed = calculateSpeed();
    newSpeedData = false;
    lastSpeedUpdate = millis();
  }

  if (millis() - lastSpeedUpdate > SPEED_TIMEOUT) {
    currentSpeed = 0.0;
  }

  // ====== 4. 紅外線遙控接收 ======
  if (IrReceiver.decode()) {
    uint32_t code = IrReceiver.decodedIRData.decodedRawData;
    Serial.print("IR: 0x");
    Serial.println(code, HEX);
    handleIRCommand(code);
    IrReceiver.resume();
  }

  // ====== 5. 霍爾感測器手動控制（優先於自動）======
  int hall = digitalRead(HALL_SENSOR_PIN);
  if (hall != lastHallState) {
    lastHallState = hall;
    wheelDown = (hall == LOW);
    ctrlSrc = 1;
    manualMode = true;

    digitalWrite(RELAY_PIN, wheelDown ? HIGH : LOW);
    setRGBColor(wheelDown ? RGB_YELLOW : RGB_GREEN);

    if (wheelDown) soundDown();
    else soundUp();

    showWheelAnim = true;
    wheelAnimStartTime = millis();
    Serial.println("HALL control");
  }

  // ====== 6. 自動模式控制（最低優先級）======
  if (!manualMode && !tiltWarning && !emergencyStop) {
    bool shouldWheelDown = (currentSpeed < SPEED_THRESHOLD);

    if (shouldWheelDown != wheelDown) {
      wheelDown = shouldWheelDown;
      ctrlSrc = 3;
      digitalWrite(RELAY_PIN, wheelDown ? HIGH : LOW);
      setRGBColor(wheelDown ? RGB_YELLOW : RGB_GREEN);

      if (wheelDown) soundDown();
      else soundUp();

      showWheelAnim = true;
      wheelAnimStartTime = millis();

      Serial.print("AUTO: Speed ");
      Serial.print(currentSpeed, 1);
      Serial.print(" -> ");
      Serial.println(wheelDown ? "DOWN" : "UP");
    }
  }

  // ====== 7. OLED 顯示更新 ======
  if (millis() - lastDisplayTime > 50) {
    showStatus();
    lastDisplayTime = millis();
  }

  // ====== 8. 序列埠詳細資訊輸出 ======
  if (millis() - lastPrintTime > 500) {
    Serial.print("Speed:");
    Serial.print(currentSpeed, 1);
    Serial.print(" Angle:");
    Serial.print(compAngleX, 1);
    Serial.print(" Wheel:");
    Serial.print(wheelDown ? "DN" : "UP");
    Serial.print(" RGB:");
    if (emergencyStop) Serial.print("RED");
    else if (tiltWarning) Serial.print("BLUE");
    else if (wheelDown) Serial.print("YLW");
    else Serial.print("GRN");
    Serial.println();
    lastPrintTime = millis();
  }

  delay(10);
}

/*
 * ============================================================================
 * RGB LED 狀態邏輯總結
 * ============================================================================
 * 
 * 【顏色變化規則】
 * 
 * 1. 緊急停止 (角度 > 35°)
 *    → setRGBColor(RGB_RED)
 *    → 鎖定紅色，直到角度 < 12° 才解除
 * 
 * 2. 傾斜警告 (角度 > 20°)
 *    → setRGBColor(RGB_BLUE)
 *    → 自動放下輔助輪
 *    → 角度 < 12° 持續 3 秒後自動收起，變回綠色
 * 
 * 3. 霍爾感測器 / 紅外線遙控
 *    → DOWN: setRGBColor(RGB_YELLOW)
 *    → UP: setRGBColor(RGB_GREEN)
 * 
 * 4. 自動模式
 *    → 速度 < 10 km/h: setRGBColor(RGB_YELLOW)
 *    → 速度 ≥ 10 km/h: setRGBColor(RGB_GREEN)
 * 
 * 5. 測試模式
 *    → setRGBColor(RGB_PURPLE)
 *    → 測試完成後 setRGBColor(RGB_OFF)
 *    → 進入正常模式後 setRGBColor(RGB_GREEN)
 * 
 * ============================================================================
 */
