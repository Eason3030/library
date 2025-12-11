/*
 * ============================================================================
 * 自行車輔助輪智能控制系統
 * ============================================================================
 * 功能：紅外線遙控 + 速度自動控制 + LED 模組指示 + 簡潔音效系統
 * 作者：[您的名字]
 * 版本：v2.1
 * ============================================================================
 */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <IRremote.h>

// ============================================================================
// 簡化音符定義（只定義需要的）
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
// LED 模組指示燈腳位
// ============================================================================
const int LED_OLED = 5;
const int LED_MPU = 7;
const int LED_HALL = 9;
const int LED_RELAY = 10;
const int LED_BUZZER = 12;
const int LED_IR = 6;

// ============================================================================
// 紅外線遙控器按鍵碼（學習模式）
// ============================================================================
const uint32_t IR_CODE_DOWN = 0xFFA857;  // 向下按鍵碼（輔助輪下降）
const uint32_t IR_CODE_UP = 0xFF629D;    // 向上按鍵碼（輔助輪上升）
const unsigned long IR_DELAY = 500;      // 防連發延遲時間（毫秒）
unsigned long lastIRTime = 0;      
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
// 【音效函式】播放音符
// ============================================================================
void beep(int freq, int duration) {
  tone(BUZZER_PIN, freq, duration);
  delay(duration);
  noTone(BUZZER_PIN);
}

// ============================================================================
// 【音效函式】開機音樂
// ============================================================================
void soundStartup() {
  beep(NOTE_C5, 100);
  beep(NOTE_E5, 100);
  beep(NOTE_G5, 150);
  delay(100);
  beep(NOTE_C6, 200);
}

// ============================================================================
// 【音效函式】輔助輪下降
// ============================================================================
void soundDown() {
  beep(NOTE_G5, 60);
  beep(NOTE_E5, 60);
  beep(NOTE_C5, 100);
}

// ============================================================================
// 【音效函式】輔助輪上升
// ============================================================================
void soundUp() {
  beep(NOTE_C5, 60);
  beep(NOTE_E5, 60);
  beep(NOTE_G5, 100);
}

// ============================================================================
// 【音效函式】按鈕確認音
// ============================================================================
void soundClick() {
  beep(NOTE_C6, 50);
}

// ============================================================================
// 【音效函式】測試完成
// ============================================================================
void soundComplete() {
  beep(NOTE_C5, 80);
  beep(NOTE_E5, 80);
  beep(NOTE_G5, 80);
  beep(NOTE_C6, 150);
}

// ============================================================================
// 【中斷服務函式】速度測量中斷
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
// 【函式】計算當前速度
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
// 【函式】MPU6050 多位元組讀取
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

// ============================================================================
// 【函式】讀取 16 位元暫存器值
// ============================================================================
int16_t read16(uint8_t reg) {
  uint8_t buf[2];
  mpuReadBytes(reg, 2, buf);
  return (int16_t)((buf[0] << 8) | buf[1]);
}

// ============================================================================
// 【函式】初始化 MPU6050
// ============================================================================
void setupMPU() {
  Wire.beginTransmission(MPU_6050);
  Wire.write(PWR_MGMT_1);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(50);
}

// ============================================================================
// 【函式】校正 IMU（計算偏移量）
// ============================================================================
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

// ============================================================================
// 【函式】讀取 IMU 數據並計算角度
// ============================================================================
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
// 【函式】關閉所有模組 LED
// ============================================================================
void allModuleLEDsOff() {
  digitalWrite(LED_OLED, LOW);
  digitalWrite(LED_IR, LOW);
  digitalWrite(LED_MPU, LOW);
  digitalWrite(LED_HALL, LOW);
  digitalWrite(LED_RELAY, LOW);
  digitalWrite(LED_BUZZER, LOW);
}

// ============================================================================
// 【函式】啟動時 LED 流水燈動畫（加入音效）
// ============================================================================
void startupLEDs() {
  int leds[] = {LED_OLED, LED_IR, LED_MPU, LED_HALL, LED_RELAY, LED_BUZZER};

  // 正向流水燈
  for (int i = 0; i < 6; i++) {
    digitalWrite(leds[i], HIGH);
    beep(400 + i * 100, 60);
    delay(40);
  }
  delay(200);

  // 反向熄滅
  for (int i = 5; i >= 0; i--) {
    digitalWrite(leds[i], LOW);
    delay(100);
  }
  delay(200);

  // 全亮閃爍 3 次
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 6; j++) digitalWrite(leds[j], HIGH);
    beep(NOTE_C6, 80);
    delay(70);
    allModuleLEDsOff();
    delay(150);
  }
}

// ============================================================================
// 【函式】OLED 啟動畫面
// ============================================================================
void showStartup() {
  display.clearDisplay();

  // LOGO 閃爍動畫
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

  // 載入進度條動畫
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
// 【函式】OLED 主畫面顯示
// ============================================================================
void showStatus() {
  display.clearDisplay();

  display.setTextSize(1);
  display.drawRect(0, 0, 128, 10, SSD1306_WHITE);
  
  display.setCursor(2, 2);
  
  if (ctrlSrc == 1) display.print("HALL");
  else if (ctrlSrc == 2) display.print("IR-REM");
  else if (ctrlSrc == 3) display.print("AUTO");
  else display.print("INIT");
  
  display.print(" | ");
  display.print(currentSpeed, 0);
  display.print("km");
  display.print(" | ");
  
  int angle = (int)compAngleX;
  if (angle >= 0) display.print(" ");
  display.print(angle);
  display.print("d");

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
// 【函式】OLED 模組測試畫面
// ============================================================================
void testModules() {
  display.clearDisplay();
  display.fillRect(0, 0, 128, 32, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setTextSize(2);
  display.setCursor(20, 8);
  display.print("TEST");
  display.display();
  display.setTextColor(SSD1306_WHITE);
  beep(NOTE_C5, 150);
  delay(850);

  for (int i = 0; i < 7; i++) {
    digitalWrite(LED_OLED + i, HIGH);
    display.clearDisplay();

    display.setTextSize(1);
    display.drawRect(0, 0, 128, 9, SSD1306_WHITE);
    display.setCursor(2, 1);
    display.print("TEST");

    display.setTextSize(2);
    int nameYPos = 12;

    soundClick();

    if (i == 0) {
      display.setCursor(25, nameYPos);
      display.print("OLED");
    } else if (i == 1) {
      display.setCursor(25, nameYPos);
      display.print("IR");
    } else if (i == 2) {
      display.setCursor(25, nameYPos);
      display.print("MPU");
    } else if (i == 3) {
      display.setCursor(25, nameYPos);
      display.print("HALL");
    } else if (i == 4) {
      display.setCursor(25, nameYPos);
      display.print("SPEED");
    } else if (i == 5) {
      display.setCursor(25, nameYPos);
      display.print("RELAY");
    } else if (i == 6) {
      display.setCursor(25, nameYPos);
      display.print("BUZZ");
    }

    display.display();

    if (i == 0) {
      displayTestResult("V", 600);
      beep(NOTE_E5, 100);
    } else if (i == 1) {
      bool detected = testIR();
      displayTestResult(detected ? "V" : "X", 400);
      beep(detected ? NOTE_E5 : NOTE_C5, 100);
    } else if (i == 2) {
      testMPU();
      displayTestResult("V", 600);
      beep(NOTE_E5, 100);
    } else if (i == 3) {
      bool detected = testHallSensor();
      displayTestResult(detected ? "V" : "X", 600);
      beep(detected ? NOTE_E5 : NOTE_C5, 100);
    } else if (i == 4) {
      bool detected = testSpeedSensor();
      displayTestResult(detected ? "V" : "X", 600);
      beep(detected ? NOTE_E5 : NOTE_C5, 100);
    } else if (i == 5) {
      testRelay();
      displayTestResult("V", 400);
      beep(NOTE_E5, 100);
    } else if (i == 6) {
      testBuzzer();
      displayTestResult("OK", 400);
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
  for (int i = 0; i < 3; i++) {
    flashAllLEDs();
  }

  delay(1000);
  allModuleLEDsOff();
}

void displayTestResult(const char* result, int delayTime) {
  display.setTextSize(2);
  display.setCursor(95, 12);
  display.print(result);
  display.display();
  delay(delayTime);
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

void testRelay() {
  digitalWrite(RELAY_PIN, HIGH);
  delay(200);
  digitalWrite(RELAY_PIN, LOW);
}

void testBuzzer() {
  beep(NOTE_C5, 80);
  beep(NOTE_E5, 80);
  beep(NOTE_G5, 100);
}

void flashAllLEDs() {
  for (int i = 0; i < 6; i++) {
    digitalWrite(LED_OLED + i, HIGH);
  }
  delay(100);
  allModuleLEDsOff();
  delay(100);
}

// ============================================================================
// 【函式】處理紅外線遙控指令
// ============================================================================
void handleIRCommand(uint32_t code) {
  // === 防連發檢查 ===
  if (millis() - lastIRTime < IR_DELAY) {
    return;  // 距離上次接收時間太短，忽略此次信號
  }

  bool validCommand = false;             // 有效指令旗標

  // === 判斷接收到的指令碼 ===
  if (code == IR_CODE_DOWN) {            // 如果是向下指令
    wheelDown = true;                    // 設定輔助輪下降
    validCommand = true;                 // 標記為有效指令
    Serial.println("IR: Wheel DOWN");    // 序列埠輸出訊息

  } else if (code == IR_CODE_UP) {       // 如果是向上指令
    wheelDown = false;                   // 設定輔助輪上升
    validCommand = true;                 // 標記為有效指令
    Serial.println("IR: Wheel UP");      // 序列埠輸出訊息
  }

  // === 如果是有效指令，執行動作 ===
  if (validCommand) {
    ctrlSrc = 2;                         // 標記控制來源為紅外線遙控
    manualMode = true;                   // 進入手動模式（停用速度自動控制）

    digitalWrite(RELAY_PIN, wheelDown ? HIGH : LOW);  // 控制繼電器
    digitalWrite(LED_IR, HIGH);          // 點亮 IR LED 表示接收到信號

    // === 聲音回饋 ===
    digitalWrite(BUZZER_PIN, HIGH);      // 開啟蜂鳴器
    delay(wheelDown ? 100 : 50);         // 向下時響 100ms，向上時響 50ms
    digitalWrite(BUZZER_PIN, LOW);       // 關閉蜂鳴器

    digitalWrite(LED_IR, LOW);           // 熄滅 IR LED

    // === 顯示動畫 ===
    showWheelAnim = true;                // 啟用動畫
    wheelAnimStartTime = millis();       // 記錄動畫開始時間

    lastIRTime = millis();               // 更新上次接收時間（防連發）
  }
}

// ============================================================================
// 【設定函式】Setup
// ============================================================================
void setup() {
  // === 序列埠初始化 ===
  Serial.begin(115200);                  // 啟動序列埠，鮑率 115200
  delay(100);                            // 等待序列埠穩定

  // === I2C 初始化 ===
  Wire.begin();                          // 啟動 I2C 通訊

  // === 數位腳位設定 ===
  pinMode(BUZZER_PIN, OUTPUT);           // 蜂鳴器設為輸出
  pinMode(RELAY_PIN, OUTPUT);            // 繼電器設為輸出
  pinMode(HALL_SENSOR_PIN, INPUT);       // 霍爾感測器設為輸入
  pinMode(HALL_SPEED_PIN, INPUT_PULLUP); // 測速霍爾設為上拉輸入

  // === LED 腳位設定 ===
  pinMode(LED_OLED, OUTPUT);             // OLED LED 設為輸出
  pinMode(LED_IR, OUTPUT);               // IR LED 設為輸出
  pinMode(LED_MPU, OUTPUT);              // MPU LED 設為輸出
  pinMode(LED_HALL, OUTPUT);             // HALL LED 設為輸出
  pinMode(LED_RELAY, OUTPUT);            // RELAY LED 設為輸出
  pinMode(LED_BUZZER, OUTPUT);           // BUZZER LED 設為輸出

  // === 初始狀態設定 ===
  digitalWrite(RELAY_PIN, LOW);          // 關閉繼電器
  digitalWrite(BUZZER_PIN, LOW);         // 關閉蜂鳴器
  allModuleLEDsOff();                    // 關閉所有 LED

  // === 紅外線接收器初始化 ===
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);  // 啟動 IR 接收器
  Serial.println("IR Receiver initialized");              // 序列埠輸出訊息

  // === 速度測量中斷初始化 ===
  attachInterrupt(digitalPinToInterrupt(HALL_SPEED_PIN), speedInterrupt, FALLING);  // 附加中斷

  // === OLED 初始化 ===
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {  // 初始化 OLED（I2C 位址 0x3C）
    Serial.println("OLED Fail");         // 如果失敗，輸出錯誤訊息
    while (1);                           // 進入無限迴圈（停止執行）
  }

  // === MPU6050 初始化 ===
  setupMPU();                            // 設定 MPU6050
  calibrateIMU();                        // 校正 IMU

  // === 啟動畫面 ===
  startupLEDs();                         // LED 流水燈動畫
  showStartup();                         // OLED 啟動畫面
  testModules();                         // 模組測試畫面

  // === 系統就緒 ===
  Serial.println("=== READY ===");       // 序列埠輸出就緒訊息
  Serial.print("IR Codes: UP=0x");       // 輸出紅外線碼資訊
  Serial.print(IR_CODE_UP, HEX);         // 輸出向上指令碼（16進位）
  Serial.print(", DOWN=0x");             // 分隔
  Serial.println(IR_CODE_DOWN, HEX);     // 輸出向下指令碼（16進位）
  Serial.print("Wheel: ");               // 輸出輪子資訊
  Serial.print(WHEEL_DIAMETER);          // 輸出輪徑
  Serial.println(" cm");                 // 單位
  Serial.print("Threshold: ");           // 輸出門檻值
  Serial.print(SPEED_THRESHOLD);         // 輸出速度門檻
  Serial.println(" km/h");               // 單位
}

// ============================================================================
// 【主迴圈】Loop
// ============================================================================
void loop() {
  float ax, gx;                          // 宣告加速度與角速度變數
  readIMU(ax, gx);                       // 讀取 IMU 數據並更新角度

  // ========== 速度測量 ==========
  if (newSpeedData) {                    // 如果有新的速度數據
    currentSpeed = calculateSpeed();     // 計算當前速度
    newSpeedData = false;                // 重置速度數據旗標
    lastSpeedUpdate = millis();          // 更新速度更新時間
  }

  // === 速度信號超時檢測 ===
  if (millis() - lastSpeedUpdate > SPEED_TIMEOUT) {  // 如果超過 2 秒沒有速度信號
    currentSpeed = 0.0;                  // 將速度設為 0
  }

  // ========== 紅外線遙控接收 ==========
  if (IrReceiver.decode()) {             // 如果接收到紅外線信號
    uint32_t code = IrReceiver.decodedIRData.decodedRawData;  // 取得接收到的碼

    // === 輸出接收到的碼（除錯用） ===
    Serial.print("IR Received: 0x");     // 序列埠輸出前綴
    Serial.println(code, HEX);           // 輸出接收到的碼（16進位）

    handleIRCommand(code);               // 處理紅外線指令

    IrReceiver.resume();                 // 準備接收下一個信號
  }

  // ========== 速度自動控制（非手動模式） ==========
  if (!manualMode) {                     // 如果在自動模式
    bool shouldWheelDown = (currentSpeed < SPEED_THRESHOLD);  // 判斷是否應下降輔助輪

    if (shouldWheelDown != wheelDown) {  // 如果狀態改變
      wheelDown = shouldWheelDown;       // 更新輔助輪狀態
      ctrlSrc = 3;                       // 標記控制來源為速度自動
      digitalWrite(RELAY_PIN, wheelDown ? HIGH : LOW);  // 控制繼電器

      // === 聲音回饋 ===
      digitalWrite(BUZZER_PIN, HIGH);    // 開啟蜂鳴器
      delay(wheelDown ? 100 : 50);       // 向下時響 100ms，向上時響 50ms
      digitalWrite(BUZZER_PIN, LOW);     // 關閉蜂鳴器

      // === 顯示動畫 ===
      showWheelAnim = true;              // 啟用動畫
      wheelAnimStartTime = millis();     // 記錄動畫開始時間

      // === 序列埠輸出 ===
      Serial.print("AUTO: Speed ");      // 輸出前綴
      Serial.print(currentSpeed);        // 輸出當前速度
      Serial.print(" -> Wheel ");        // 分隔
      Serial.println(wheelDown ? "DOWN" : "UP");  // 輸出輔助輪狀態
    }
  }

  // 霍爾感測器手動控制
  int hall = digitalRead(HALL_SENSOR_PIN);
  if (hall != lastHallState) {
    wheelDown = (hall == LOW);
    ctrlSrc = 1;
    manualMode = true;

    digitalWrite(RELAY_PIN, wheelDown ? HIGH : LOW);
    digitalWrite(LED_HALL, HIGH);

    if (wheelDown) {
      soundDown();
    } else {
      soundUp();
    }

    digitalWrite(LED_HALL, LOW);

    showWheelAnim = true;
    wheelAnimStartTime = millis();
    lastHallState = hall;

    Serial.println("HALL: Manual control");
  }

  // 顯示更新
  if (millis() - lastDisplayTime > 50) {
    showStatus();
    lastDisplayTime = millis();
  }

  // 序列埠輸出
  if (millis() - lastPrintTime > 500) {
    Serial.print("Speed: ");
    Serial.print(currentSpeed, 1);
    Serial.print(" km/h | Angle: ");
    Serial.print(compAngleX, 1);
    Serial.print("° | Wheel: ");
    Serial.print(wheelDown ? "DOWN" : "UP");
    Serial.print(" | Mode: ");
    Serial.println(manualMode ? "MANUAL" : "AUTO");
    lastPrintTime = millis();
  }

  delay(20);
}
