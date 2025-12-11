/*
 * ============================================================================
 * 自行車輔助輪智能控制系統
 * ============================================================================
 * 功能：紅外線遙控 + 速度自動控制 + LED 模組指示
 * 作者：[您的名字]
 * 版本：v2.0
 * ============================================================================
 */

#include <Wire.h>                      // I2C 通訊函式庫
#include <Adafruit_GFX.h>              // Adafruit 圖形函式庫
#include <Adafruit_SSD1306.h>          // OLED 顯示器函式庫
#include <IRremote.h>                  // 紅外線遙控函式庫

// ============================================================================
// OLED 顯示器設定
// ============================================================================
#define SCREEN_WIDTH 128               // OLED 寬度（像素）
#define SCREEN_HEIGHT 32               // OLED 高度（像素）
#define OLED_RESET -1                  // OLED 重置腳（-1 表示共用 Arduino 重置腳）
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ============================================================================
// MPU6050 陀螺儀/加速度計設定
// ============================================================================
#define MPU_6050 0x68                  // MPU6050 的 I2C 位址
#define PWR_MGMT_1 0x6B                // 電源管理暫存器位址
#define ACCEL_XOUT_H 0x3B              // X 軸加速度高位元組位址
#define GYRO_XOUT_H  0x43              // X 軸陀螺儀高位元組位址

// ============================================================================
// 數位腳位定義
// ============================================================================
const int BUZZER_PIN = 8;              // 蜂鳴器腳位
const int RELAY_PIN  = 11;             // 繼電器腳位（控制輔助輪馬達）
const int IR_RECEIVE_PIN = 3;          // 紅外線接收器腳位
const int HALL_SENSOR_PIN = 4;         // 霍爾感測器腳位（手動控制）
const int HALL_SPEED_PIN = 2;          // 測速霍爾感測器腳位（中斷腳）

// ============================================================================
// LED 模組指示燈腳位
// ============================================================================
const int LED_OLED = 5;                // OLED 模組指示燈
const int LED_MPU = 7;                 // MPU6050 模組指示燈
const int LED_HALL = 9;                // 霍爾感測器指示燈
const int LED_RELAY = 10;              // 繼電器模組指示燈
const int LED_BUZZER = 12;             // 蜂鳴器模組指示燈
const int LED_IR = 6;                  // 紅外線模組指示燈

// ============================================================================
// 紅外線遙控器按鍵碼
// ============================================================================
const uint32_t IR_CODE_DOWN = 0xEA15FF00;  // 向下按鍵碼（輔助輪下降）
const uint32_t IR_CODE_UP =0xB946FF00;    // 向上按鍵碼（輔助輪上升）
const unsigned long IR_DELAY = 500;      // 防連發延遲時間（毫秒）
unsigned long lastIRTime = 0;            // 上次接收紅外線信號的時間

// ============================================================================
// 速度測量參數
// ============================================================================
const float WHEEL_DIAMETER = 65.0;           // 輪子直徑（公分）
const float WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * 3.14159;  // 輪子圓周長（公分）
const float SPEED_THRESHOLD = 10.0;          // 速度門檻值（km/h），低於此值輔助輪下降
const unsigned long SPEED_TIMEOUT = 2000;    // 速度信號超時時間（毫秒），超過此時間視為停止

// ============================================================================
// IMU 濾波器參數
// ============================================================================
const float dt = 0.01f;                // 採樣時間間隔（秒）
const float alpha = 0.98f;             // 互補濾波器係數（0.98 = 98% 陀螺儀 + 2% 加速度計）
float compAngleX = 0.0f;               // 互補濾波後的 X 軸角度
float gyroBiasX = 0.0f;                // 陀螺儀 X 軸偏移量
float accelBiasX = 0.0f;               // 加速度計 X 軸偏移量

// ============================================================================
// 系統狀態變數
// ============================================================================
bool wheelDown = false;                // 輔助輪狀態（true=下降, false=上升）
unsigned long wheelAnimStartTime = 0;  // 輪子動畫開始時間
bool showWheelAnim = false;            // 是否顯示輪子動畫
byte ctrlSrc = 0;                      // 控制來源（0=初始, 1=霍爾, 2=紅外線, 3=速度自動）

int lastHallState = HIGH;              // 上次霍爾感測器狀態
unsigned long lastPrintTime = 0;       // 上次序列埠輸出時間
unsigned long lastDisplayTime = 0;     // 上次 OLED 更新時間
bool manualMode = false;               // 手動模式旗標（true=手動, false=自動）

// ============================================================================
// 速度測量變數（中斷使用）
// ============================================================================
volatile unsigned long lastSpeedTrigger = 0;      // 上次速度觸發時間（微秒）
volatile unsigned long timeBetweenTriggers = 0;   // 兩次觸發之間的時間間隔（微秒）
volatile bool newSpeedData = false;               // 是否有新的速度數據
float currentSpeed = 0.0;                         // 當前速度（km/h）
unsigned long lastSpeedUpdate = 0;                // 上次速度更新時間

// ============================================================================
// 【中斷服務函式】速度測量中斷
// ============================================================================
void speedInterrupt() {
  unsigned long currentTime = micros();  // 取得當前時間（微秒）
  
  // 防抖動處理：至少間隔 50ms 才視為有效觸發
  if (currentTime - lastSpeedTrigger > 50000) {
    timeBetweenTriggers = currentTime - lastSpeedTrigger;  // 計算時間間隔
    lastSpeedTrigger = currentTime;                        // 更新上次觸發時間
    newSpeedData = true;                                   // 標記有新數據
  }
}

// ============================================================================
// 【函式】計算當前速度
// ============================================================================
float calculateSpeed() {
  if (timeBetweenTriggers == 0) return 0.0;  // 沒有數據則返回 0
  
  float distancePerTrigger = WHEEL_CIRCUMFERENCE;           // 一次觸發 = 一圈 = 圓周長（公分）
  float timeInSeconds = timeBetweenTriggers / 1000000.0;    // 將微秒轉換為秒
  float speedCmPerSec = distancePerTrigger / timeInSeconds; // 公分/秒
  float speedKmPerHour = speedCmPerSec * 0.036;             // 轉換為 km/h
  
  return speedKmPerHour;  // 返回速度（km/h）
}

// ============================================================================
// 【函式】MPU6050 多位元組讀取
// ============================================================================
void mpuReadBytes(uint8_t reg, uint8_t count, uint8_t* dest) {
  Wire.beginTransmission(MPU_6050);     // 開始 I2C 傳輸
  Wire.write(reg);                      // 寫入暫存器位址
  Wire.endTransmission(false);          // 結束傳輸但保持連線
  Wire.requestFrom((uint8_t)MPU_6050, count);  // 請求讀取指定數量的位元組
  for (uint8_t i = 0; i < count; i++) {
    dest[i] = Wire.read();               // 逐一讀取資料
  }
}

// ============================================================================
// 【函式】讀取 16 位元暫存器值
// ============================================================================
int16_t read16(uint8_t reg) {
  uint8_t buf[2];                        // 建立 2 位元組緩衝區
  mpuReadBytes(reg, 2, buf);             // 讀取 2 位元組
  return (int16_t)((buf[0] << 8) | buf[1]);  // 合併高低位元組為 16 位元整數
}

// ============================================================================
// 【函式】初始化 MPU6050
// ============================================================================
void setupMPU() {
  Wire.beginTransmission(MPU_6050);      // 開始 I2C 傳輸
  Wire.write(PWR_MGMT_1);                // 選擇電源管理暫存器
  Wire.write(0x00);                      // 寫入 0x00 喚醒 MPU6050
  Wire.endTransmission();                // 結束傳輸
  delay(50);                             // 等待 MPU6050 穩定
}

// ============================================================================
// 【函式】校正 IMU（計算偏移量）
// ============================================================================
void calibrateIMU() {
  float sumAx = 0, sumGx = 0;            // 初始化累加變數
  
  for (byte i = 0; i < 50; i++) {        // 採樣 50 次
    sumAx += read16(ACCEL_XOUT_H) / 16384.0f;  // 累加加速度值（±2g）
    sumGx += read16(GYRO_XOUT_H) / 131.0f;     // 累加陀螺儀值（±250°/s）
    delay(5);                            // 每次採樣間隔 5ms
  }
  
  accelBiasX = sumAx / 50;               // 計算加速度平均偏移量
  gyroBiasX  = sumGx / 50;               // 計算陀螺儀平均偏移量
}

// ============================================================================
// 【函式】讀取 IMU 數據並計算角度
// ============================================================================
void readIMU(float &ax, float &gx) {
  ax = read16(ACCEL_XOUT_H) / 16384.0f;      // 讀取 X 軸加速度（g）
  float az = read16(ACCEL_XOUT_H + 4) / 16384.0f;  // 讀取 Z 軸加速度（g）
  gx = read16(GYRO_XOUT_H) / 131.0f;         // 讀取 X 軸角速度（°/s）

  gx -= gyroBiasX;                           // 移除陀螺儀偏移量
  ax -= accelBiasX;                          // 移除加速度偏移量

  float accelAngle = atan2(ax, az) * 57.2958f;  // 用加速度計算角度（弧度轉角度）
  compAngleX = alpha * (compAngleX + gx * dt)   // 互補濾波器公式
             + (1 - alpha) * accelAngle;        // 融合陀螺儀與加速度計數據
}

// ============================================================================
// 【函式】關閉所有模組 LED
// ============================================================================
void allModuleLEDsOff() {
  digitalWrite(LED_OLED, LOW);           // 關閉 OLED LED
  digitalWrite(LED_IR, LOW);             // 關閉 IR LED
  digitalWrite(LED_MPU, LOW);            // 關閉 MPU LED
  digitalWrite(LED_HALL, LOW);           // 關閉 HALL LED
  digitalWrite(LED_RELAY, LOW);          // 關閉 RELAY LED
  digitalWrite(LED_BUZZER, LOW);         // 關閉 BUZZER LED
}

// ============================================================================
// 【函式】啟動時 LED 流水燈動畫
// ============================================================================
void startupLEDs() {
  int leds[] = {LED_OLED, LED_IR, LED_MPU, LED_HALL, LED_RELAY, LED_BUZZER};  // LED 陣列

  // === 正向流水燈 ===
  for (int i = 0; i < 6; i++) {
    digitalWrite(leds[i], HIGH);         // 依序點亮 LED
    delay(100);                          // 延遲 100ms
  }
  delay(200);                            // 全亮停留 200ms

  // === 反向熄滅 ===
  for (int i = 5; i >= 0; i--) {
    digitalWrite(leds[i], LOW);          // 依序熄滅 LED
    delay(100);                          // 延遲 100ms
  }
  delay(200);                            // 全滅停留 200ms

  // === 全亮閃爍 3 次 ===
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 6; j++) digitalWrite(leds[j], HIGH);  // 全部點亮
    delay(150);                          // 延遲 150ms
    allModuleLEDsOff();                  // 全部熄滅
    delay(150);                          // 延遲 150ms
  }
}

// ============================================================================
// 【函式】OLED 啟動畫面
// ============================================================================
void showStartup() {
  display.clearDisplay();                // 清空顯示緩衝區

  // === LOGO 閃爍動畫（2次） ===
  for (int i = 0; i < 2; i++) {
    display.fillRect(0, 0, 128, 32, SSD1306_WHITE);  // 填滿白色背景
    display.setTextColor(SSD1306_BLACK);             // 設定文字為黑色（反白）
    display.setTextSize(3);                          // 設定文字大小為 3
    display.setCursor(14, 6);                        // 設定游標位置
    display.print("BIKE");                           // 顯示 "BIKE"
    display.display();                               // 更新顯示
    display.setTextColor(SSD1306_WHITE);             // 恢復文字為白色
    delay(300);                                      // 停留 300ms
    display.clearDisplay();                          // 清空顯示
    display.display();                               // 更新顯示
    delay(150);                                      // 停留 150ms
  }

  // === 載入進度條動畫 ===
  display.clearDisplay();                // 清空顯示
  display.setTextSize(1);                // 設定文字大小為 1
  display.setCursor(28, 8);              // 設定游標位置
  display.print("LOADING");              // 顯示 "LOADING"

  for (int i = 0; i <= 100; i += 10) {   // 進度條從 0% 到 100%
    display.fillRect(14, 20, i, 4, SSD1306_WHITE);  // 繪製進度條
    display.display();                   // 更新顯示
    delay(80);                           // 每 10% 延遲 80ms
  }
  delay(200);                            // 完成後停留 200ms
}

// ============================================================================
// 【函式】OLED 主畫面顯示
// ============================================================================
void showStatus() {
  display.clearDisplay();                // 清空顯示緩衝區

  // ========== 頂部標籤框（單一大框） ==========
  display.setTextSize(1);                // 設定文字大小為 1
  display.drawRect(0, 0, 128, 10, SSD1306_WHITE);  // 繪製頂部大框
  
  // === 標籤內容：來源 + 速度 + 角度 ===
  display.setCursor(2, 2);               // 設定游標位置
  
  // 控制來源
  if (ctrlSrc == 1) display.print("HALL");        // 霍爾感測器控制
  else if (ctrlSrc == 2) display.print("IR-REM"); // 紅外線遙控控制
  else if (ctrlSrc == 3) display.print("AUTO");   // 速度自動控制
  else display.print("INIT");                     // 初始狀態
  
  // 分隔符
  display.print(" | ");
  
  // 速度顯示
  display.print(currentSpeed, 0);        // 顯示速度（無小數）
  display.print("km");                   // 單位
  
  // 分隔符
  display.print(" | ");
  
  // 角度顯示
  int angle = (int)compAngleX;           // 取整數角度
  if (angle >= 0) display.print(" ");    // 正數補空格對齊
  display.print(angle);                  // 顯示角度
  display.print("d");                    // 度數符號

  // ========== 中央大狀態顯示 ==========
  display.setTextSize(2);                // 設定文字大小為 2

  // === 如果正在播放動畫（600ms 內） ===
  if (showWheelAnim && (millis() - wheelAnimStartTime < 600)) {
    byte frame = ((millis() - wheelAnimStartTime) / 100) % 3;  // 計算當前幀（0-2）

    if (wheelDown) {  // 輔助輪下降動畫
      display.setCursor(4, 14);          // 設定游標位置
      if (frame == 0) display.print("|");       // 幀 0：顯示 |
      else if (frame == 1) display.print("v");  // 幀 1：顯示 v
      else display.print("V");                  // 幀 2：顯示 V

      display.setCursor(20, 14);         // 設定游標位置
      display.print("DOWN");             // 顯示 "DOWN"

      int arrowY = 16 + frame * 3;       // 箭頭 Y 座標（向下移動）
      display.fillTriangle(108, arrowY, 112, arrowY+4, 116, arrowY, SSD1306_WHITE);      // 上箭頭
      display.fillTriangle(108, arrowY+5, 112, arrowY+9, 116, arrowY+5, SSD1306_WHITE);  // 下箭頭

    } else {  // 輔助輪上升動畫
      display.setCursor(4, 14);          // 設定游標位置
      if (frame == 0) display.print("|");       // 幀 0：顯示 |
      else if (frame == 1) display.print("^");  // 幀 1：顯示 ^
      else display.print("^");                  // 幀 2：顯示 ^

      display.setCursor(20, 14);         // 設定游標位置
      display.print("UP");               // 顯示 "UP"

      int arrowY = 24 - frame * 3;       // 箭頭 Y 座標（向上移動）
      display.fillTriangle(108, arrowY+9, 112, arrowY+5, 116, arrowY+9, SSD1306_WHITE);  // 下箭頭
      display.fillTriangle(108, arrowY+4, 112, arrowY, 116, arrowY+4, SSD1306_WHITE);    // 上箭頭
    }

  } else {  // === 靜態顯示 ===
    showWheelAnim = false;               // 動畫結束

    if (wheelDown) {  // 輔助輪下降狀態
      display.setCursor(4, 14);          // 設定游標位置
      display.print("V DOWN");           // 顯示 "V DOWN"
      display.fillRect(0, 30, 128, 2, SSD1306_WHITE);  // 底部線條
      display.fillTriangle(110, 16, 114, 20, 118, 16, SSD1306_WHITE);  // 箭頭 1
      display.fillTriangle(110, 22, 114, 26, 118, 22, SSD1306_WHITE);  // 箭頭 2

    } else {  // 輔助輪上升狀態
      display.setCursor(16, 14);         // 設定游標位置
      display.print("^ UP");             // 顯示 "^ UP"

      for (int x = 0; x < 128; x += 6) { // 虛線效果
        display.drawFastHLine(x, 12, 3, SSD1306_WHITE);  // 繪製短橫線
      }

      display.fillTriangle(110, 26, 114, 22, 118, 26, SSD1306_WHITE);  // 箭頭 1
      display.fillTriangle(110, 20, 114, 16, 118, 20, SSD1306_WHITE);  // 箭頭 2
    }
  }

  display.display();                     // 更新顯示
}

// ============================================================================
// 【函式】OLED 模組測試畫面
// ============================================================================
void testModules() {
  // === 測試標題畫面 ===
  display.clearDisplay();
  display.fillRect(0, 0, 128, 32, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setTextSize(2);  // 統一字體大小
  display.setCursor(20, 8);  // 置中並上移
  display.print("TEST");
  display.display();
  display.setTextColor(SSD1306_WHITE);
  delay(1000);

  // 逐一測試 7 個模組
  for (int i = 0; i < 7; i++) {
    digitalWrite(LED_OLED + i, HIGH);
    display.clearDisplay();

    // 顯示測試標題和模組名稱
    display.setTextSize(1); // 用較小字體顯示標題
    display.drawRect(0, 0, 128, 9, SSD1306_WHITE); // 顯示標題框
    display.setCursor(2, 1);
    display.print("TEST");

    display.setTextSize(2);  // 統一字體大小為 2
    int nameYPos = 12;  // 預設顯示位置

    // 根據測試項目顯示名稱
    if (i == 0) { // OLED
      display.setCursor(25, nameYPos);  
      display.print("OLED");
    } else if (i == 1) { // IR
      display.setCursor(25, nameYPos);  
      display.print("IR");
    } else if (i == 2) { // MPU
      display.setCursor(25, nameYPos);  
      display.print("MPU");
    } else if (i == 3) { // HALL
      display.setCursor(25, nameYPos);  
      display.print("HALL");
    } else if (i == 4) { // SPEED
      display.setCursor(25, nameYPos);  
      display.print("SPEED");
    } else if (i == 5) { // RELAY
      display.setCursor(25, nameYPos);  
      display.print("RELAY");
    } else if (i == 6) { // BUZZ
      display.setCursor(25, nameYPos);  
      display.print("BUZZ");
    }

    display.display();

    // 執行對應模組測試
    if (i == 0) {  // OLED 測試
      displayTestResult("V", 600);
    } else if (i == 1) {  // IR 紅外線測試
      bool detected = testIR();
      displayTestResult(detected ? "V" : "X", 400);
    } else if (i == 2) {  // MPU 陀螺儀測試
      testMPU();
      displayTestResult("V", 600);
    } else if (i == 3) {  // HALL 霍爾感測器測試
      bool detected = testHallSensor();
      displayTestResult(detected ? "V" : "X", 600);
    } else if (i == 4) {  // SPEED 測速感測器測試
      bool detected = testSpeedSensor();
      displayTestResult(detected ? "V" : "X", 600);
    } else if (i == 5) {  // RELAY 繼電器測試
      testRelay();
      displayTestResult("V", 400);
    } else if (i == 6) {  // BUZZER 蜂鳴器測試
      testBuzzer();
      displayTestResult("OK", 400);
    }

    delay(200);
  }

  // === 完成畫面 ===
  display.clearDisplay();
  display.fillRect(0, 0, 128, 32, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setTextSize(2);  // 統一字體大小
  display.setCursor(30, 8);  // 置中並上移
  display.print("DONE");
  display.display();
  display.setTextColor(SSD1306_WHITE);

  // === 成功音效 + LED 閃爍 ===
  for (int i = 0; i < 3; i++) {
    flashAllLEDs();
  }

  delay(1000);
  allModuleLEDsOff();
}

void displayTestResult(const char* result, int delayTime) {
  display.setTextSize(2);  // 統一字體大小
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
  for (int j = 0; j < 2; j++) {
    digitalWrite(BUZZER_PIN, HIGH);
    digitalWrite(LED_BUZZER, LOW);
    delay(75);
    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(LED_BUZZER, HIGH);
    delay(75);
  }
}

void flashAllLEDs() {
  for (int i = 0; i < 6; i++) {
    digitalWrite(LED_OLED + i, HIGH);
  }
  digitalWrite(BUZZER_PIN, HIGH);
  delay(100);

  allModuleLEDsOff();
  digitalWrite(BUZZER_PIN, LOW);
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

  // ========== 霍爾感測器手動控制 ==========
  int hall = digitalRead(HALL_SENSOR_PIN);  // 讀取霍爾感測器狀態
  if (hall != lastHallState) {           // 如果狀態改變
    wheelDown = (hall == LOW);           // 磁鐵靠近（LOW）= 輔助輪下降
    ctrlSrc = 1;                         // 標記控制來源為霍爾感測器
    manualMode = true;                   // 進入手動模式

    digitalWrite(RELAY_PIN, wheelDown ? HIGH : LOW);  // 控制繼電器
    digitalWrite(LED_HALL, HIGH);        // 點亮 HALL LED

    // === 聲音回饋 ===
    digitalWrite(BUZZER_PIN, HIGH);      // 開啟蜂鳴器
    delay(wheelDown ? 100 : 50);         // 向下時響 100ms，向上時響 50ms
    digitalWrite(BUZZER_PIN, LOW);       // 關閉蜂鳴器

    digitalWrite(LED_HALL, LOW);         // 熄滅 HALL LED

    // === 顯示動畫 ===
    showWheelAnim = true;                // 啟用動畫
    wheelAnimStartTime = millis();       // 記錄動畫開始時間
    lastHallState = hall;                // 更新上次霍爾狀態

    Serial.println("HALL: Manual control");  // 序列埠輸出訊息
  }

  // ========== 顯示更新 ==========
  if (millis() - lastDisplayTime > 50) {  // 每 50ms 更新一次 OLED
    showStatus();                        // 更新 OLED 顯示
    lastDisplayTime = millis();          // 更新顯示時間
  }

  // ========== 序列埠輸出 ==========
  if (millis() - lastPrintTime > 500) {  // 每 500ms 輸出一次狀態
    Serial.print("Speed: ");             // 輸出速度
    Serial.print(currentSpeed, 1);       // 輸出速度值（1位小數）
    Serial.print(" km/h | Angle: ");     // 分隔
    Serial.print(compAngleX, 1);         // 輸出角度（1位小數）
    Serial.print("° | Wheel: ");         // 分隔
    Serial.print(wheelDown ? "DOWN" : "UP");  // 輸出輔助輪狀態
    Serial.print(" | Mode: ");           // 分隔
    Serial.println(manualMode ? "MANUAL" : "AUTO");  // 輸出控制模式
    lastPrintTime = millis();            // 更新輸出時間
  }

  delay(20);                             // 主迴圈延遲 20ms（避免 CPU 過載）
}

/*
 * ============================================================================
 * 程式功能總結
 * ============================================================================
 * 
 * 【控制模式】
 * 1. 自動模式（預設）：
 *    - 速度 < 10 km/h → 輔助輪自動下降
 *    - 速度 ≥ 10 km/h → 輔助輪自動上升
 * 
 * 2. 手動模式：
 *    - 紅外線遙控 0xFFA857 → 輔助輪下降
 *    - 紅外線遙控 0xFF629D → 輔助輪上升
 *    - 霍爾感測器（PIN 4）磁鐵靠近 → 輔助輪下降
 *    - 霍爾感測器（PIN 4）磁鐵遠離 → 輔助輪上升
 * 
 * 【LED 指示燈】
 * - PIN 5：OLED 模組（測試時亮起）
 * - PIN 6：紅外線模組（接收信號時閃爍）
 * - PIN 7：MPU6050 模組（測試時亮起）
 * - PIN 9：霍爾感測器（檢測磁鐵時閃爍）
 * - PIN 10：繼電器模組（測試時亮起）
 * - PIN 12：蜂鳴器模組（測試時閃爍）
 * 
 * 【接線】
 * - IR 接收器：PIN 3
 * - 測速霍爾：PIN 2（中斷腳）
 * - 控制霍爾：PIN 4
 * - 繼電器：PIN 11
 * - 蜂鳴器：PIN 8
 * 
 * 【函式庫需求】
 * - Wire（內建）
 * - Adafruit_GFX
 * - Adafruit_SSD1306
 * - IRremote（by shirriff/Arduino-IRremote）
 * 
 * ============================================================================
 */
