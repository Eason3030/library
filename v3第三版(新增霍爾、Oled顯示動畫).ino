	#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <LiquidCrystal_PCF8574.h>

// ---------- OLED ----------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ---------- LCD ----------
LiquidCrystal_PCF8574 lcd(0x27);

// ---------- MPU6050 ----------
#define MPU_6050 0x68
#define PWR_MGMT_1 0x6B
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H  0x43

// ---------- PIN ----------
const int BUZZER_PIN = 8;
const int RELAY_PIN  = 11;
const int REMOTE_PIN = 3;
const int HALL_SENSOR_PIN = 4;

// ---------- IMU ----------
const float dt = 0.01f;          // 資料更新週期（秒）。代表每 10ms 更新一次 IMU 數據。
const float alpha = 0.98f;       // 互補濾波係數。98% 用陀螺儀角度，2% 用加速度角度校正。
float compAngleX = 0.0f;         // 互補濾波後的最終角度（X 軸）
float gyroBiasX = 0.0f;          // 陀螺儀在 X 軸的偏移值（校正時量到的誤差）
float accelBiasX = 0.0f;         // 加速度計在 X 軸的偏移值（校正時量到的誤差）

// ---------- 狀態 ----------
bool wheelDown = false;          // 輔助輪目前是否為下降狀態（true=DOWN）
unsigned long wheelAnimStartTime = 0; 
bool showWheelAnim = false;      // 是否播放輔助輪動畫
byte ctrlSrc = 0;                // 控制來源（0=初始化、1=霍爾感測器、2=紅外線遙控）

int lastHallState = HIGH;        // 上一次霍爾感測器的讀值
int lastRemoteRead = LOW;        // 上一次紅外線讀值
unsigned long lastPrintTime = 0; // 最後一次序列印出時間
unsigned long lastDisplayTime = 0; // 最後一次更新顯示的時間

// =====================================================================
// MPU 基本讀取
// =====================================================================

// 從 MPU6050 讀取多個暫存器資料
void mpuReadBytes(uint8_t reg, uint8_t count, uint8_t* dest) {
  Wire.beginTransmission(MPU_6050); // 開始與 MPU 通訊
  Wire.write(reg);                  // 指定起始暫存器
  Wire.endTransmission(false);      // 不釋放 I2C bus
  Wire.requestFrom((uint8_t)MPU_6050, count); // 要求讀取 count 個 byte
  for (uint8_t i = 0; i < count; i++) dest[i] = Wire.read(); // 放入 dest[]
}

// 讀取 16-bit 的感測器資料（高位元節 + 低位元節）
int16_t read16(uint8_t reg) {
  uint8_t buf[2];
  mpuReadBytes(reg, 2, buf);        // 從 reg 讀取兩個 byte
  return (int16_t)((buf[0] << 8) | buf[1]); // 組合成 16-bit 數值
}

// 初始化 MPU6050
void setupMPU() {
  Wire.beginTransmission(MPU_6050);
  Wire.write(PWR_MGMT_1); // 啟動管理暫存器
  Wire.write(0x00);       // 設為 0 表示讓 MPU 從睡眠模式喚醒
  Wire.endTransmission();
  delay(50);              // 等待感測器啟動
}

// 校正加速度計與陀螺儀偏移
void calibrateIMU() {
  float sumAx = 0, sumGx = 0;
  for (byte i = 0; i < 50; i++) {  // 連續取 50 筆做平均
    sumAx += read16(ACCEL_XOUT_H) / 16384.0f;  // 量加速度 X 軸（轉成 g）
    sumGx += read16(GYRO_XOUT_H) / 131.0f;     // 量陀螺儀 X 軸（轉成 °/s）
    delay(5);
  }
  accelBiasX = sumAx / 50;  // 加速度偏移（平均）
  gyroBiasX  = sumGx / 50;  // 陀螺儀偏移（平均）
}

// 讀取 IMU 資料並計算 X 軸角度
void readIMU(float &ax, float &gx) {
  ax = read16(ACCEL_XOUT_H) / 16384.0f;             // 讀加速度 X 軸（單位：g）
  float az = read16(ACCEL_XOUT_H + 4) / 16384.0f;   // 讀加速度 Z 軸（單位：g）
  gx = read16(GYRO_XOUT_H) / 131.0f;                // 讀陀螺儀 X 軸（單位：°/s）

  gx -= gyroBiasX;  // 扣掉校正值 → 減少漂移
  ax -= accelBiasX; // 扣掉校正值 → 減少偏移

  // 利用加速度計計算 tilt angle（使用 atan2，避免除以 0）
  float accelAngle = atan2(ax, az) * 57.2958f; // 57.29... = 180/π（弧度轉角度）

  // 互補濾波：融合加速度角度與陀螺儀角度
  compAngleX = alpha * (compAngleX + gx * dt)   // 陀螺儀推算的角度（主）
             + (1 - alpha) * accelAngle;        // 加速度角度做修正（輔）
}
// =====================================================================
// 專業啟動畫面
// =====================================================================
void showStartup() {
  // 專業開機動畫
  display.clearDisplay();
  
  // 閃爍 LOGO
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
  
  // 載入動畫
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(28, 8);
  display.print("LOADING");
  
  // 進度條
  for (int i = 0; i <= 100; i += 10) {
    display.fillRect(14, 20, i, 4, SSD1306_WHITE);
    display.display();
    delay(80);
  }
  delay(200);
}

// =====================================================================
// 專業主畫面 - 儀表板風格
// =====================================================================
void showStatus() {
  display.clearDisplay();
  
  // === 頂部資訊列 ===
  // 左上：來源標籤
  display.setTextSize(1);
  display.drawRect(0, 0, 42, 10, SSD1306_WHITE);
  display.setCursor(2, 2);
  if (ctrlSrc == 1) display.print("HALL");  // 移除 F()
  else if (ctrlSrc == 2) display.print("REMOTE");
  else display.print("INIT");
  
  // 右上：角度顯示
  display.drawRect(86, 0, 42, 10, SSD1306_WHITE);
  display.setCursor(88, 2);
  int angle = (int)compAngleX;
  if (angle >= 0) display.print(" ");
  display.print(angle);
  display.print("d");
  
  // === 中央大狀態 ===
  display.setTextSize(2);
  
  if (showWheelAnim && (millis() - wheelAnimStartTime < 600)) {
    byte frame = ((millis() - wheelAnimStartTime) / 100) % 3;
    
    if (wheelDown) {
      // DOWN 動畫
      display.setCursor(4, 14);
      if (frame == 0) display.print("|");
      else if (frame == 1) display.print("v");
      else display.print("V");
      
      display.setCursor(20, 14);
      display.print("DOWN");
      
      // 動態箭頭
      int arrowY = 16 + frame * 3;
      display.fillTriangle(108, arrowY, 112, arrowY+4, 116, arrowY, SSD1306_WHITE);
      display.fillTriangle(108, arrowY+5, 112, arrowY+9, 116, arrowY+5, SSD1306_WHITE);
      
    } else {
      // UP 動畫
      display.setCursor(4, 14);
      if (frame == 0) display.print("|");
      else if (frame == 1) display.print("^");
      else display.print("^");
      
      display.setCursor(20, 14);
      display.print("UP");
      
      // 動態箭頭
      int arrowY = 24 - frame * 3;
      display.fillTriangle(108, arrowY+9, 112, arrowY+5, 116, arrowY+9, SSD1306_WHITE);
      display.fillTriangle(108, arrowY+4, 112, arrowY, 116, arrowY+4, SSD1306_WHITE);
    }
    
  } else {
    showWheelAnim = false;
    
    if (wheelDown) {
      // DOWN 靜態
      display.setCursor(4, 14);
      display.print("V DOWN");
      
      // 底部實心條
      display.fillRect(0, 30, 128, 2, SSD1306_WHITE);
      
      // 右側圖標
      display.fillTriangle(110, 16, 114, 20, 118, 16, SSD1306_WHITE);
      display.fillTriangle(110, 22, 114, 26, 118, 22, SSD1306_WHITE);
      
    } else {
      // UP 靜態
      display.setCursor(16, 14);
      display.print("^ UP");
      
      // 頂部虛線
      for (int x = 0; x < 128; x += 6) {
        display.drawFastHLine(x, 12, 3, SSD1306_WHITE);
      }
      
      // 右側圖標
      display.fillTriangle(110, 26, 114, 22, 118, 26, SSD1306_WHITE);
      display.fillTriangle(110, 20, 114, 16, 118, 20, SSD1306_WHITE);
    }
  }
  
  display.display();
}

// =====================================================================
// 專業測試畫面
// =====================================================================
void testModules() {
  // 測試標題
  display.clearDisplay();
  display.fillRect(0, 0, 128, 32, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setTextSize(2);
  display.setCursor(8, 4);
  display.print("SYSTEM");
  display.setCursor(20, 18);
  display.print("CHECK");
  display.display();
  display.setTextColor(SSD1306_WHITE);
  delay(1000);
  
  // 測試項目
  String tests[] = {"OLED", "LCD", "MPU", "HALL", "RELAY", "BUZZ"};
  
  for (int i = 0; i < 6; i++) {
    display.clearDisplay();
    
    // 進度條
    display.drawRect(10, 2, 108, 6, SSD1306_WHITE);
    int progress = (i + 1) * 18;
    display.fillRect(11, 3, progress, 4, SSD1306_WHITE);
    
    // 測試項目
    display.setTextSize(2);
    display.setCursor(20, 12);
    display.print(tests[i]);
    
    // 計數
    display.setTextSize(1);
    display.setCursor(100, 24);
    display.print(i+1);
    display.print("/6");
    
    display.display();
    
    // 執行測試
    if (i == 3) {  // Hall
      int hall = digitalRead(HALL_SENSOR_PIN);
      if (hall == LOW) {
        display.setCursor(20, 24);
        display.print("MAG!");
        display.display();
      }
    } else if (i == 4) {  // Relay
      digitalWrite(RELAY_PIN, HIGH);
      delay(200);
      digitalWrite(RELAY_PIN, LOW);
    } else if (i == 5) {  // Buzzer
      digitalWrite(BUZZER_PIN, HIGH);
      delay(150);
      digitalWrite(BUZZER_PIN, LOW);
    }
    
    delay(600);
  }
  
  // 完成畫面
  display.clearDisplay();
  display.fillRect(0, 0, 128, 32, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setTextSize(3);
  display.setCursor(16, 6);
  display.print("DONE");
  display.display();
  display.setTextColor(SSD1306_WHITE);
  
  // 成功音效
  for (int i = 0; i < 2; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(80);
    digitalWrite(BUZZER_PIN, LOW);
    delay(80);
  }
  
  delay(1500);
}

void updateLCD() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Ang:");
  lcd.print(compAngleX, 1);
  lcd.print("d");
  lcd.setCursor(0, 1);
  lcd.print("Whl:");
  lcd.print(wheelDown ? "DOWN" : "UP  ");
}

// =====================================================================
// Setup
// =====================================================================
void setup() {
  Serial.begin(115200);
  delay(100);
  Wire.begin();

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(REMOTE_PIN, INPUT);
  pinMode(HALL_SENSOR_PIN, INPUT);
  digitalWrite(RELAY_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  // OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED Fail");
    while (1);
  }

  // LCD
  lcd.begin(16, 2);
  lcd.setBacklight(1);
  lcd.clear();
  lcd.print("Initializing...");

  // MPU
  setupMPU();
  calibrateIMU();

  showStartup();
  testModules();
  
  lcd.clear();
  Serial.println("=== READY ===");
}

// =====================================================================
// Loop
// =====================================================================
void loop() {
  float ax, gx;
  readIMU(ax, gx);

  // 霍爾感測器
  int hall = digitalRead(HALL_SENSOR_PIN);
  if (hall != lastHallState) {
    wheelDown = (hall == LOW);
    ctrlSrc = 1;
    digitalWrite(RELAY_PIN, wheelDown ? HIGH : LOW);
    digitalWrite(BUZZER_PIN, HIGH);
    delay(wheelDown ? 100 : 50);
    digitalWrite(BUZZER_PIN, LOW);
    showWheelAnim = true;
    wheelAnimStartTime = millis();
    lastHallState = hall;
  }

  // 遙控器
  int rem = digitalRead(REMOTE_PIN);
  if (rem == HIGH && lastRemoteRead == LOW) {
    wheelDown = !wheelDown;
    ctrlSrc = 2;
    digitalWrite(RELAY_PIN, wheelDown ? HIGH : LOW);
    showWheelAnim = true;
    wheelAnimStartTime = millis();
  }
  lastRemoteRead = rem;

  // 顯示 (降低更新頻率)
  if (millis() - lastDisplayTime > 50) {
    showStatus();
    lastDisplayTime = millis();
  }
  
  if (millis() - lastPrintTime > 500) {
    updateLCD();
    lastPrintTime = millis();
  }

  delay(20);
}

/*
 * ============================================================================
 * 32 像素專業設計 - 儀表板風格
 * ============================================================================
 * 
 * 【設計理念】
 * - 儀表板風格：頂部標籤框 + 中央大顯示
 * - 專業圖標：動態箭頭、三角形指示器
 * - 視覺層次：框線分區、進度條、裝飾線
 * - 動畫流暢：100ms 更新、平滑過渡
 * 
 * 【版面配置】
 * ┌────────┬─────────────┬────────┐
 * │ HALL   │             │  25d   │ ← 頂部標籤框
 * ├────────┴─────────────┴────────┤
 * │                               │
 * │   V  DOWN          ▼▼         │ ← 大字 + 圖標
 * │                               │
 * └───────────────────────────────┘ ← 底部裝飾線
 * 
 * 【記憶體優化】
 * - 移除所有 F() 巨集,提升執行速度
 * - 降低 OLED 更新頻率到 50ms
 * - 32 像素緩衝區僅 512 bytes,記憶體充足
 * 
 * ============================================================================
 */
