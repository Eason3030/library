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

// ---------- LED PINS - 模組指示燈 ----------
const int LED_OLED = 5;        // OLED 模組指示燈
const int LED_LCD = 6;         // LCD 模組指示燈
const int LED_MPU = 7;         // MPU6050 模組指示燈
const int LED_HALL = 9;        // 霍爾感測器指示燈
const int LED_RELAY = 10;      // 繼電器模組指示燈
const int LED_BUZZER = 12;     // 蜂鳴器模組指示燈

// ---------- IMU ----------
const float dt = 0.01f;
const float alpha = 0.98f;
float compAngleX = 0.0f;
float gyroBiasX = 0.0f;
float accelBiasX = 0.0f;

// ---------- 狀態 ----------
bool wheelDown = false;
unsigned long wheelAnimStartTime = 0; 
bool showWheelAnim = false;
byte ctrlSrc = 0;

int lastHallState = HIGH;
int lastRemoteRead = LOW;
unsigned long lastPrintTime = 0;
unsigned long lastDisplayTime = 0;
unsigned long lastLedUpdate = 0;
unsigned long lastActivityBlink = 0;
bool activityLedState = false;

// LED 動畫狀態
unsigned long ledTestTimer = 0;

// =====================================================================
// LED 控制函數 - 模組指示燈
// =====================================================================

// 關閉所有模組指示燈
void allModuleLEDsOff() {
  digitalWrite(LED_OLED, LOW);
  digitalWrite(LED_LCD, LOW);
  digitalWrite(LED_MPU, LOW);
  digitalWrite(LED_HALL, LOW);
  digitalWrite(LED_RELAY, LOW);
  digitalWrite(LED_BUZZER, LOW);
}

// 啟動時所有 LED 流水燈效果
void startupLEDs() {
  int leds[] = {LED_OLED, LED_LCD, LED_MPU, LED_HALL, LED_RELAY, LED_BUZZER};
  
  // 正向流水
  for (int i = 0; i < 6; i++) {
    digitalWrite(leds[i], HIGH);
    delay(100);
  }
  delay(200);
  
  // 反向熄滅
  for (int i = 5; i >= 0; i--) {
    digitalWrite(leds[i], LOW);
    delay(100);
  }
  delay(200);
  
  // 全亮閃爍
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 6; j++) digitalWrite(leds[j], HIGH);
    delay(150);
    allModuleLEDsOff();
    delay(150);
  }
}

// =====================================================================
// MPU 基本讀取
// =====================================================================

void mpuReadBytes(uint8_t reg, uint8_t count, uint8_t* dest) {
  Wire.beginTransmission(MPU_6050);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_6050, count);
  for (uint8_t i = 0; i < count; i++) dest[i] = Wire.read();
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

// =====================================================================
// 專業啟動畫面
// =====================================================================
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
    delay(80);
  }
  delay(200);
}

// =====================================================================
// 專業主畫面
// =====================================================================
void showStatus() {
  display.clearDisplay();
  
  display.setTextSize(1);
  display.drawRect(0, 0, 42, 10, SSD1306_WHITE);
  display.setCursor(2, 2);
  if (ctrlSrc == 1) display.print("HALL");
  else if (ctrlSrc == 2) display.print("REMOTE");
  else display.print("INIT");
  
  display.drawRect(86, 0, 42, 10, SSD1306_WHITE);
  display.setCursor(88, 2);
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

// =====================================================================
// 專業測試畫面 - 配合 LED 指示
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
  
  // 測試項目與對應 LED
  String tests[] = {"OLED", "LCD", "MPU", "HALL", "RELAY", "BUZZ"};
  int leds[] = {LED_OLED, LED_LCD, LED_MPU, LED_HALL, LED_RELAY, LED_BUZZER};
  
  for (int i = 0; i < 6; i++) {
    // === 點亮當前測試模組的 LED ===
    digitalWrite(leds[i], HIGH);
    
    display.clearDisplay();
    
    // 進度條
    display.drawRect(10, 2, 108, 6, SSD1306_WHITE);
    int progress = (i + 1) * 18;
    display.fillRect(11, 3, progress, 4, SSD1306_WHITE);
    
    // 測試項目（大字顯示）
    display.setTextSize(2);
    display.setCursor(20, 12);
    display.print(tests[i]);
    
    // 計數
    display.setTextSize(1);
    display.setCursor(100, 24);
    display.print(i+1);
    display.print("/6");
    
    // 右下角顯示 LED 狀態
    display.setCursor(2, 24);
    display.print("LED:");
    display.print(i+1);
    
    display.display();
    
    // === 執行對應模組測試 ===
    if (i == 0) {  // OLED
      // OLED 本身正在顯示，測試通過
      delay(600);
      
    } else if (i == 1) {  // LCD
      lcd.clear();
      lcd.print("Testing LCD...");
      delay(600);
      
    } else if (i == 2) {  // MPU
      // 讀取一次 IMU 數據
      float ax, gx;
      readIMU(ax, gx);
      display.setTextSize(1);
      display.setCursor(10, 24);
      display.print("Ang:");
      display.print((int)compAngleX);
      display.display();
      delay(600);
      
    } else if (i == 3) {  // Hall
      int hall = digitalRead(HALL_SENSOR_PIN);
      if (hall == LOW) {
        display.setTextSize(1);
        display.setCursor(20, 24);
        display.print("MAG!");
        display.display();
        // LED 快速閃爍表示檢測到磁鐵
        for (int j = 0; j < 3; j++) {
          digitalWrite(LED_HALL, LOW);
          delay(80);
          digitalWrite(LED_HALL, HIGH);
          delay(80);
        }
      }
      delay(600);
      
    } else if (i == 4) {  // Relay
      // 繼電器動作時 LED 同步
      digitalWrite(RELAY_PIN, HIGH);
      delay(200);
      digitalWrite(RELAY_PIN, LOW);
      delay(400);
      
    } else if (i == 5) {  // Buzzer
      // 蜂鳴器響時 LED 閃爍
      for (int j = 0; j < 2; j++) {
        digitalWrite(BUZZER_PIN, HIGH);
        digitalWrite(LED_BUZZER, LOW);
        delay(75);
        digitalWrite(BUZZER_PIN, LOW);
        digitalWrite(LED_BUZZER, HIGH);
        delay(75);
      }
    }
    
    // 保持 LED 亮 200ms 後再進入下一項
    delay(200);
  }
  
  // === 完成畫面：所有 LED 閃爍慶祝 ===
  display.clearDisplay();
  display.fillRect(0, 0, 128, 32, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setTextSize(3);
  display.setCursor(16, 6);
  display.print("DONE");
  display.display();
  display.setTextColor(SSD1306_WHITE);
  
  // 成功音效 + LED 閃爍
  for (int i = 0; i < 3; i++) {
    // 所有 LED 亮
    for (int j = 0; j < 6; j++) digitalWrite(leds[j], HIGH);
    digitalWrite(BUZZER_PIN, HIGH);
    delay(100);
    
    // 所有 LED 滅
    allModuleLEDsOff();
    digitalWrite(BUZZER_PIN, LOW);
    delay(100);
  }
  
  delay(1000);
  allModuleLEDsOff();  // 測試完成，關閉所有指示燈
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
  
  // LED 腳位設定
  pinMode(LED_OLED, OUTPUT);
  pinMode(LED_LCD, OUTPUT);
  pinMode(LED_MPU, OUTPUT);
  pinMode(LED_HALL, OUTPUT);
  pinMode(LED_RELAY, OUTPUT);
  pinMode(LED_BUZZER, OUTPUT);
  
  digitalWrite(RELAY_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  allModuleLEDsOff();  // 初始化時關閉所有 LED

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED Fail");
    while (1);
  }

  lcd.begin(16, 2);
  lcd.setBacklight(1);
  lcd.clear();
  lcd.print("Initializing...");

  setupMPU();
  calibrateIMU();

  startupLEDs();  // LED 啟動動畫
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

  // 更新 LED (運行時可選擇性保持某些 LED 亮起)
  unsigned long now = millis();
  
  // 例如：根據需要在運行時控制 LED
  // 這裡可以加入運行時的 LED 指示邏輯

  // 顯示更新
  if (now - lastDisplayTime > 50) {
    showStatus();
    lastDisplayTime = now;
  }
  
  if (now - lastPrintTime > 500) {
    updateLCD();
    lastPrintTime = now;
  }

  delay(20);
}

/*
 * ============================================================================
 * LED 模組指示燈系統 - 配合 OLED 自檢畫面
 * ============================================================================
 * 
 * 【設計理念】
 * - 每個模組配置獨立的 LED 指示燈
 * - 自檢時 OLED 顯示哪個模組，對應的 LED 就亮起
 * - 視覺化模組測試狀態，直觀看出哪個模組正在檢測
 * 
 * 【LED 配置】
 * Pin 5  → OLED 模組指示燈
 * Pin 6  → LCD 模組指示燈
 * Pin 7  → MPU6050 模組指示燈
 * Pin 9  → 霍爾感測器指示燈
 * Pin 10 → 繼電器模組指示燈
 * Pin 12 → 蜂鳴器模組指示燈
 * 
 * 【自檢流程】
 * 1. 啟動動畫：流水燈效果
 * 2. 逐一測試：
 *    - OLED 顯示 "OLED" → LED_OLED 亮
 *    - OLED 顯示 "LCD"  → LED_LCD 亮
 *    - OLED 顯示 "MPU"  → LED_MPU 亮
 *    - OLED 顯示 "HALL" → LED_HALL 亮 (檢測到磁鐵時快閃)
 *    - OLED 顯示 "RELAY" → LED_RELAY 亮 (繼電器動作同步)
 *    - OLED 顯示 "BUZZ" → LED_BUZZER 閃爍 (蜂鳴器同步)
 * 3. 完成動畫：所有 LED 同步閃爍
 * 
 * 【接線建議】
 * - 使用 5mm LED + 330Ω 限流電阻
 * - 建議使用不同顏色 LED 區分模組：
 *   白色(OLED)、藍色(LCD)、綠色(MPU)
 *   黃色(HALL)、紅色(RELAY)、橙色(BUZZER)
 * 
 * 【視覺效果】
 * - 啟動：正向流水 → 反向熄滅 → 全亮閃爍
 * - 測試：逐個亮起，保持 800ms
 * - 完成：全體同步閃爍 3 次
 * 
 * ============================================================================
 */
