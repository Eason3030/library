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
const int RELAY_PIN = 11;
const int REMOTE_PIN = 3;
const int HALL_SENSOR_PIN = 4;
const int HALL_SPEED_PIN = 2;  // 測速用霍爾感測器 (中斷腳)

// ---------- 輪子參數 ----------
const float WHEEL_DIAMETER = 65.0;  // 輪子直徑 65 公分
const float WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * 3.14159;  // 圓周長 = 204.2 公分
const float SPEED_THRESHOLD = 10.0;  // 速度門檻 10 km/h
const unsigned long SPEED_TIMEOUT = 2000;  // 2秒沒信號 = 停止

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
byte ctrlSrc = 0;  // 0=初始 1=霍爾 2=遙控 3=速度自動

int lastHallState = HIGH;
int lastRemoteRead = LOW;
unsigned long lastPrintTime = 0;
unsigned long lastDisplayTime = 0;
bool manualMode = false;  // 手動模式旗標

// ---------- 速度測量 ----------
volatile unsigned long lastSpeedTrigger = 0;
volatile unsigned long timeBetweenTriggers = 0;
volatile bool newSpeedData = false;
float currentSpeed = 0.0;  // km/h
unsigned long lastSpeedUpdate = 0;

// =====================================================================
// 速度測量中斷
// =====================================================================
void speedInterrupt() {
  unsigned long currentTime = micros();
  
  // 防抖動: 至少間隔 50ms
  if (currentTime - lastSpeedTrigger > 50000) {
    timeBetweenTriggers = currentTime - lastSpeedTrigger;
    lastSpeedTrigger = currentTime;
    newSpeedData = true;
  }
}

// =====================================================================
// 計算速度
// =====================================================================
float calculateSpeed() {
  if (timeBetweenTriggers == 0) return 0.0;
  
  float distancePerTrigger = WHEEL_CIRCUMFERENCE;  // 1個磁鐵 = 1圈 = 204.2 cm
  float timeInSeconds = timeBetweenTriggers / 1000000.0;
  float speedCmPerSec = distancePerTrigger / timeInSeconds;
  float speedKmPerHour = speedCmPerSec * 0.036;
  
  return speedKmPerHour;
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
  compAngleX = alpha * (compAngleX + gx * dt) + (1 - alpha) * accelAngle;
}

// =====================================================================
// 專業啟動畫面
// =====================================================================
void showStartup() {
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
// 專業主畫面 - 加入速度顯示
// =====================================================================
void showStatus() {
  display.clearDisplay();
  
  // === 頂部資訊列 ===
  // 左上：來源標籤
  display.setTextSize(1);
  display.drawRect(0, 0, 42, 10, SSD1306_WHITE);
  display.setCursor(2, 2);
  if (ctrlSrc == 1) display.print("HALL");
  else if (ctrlSrc == 2) display.print("REMOTE");
  else if (ctrlSrc == 3) display.print("AUTO");
  else display.print("INIT");
  
  // 中上：速度顯示
  display.drawRect(44, 0, 42, 10, SSD1306_WHITE);
  display.setCursor(46, 2);
  display.print(currentSpeed, 0);
  display.print("km");
  
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
// 專業測試畫面 - 加入 SPEED 測試
// =====================================================================
void testModules() {
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
  
  // 測試項目 (加入 SPEED)
  String tests[] = {"OLED", "LCD", "MPU", "HALL", "SPEED", "RELAY", "BUZZ"};
  
  for (int i = 0; i < 7; i++) {
    display.clearDisplay();
    
    // 進度條
    display.drawRect(10, 2, 108, 6, SSD1306_WHITE);
    int progress = (i + 1) * 15;
    display.fillRect(11, 3, progress, 4, SSD1306_WHITE);
    
    // 測試項目
    display.setTextSize(2);
    display.setCursor(i == 4 ? 12 : 20, 12);  // SPEED 字較長，左移
    display.print(tests[i]);
    
    // 計數
    display.setTextSize(1);
    display.setCursor(100, 24);
    display.print(i+1);
    display.print("/7");
    
    display.display();
    
    // 執行測試
    if (i == 3) {  // Hall
      int hall = digitalRead(HALL_SENSOR_PIN);
      if (hall == LOW) {
        display.setCursor(20, 24);
        display.print("MAG!");
        display.display();
      }
    } else if (i == 4) {  // Speed
      // 檢測測速感測器
      display.setTextSize(1);
      display.setCursor(10, 24);
      display.print("Spin wheel");
      display.display();
      
      // 等待 1 秒看有沒有信號
      unsigned long startWait = millis();
      bool detected = false;
      while (millis() - startWait < 1000) {
        if (newSpeedData) {
          detected = true;
          break;
        }
      }
      
      if (detected) {
        display.setCursor(10, 24);
        display.print("DETECTED!  ");
        display.display();
        newSpeedData = false;
      }
      
    } else if (i == 5) {  // Relay
      digitalWrite(RELAY_PIN, HIGH);
      delay(200);
      digitalWrite(RELAY_PIN, LOW);
    } else if (i == 6) {  // Buzzer
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
  lcd.print("S:");
  lcd.print(currentSpeed, 1);
  lcd.print(" A:");
  lcd.print(compAngleX, 0);
  lcd.print("d");
  
  lcd.setCursor(0, 1);
  if (manualMode) {
    lcd.print("Mode:MANUAL");
  } else {
    lcd.print("Whl:");
    lcd.print(wheelDown ? "DOWN" : "UP  ");
  }
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
  pinMode(HALL_SPEED_PIN, INPUT_PULLUP);  // 測速霍爾
  
  digitalWrite(RELAY_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  // 附加速度測量中斷
  attachInterrupt(digitalPinToInterrupt(HALL_SPEED_PIN), speedInterrupt, FALLING);

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
  Serial.print("Wheel: ");
  Serial.print(WHEEL_DIAMETER);
  Serial.println(" cm");
  Serial.print("Threshold: ");
  Serial.print(SPEED_THRESHOLD);
  Serial.println(" km/h");
}

// =====================================================================
// Loop
// =====================================================================
void loop() {
  float ax, gx;
  readIMU(ax, gx);

  // ========== 速度測量 ==========
  if (newSpeedData) {
    currentSpeed = calculateSpeed();
    newSpeedData = false;
    lastSpeedUpdate = millis();
  }
  
  // 超過 2 秒沒信號 = 停止
  if (millis() - lastSpeedUpdate > SPEED_TIMEOUT) {
    currentSpeed = 0.0;
  }

  // ========== 自動速度控制 (非手動模式) ==========
  if (!manualMode) {
    bool shouldWheelDown = (currentSpeed < SPEED_THRESHOLD);
    
    if (shouldWheelDown != wheelDown) {
      wheelDown = shouldWheelDown;
      ctrlSrc = 3;  // 速度自動控制
      digitalWrite(RELAY_PIN, wheelDown ? HIGH : LOW);
      
      digitalWrite(BUZZER_PIN, HIGH);
      delay(wheelDown ? 100 : 50);
      digitalWrite(BUZZER_PIN, LOW);
      
      showWheelAnim = true;
      wheelAnimStartTime = millis();
      
      Serial.print("AUTO: Speed ");
      Serial.print(currentSpeed);
      Serial.print(" -> Wheel ");
      Serial.println(wheelDown ? "DOWN" : "UP");
    }
  }

  // ========== 霍爾感測器手動控制 ==========
  int hall = digitalRead(HALL_SENSOR_PIN);
  if (hall != lastHallState) {
    wheelDown = (hall == LOW);
    ctrlSrc = 1;
    manualMode = true;  // 進入手動模式
    
    digitalWrite(RELAY_PIN, wheelDown ? HIGH : LOW);
    digitalWrite(BUZZER_PIN, HIGH);
    delay(wheelDown ? 100 : 50);
    digitalWrite(BUZZER_PIN, LOW);
    
    showWheelAnim = true;
    wheelAnimStartTime = millis();
    lastHallState = hall;
    
    Serial.println("HALL: Manual control");
  }

  // ========== 遙控器切換模式 ==========
  int rem = digitalRead(REMOTE_PIN);
  if (rem == HIGH && lastRemoteRead == LOW) {
    manualMode = !manualMode;  // 切換模式
    ctrlSrc = 2;
    
    if (manualMode) {
      // 進入手動：切換輪子狀態
      wheelDown = !wheelDown;
      digitalWrite(RELAY_PIN, wheelDown ? HIGH : LOW);
    }
    
    digitalWrite(BUZZER_PIN, HIGH);
    delay(100);
    digitalWrite(BUZZER_PIN, LOW);
    
    showWheelAnim = true;
    wheelAnimStartTime = millis();
    
    Serial.print("Mode: ");
    Serial.println(manualMode ? "MANUAL" : "AUTO");
  }
  lastRemoteRead = rem;

  // ========== 顯示更新 ==========
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
 * 速度自動控制系統 - 專業儀表板版
 * ============================================================================
 * 
 * 【控制邏輯】
 * 自動模式 (預設):
 * - 速度 < 10 km/h → 輔助輪自動下降
 * - 速度 ≥ 10 km/h → 輔助輪自動上升
 * 
 * 手動模式:
 * - 霍爾感測器 (PIN 4) 或遙控器觸發 → 進入手動
 * - 再按遙控器 → 切回自動
 * 
 * 【接線】
 * - 測速霍爾: PIN 2 (中斷腳,磁鐵經過計速)
 * - 控制霍爾: PIN 4 (手動控制輔助輪)
 * - 遙控器: PIN 3 (切換模式)
 * 
 * 【顯示】
 * OLED 頂部三格:
 * [控制來源] [速度 km] [角度°]
 * 
 * ============================================================================
 */
