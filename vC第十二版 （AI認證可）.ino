// ============================================================================
// 自行車輔助輪智能控制系統 v4.5 - 往復式致動器版
//
// 【硬體說明】
// - Gearwell 往復式線性致動器(12V 15rpm):像電動推桿,可以上下推動輔助輪
// - 馬達單向持續旋轉 → 凸輪推杆上下往復:馬達一直轉同一個方向,透過機械結構產生上下運動
// - PWM 控制轉速(ON=旋轉, OFF=停止):用 PWM 訊號控制馬達轉速快慢
// - 磁簧開關偵測上/下限「甜蜜點」:用磁鐵感應開關偵測推桿是否到達上限或下限位置
//
// 【控制邏輯】
// 1. 需要上升 → PWM ON → 馬達轉 → 推杆上升 → 上限磁簧觸發 → PWM OFF
// 2. 需要下降 → PWM ON → 馬達轉 → 推杆下降 → 下限磁簧觸發 → PWM OFF
// 3. 冷卻模式 → 5 秒鎖定 → 自動解除
// ============================================================================

// ============================================================================
// 引入需要的函式庫(libraries)
// ============================================================================
#include <Wire.h>                // I2C 通訊函式庫,用來跟感測器溝通
#include <Adafruit_GFX.h>        // Adafruit 圖形函式庫,畫圖用
#include <Adafruit_SSD1306.h>    // OLED 螢幕控制函式庫
#include <IRremote.h>            // 紅外線遙控器函式庫
#include <INA226_WE.h>           // 電流感測器函式庫

// ============================================================================
// OLED 螢幕設定
// ============================================================================
#define SCREEN_WIDTH 128         // 螢幕寬度 128 像素
#define SCREEN_HEIGHT 64         // 螢幕高度 64 像素
#define OLED_RESET -1            // OLED 重置腳位(-1 表示不使用)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);  // 建立 OLED 螢幕物件

// ============================================================================
// 系統設定
// ============================================================================
const bool BUTTON_IS_NC_TYPE = true;        // 按鈕是常閉(NC)型式
const bool IGNORE_CURRENT_SENSOR = true;    // 忽略電流感測器(測試用)

// ============================================================================
// 音符定義(蜂鳴器發聲用)
// ============================================================================
#define NOTE_C5  523    // Do 音符頻率
#define NOTE_E5  659    // Mi 音符頻率
#define NOTE_G5  784    // Sol 音符頻率
#define NOTE_C6  1047   // 高音 Do 音符頻率
#define NOTE_A5  880    // La 音符頻率

// ============================================================================
// MPU6050 陀螺儀/加速度感測器設定
// ============================================================================
#define MPU_6050 0x68          // MPU6050 的 I2C 位址
#define PWR_MGMT_1 0x6B        // 電源管理暫存器位址
#define ACCEL_XOUT_H 0x3B      // X 軸加速度資料暫存器位址
#define GYRO_XOUT_H  0x43      // X 軸陀螺儀資料暫存器位址

// ============================================================================
// Arduino 腳位定義(每個感測器/馬達接在哪個腳位)
// ============================================================================
const byte BUZZER_PIN = 8;              // 蜂鳴器腳位
const byte RELAY_PIN = 11;              // 繼電器腳位(控制方向:HIGH=上升, LOW=下降)
const byte IR_RECEIVE_PIN = 3;          // 紅外線接收器腳位
const byte LIMIT_UP_PIN = 4;            // 上限磁簧開關腳位
const byte LIMIT_DOWN_PIN = 10;         // 下限磁簧開關腳位
const byte HALL_SPEED_PIN = 2;          // 霍爾測速感測器腳位
const byte EMERGENCY_STOP_PIN = 12;     // 緊急停止按鈕腳位
const byte PWM_CONTROL_PIN = 9;         // PWM 控制腳位(控制馬達啟停和速度)
const byte SPEED_POT_PIN = A0;          // 速度旋鈕腳位(類比輸入)

const byte RGB_RED_PIN = 5;             // RGB LED 紅色腳位
const byte RGB_GREEN_PIN = 7;           // RGB LED 綠色腳位
const byte RGB_BLUE_PIN = 6;            // RGB LED 藍色腳位
const bool RGB_COMMON_ANODE = false;    // RGB LED 是共陽極嗎?(false=共陰極)

// ============================================================================
// 系統狀態定義(用列舉讓程式碼更好讀)
// ============================================================================

// 運動狀態:推桿現在在做什麼?
enum MotionState : byte {
  IDLE,           // 停止(不動)
  MOVING_UP,      // 正在上升
  MOVING_DOWN     // 正在下降
};

// 系統狀態:整個系統現在的情況
enum SystemState : byte {
  RUNNING,        // 正常運作中
  COOLING_DOWN,   // 冷卻中(發生緊急狀況後需要休息 5 秒)
  EMERGENCY       // 緊急狀態(嚴重錯誤)
};

// RGB LED 顏色定義
enum RGBColor : byte {
  RGB_OFF,      // 關閉(黑色)
  RGB_GREEN,    // 綠色(正常)
  RGB_YELLOW,   // 黃色(下降中)
  RGB_BLUE,     // 藍色(傾斜警告)
  RGB_RED,      // 紅色(緊急/冷卻)
  RGB_PURPLE    // 紫色(其他)
};

// ============================================================================
// 全域變數(整個程式都可以使用的變數)
// ============================================================================

// 電流感測器物件
INA226_WE ina226;
const float MAX_CURRENT_A = 5.0;    // 最大允許電流 5 安培
float current_A = 0.0;              // 目前電流值

// 系統狀態變數
MotionState motionState = IDLE;                 // 推桿運動狀態(初始值:停止)
SystemState currentSystemState = RUNNING;       // 系統狀態(初始值:正常運作)

// ========== 速度測量相關變數 ==========
#define WHEEL_CIRCUMFERENCE 204.2               // 輪子周長(公分)
const float SPEED_THRESHOLD = 10.0;             // 速度門檻 10 km/h
const uint16_t SPEED_TIMEOUT = 2000;            // 速度超時時間 2 秒
volatile uint32_t lastSpeedTrigger = 0;         // 上次霍爾感測器觸發時間
volatile uint32_t timeBetweenTriggers = 0;      // 兩次觸發之間的時間差
volatile bool newSpeedData = false;             // 有新的速度資料嗎?
float currentSpeed = 0.0;                       // 目前速度(km/h)
uint32_t lastSpeedUpdate = 0;                   // 上次更新速度的時間

// ========== IMU 陀螺儀/加速度計資料 ==========
float compAngleX = 0.0f;        // 互補濾波後的 X 軸角度(車身傾斜角度)
float gyroBiasX = 0.0f;         // 陀螺儀 X 軸偏移量(校正用)
float accelBiasX = 0.0f;        // 加速度計 X 軸偏移量(校正用)

// ========== 傾斜保護相關變數 ==========
const float SAFE_ANGLE = 20.0f;             // 安全角度門檻 20 度
const float TILT_SAFE = 12.0f;              // 回到安全角度 12 度
const float EMERGENCY_ANGLE = 35.0f;        // 緊急角度 35 度
const uint16_t TILT_TRIGGER_TIME = 500;     // 傾斜觸發時間 0.5 秒
const uint16_t AUTO_UP_DELAY = 3000;        // 自動上升延遲 3 秒
uint32_t tiltStartTime = 0;                 // 開始傾斜的時間
bool tiltWarning = false;                   // 傾斜警告旗標
uint32_t safeStartTime = 0;                 // 回到安全角度的時間

// ========== 控制變數 ==========
bool wheelDown = false;         // 目標狀態:輔助輪要放下來嗎?(false=收起,true=放下)
bool manualMode = false;        // 手動模式開啟嗎?
byte ctrlSrc = 0;               // 控制來源(0=無,1=磁簧,2=紅外線,3=自動,4=傾斜)

// ========== 限位鎖定變數(防止推桿過度運動) ==========
bool limitUpLocked = false;     // 上限已經觸發而被鎖定
bool limitDownLocked = false;   // 下限已經觸發而被鎖定

// ========== 時間控制變數 ==========
uint32_t haltTime = 0;                      // 停止時間記錄
const uint16_t RESTART_DELAY_MS = 5000;     // 重啟延遲 5 秒
uint32_t lastPrintTime = 0;                 // 上次印出資料的時間
uint32_t lastDisplayTime = 0;               // 上次更新螢幕的時間
uint32_t lastIRCode = 0;                    // 上次紅外線訊號時間
const uint16_t IR_DELAY = 500;              // 紅外線防抖動延遲 0.5 秒

// ========== 非阻塞音效變數(讓蜂鳴器不會卡住程式) ==========
uint32_t beepEndTime = 0;       // 蜂鳴結束時間
bool isBeeping = false;         // 正在發出聲音嗎?

// ========== 看門狗變數(防止程式當機) ==========
uint32_t lastLoopTime = 0;                  // 上次執行主迴圈的時間
const uint32_t WATCHDOG_TIMEOUT = 2000;     // 看門狗超時時間 2 秒

// ========== PWM 控制變數 ==========
int targetPWM = 0;              // 目標 PWM 值(0-255,控制馬達轉速)

// ============================================================================
// RGB LED 控制函式
// ============================================================================

/**
 * 設定 RGB LED 的顏色
 * @param r 紅色亮度(0-255)
 * @param g 綠色亮度(0-255)
 * @param b 藍色亮度(0-255)
 */
void setRGB(byte r, byte g, byte b) {
  if (RGB_COMMON_ANODE) {
    // 如果是共陽極,需要反轉數值(255-亮度)
    analogWrite(RGB_RED_PIN, 255-r);
    analogWrite(RGB_GREEN_PIN, 255-g);
    analogWrite(RGB_BLUE_PIN, 255-b);
  } else {
    // 共陰極直接輸出
    analogWrite(RGB_RED_PIN, r);
    analogWrite(RGB_GREEN_PIN, g);
    analogWrite(RGB_BLUE_PIN, b);
  }
}

/**
 * 用預設的顏色設定 RGB LED
 * @param c 顏色代碼(RGB_OFF, RGB_GREEN 等)
 */
void setRGBColor(RGBColor c) {
  switch(c) {
    case RGB_OFF: setRGB(0,0,0); break;           // 關閉
    case RGB_GREEN: setRGB(0,255,0); break;       // 綠色
    case RGB_YELLOW: setRGB(255,255,0); break;    // 黃色
    case RGB_BLUE: setRGB(0,0,255); break;        // 藍色
    case RGB_RED: setRGB(255,0,0); break;         // 紅色
    case RGB_PURPLE: setRGB(128,0,255); break;    // 紫色
  }
}

/**
 * 根據系統狀態決定 RGB LED 該顯示什麼顏色
 * @return 應該顯示的顏色
 */
RGBColor decideRGB() {
  // 優先順序:緊急/冷卻 > 傾斜警告 > 運動狀態
  if (currentSystemState == COOLING_DOWN || currentSystemState == EMERGENCY) 
    return RGB_RED;       // 紅色:有問題
  if (tiltWarning) 
    return RGB_BLUE;      // 藍色:傾斜警告
  if (motionState == MOVING_DOWN) 
    return RGB_YELLOW;    // 黃色:正在下降
  if (motionState == MOVING_UP) 
    return RGB_GREEN;     // 綠色:正在上升
  if (wheelDown) 
    return RGB_YELLOW;    // 黃色:輔助輪在下面
  return RGB_GREEN;       // 綠色:正常狀態
}

// ============================================================================
// 非阻塞音效(不使用 delay,避免卡住程式)
// ============================================================================

/**
 * 開始發出聲音
 * @param freq 頻率(Hz,音調高低)
 * @param duration 持續時間(毫秒)
 */
void beepStart(uint16_t freq, uint16_t duration) {
  tone(BUZZER_PIN, freq);                 // 開始發出指定頻率的聲音
  beepEndTime = millis() + duration;      // 計算結束時間
  isBeeping = true;                       // 標記正在發聲
}

/**
 * 更新音效狀態(在主迴圈中持續呼叫)
 * 時間到了就自動停止聲音
 */
void beepUpdate() {
  if (isBeeping && millis() >= beepEndTime) {
    noTone(BUZZER_PIN);     // 停止聲音
    isBeeping = false;       // 標記停止發聲
  }
}

// ============================================================================
// 速度計算(利用霍爾感測器)
// ============================================================================

/**
 * 霍爾感測器中斷服務程式
 * 當感測器偵測到磁鐵經過時自動執行
 */
void speedInterrupt() {
  uint32_t t = micros();    // 取得目前時間(微秒)
  
  // 防抖動:如果距離上次觸發不到 50 毫秒就忽略
  if (t - lastSpeedTrigger > 50000) {
    timeBetweenTriggers = t - lastSpeedTrigger;   // 計算時間差
    lastSpeedTrigger = t;                         // 記錄這次觸發時間
    newSpeedData = true;                          // 標記有新資料
  }
}

/**
 * 計算目前速度
 * @return 速度(km/h)
 */
float calculateSpeed() {
  if (timeBetweenTriggers == 0) return 0.0;   // 沒有資料就回傳 0
  
  float timeInSec = timeBetweenTriggers/1000000.0;    // 轉換成秒
  // 速度 = 輪子周長 / 時間 * 0.036 (轉換成 km/h)
  return (WHEEL_CIRCUMFERENCE/timeInSec)*0.036;
}

// ============================================================================
// MPU6050 陀螺儀/加速度計函式
// ============================================================================

/**
 * 從 MPU6050 讀取 16 位元資料
 * @param reg 暫存器位址
 * @return 16 位元資料
 */
int16_t read16(byte reg) {
  Wire.beginTransmission(MPU_6050);     // 開始 I2C 通訊
  Wire.write(reg);                       // 寫入暫存器位址
  Wire.endTransmission(false);           // 結束傳送但保持連線
  Wire.requestFrom((byte)MPU_6050,(byte)2);  // 要求讀取 2 個位元組
  return (Wire.read()<<8)|Wire.read();   // 組合成 16 位元資料
}

/**
 * 初始化 MPU6050
 */
void setupMPU() {
  Wire.beginTransmission(MPU_6050);
  Wire.write(PWR_MGMT_1);     // 寫入電源管理暫存器
  Wire.write(0x00);           // 設定為正常模式(喚醒感測器)
  Wire.endTransmission();
  delay(50);                  // 等待感測器穩定
}

/**
 * 校正 IMU(消除誤差)
 * 需要在平放靜止狀態下執行
 */
void calibrateIMU() {
  float sumAx=0, sumGx=0;
  
  // 讀取 50 次取平均
  for(byte i=0; i<50; i++) {
    sumAx += read16(ACCEL_XOUT_H)/16384.0f;   // 加速度原始值轉 g
    sumGx += read16(GYRO_XOUT_H)/131.0f;      // 陀螺儀原始值轉 度/秒
    delay(5);
  }
  
  accelBiasX = sumAx/50;    // 計算加速度偏移量
  gyroBiasX = sumGx/50;     // 計算陀螺儀偏移量
}

/**
 * 讀取 IMU 資料並計算傾斜角度
 * @param ax 回傳 X 軸加速度
 * @param gx 回傳 X 軸角速度
 */
void readIMU(float &ax, float &gx) {
  // 讀取原始資料
  ax = read16(ACCEL_XOUT_H)/16384.0f;         // X 軸加速度
  float az = read16(ACCEL_XOUT_H+4)/16384.0f; // Z 軸加速度
  gx = read16(GYRO_XOUT_H)/131.0f;            // X 軸角速度
  
  // 減去偏移量(校正)
  gx -= gyroBiasX;
  ax -= accelBiasX;
  
  // 用加速度計算角度
  float accelAngle = atan2(ax,az)*57.2958f;   // 反正切轉角度(弧度轉度)
  
  // 互補濾波:結合陀螺儀和加速度計
  // 98% 信任陀螺儀積分,2% 信任加速度計(消除漂移)
  compAngleX = 0.98*(compAngleX+gx*0.01)+0.02*accelAngle;
}

// ============================================================================
// OLED 螢幕顯示函式
// ============================================================================

/**
 * 顯示開機畫面
 */
void showStartupPro() {
  display.clearDisplay();           // 清除螢幕
  display.setTextSize(3);          // 設定大字體
  display.setTextColor(SSD1306_WHITE);  // 白色文字
  display.setCursor(15,10);        // 設定游標位置
  display.print(F("BIKE"));        // 顯示 BIKE
  display.setCursor(10,40);
  display.setTextSize(2);
  display.print(F("v4.5"));        // 顯示版本號
  display.display();               // 更新螢幕
  delay(1000);                     // 等待 1 秒
}

/**
 * 繪製主要儀表板畫面
 */
void drawDashboard() {
  display.clearDisplay();
  
  // ========== 超大速度顯示 ==========
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(4);           // 超大字體
  
  int speedInt = (int)currentSpeed;  // 取整數部分
  int xPos = (speedInt < 10) ? 40 : 10;  // 個位數置中,兩位數靠左
  
  display.setCursor(xPos, 5);
  display.print(speedInt);          // 顯示速度數字
  
  // ========== 單位 km/h ==========
  display.setTextSize(2);
  display.setCursor(85, 15);
  display.print(F("km"));
  display.setCursor(85, 30);
  display.print(F("/h"));
  
  // ========== 底部資訊 ==========
  display.setTextSize(1);
  display.setCursor(2, 54);
  
  // 顯示角度
  display.print(F("A:"));
  int angleInt = (int)abs(compAngleX);  // 絕對值
  if(angleInt < 10) display.print(F(" "));  // 個位數前面加空格對齊
  display.print(angleInt);
  display.print(F("'"));
  
  // 顯示傾斜方向
  if(compAngleX > 5) display.print(F("R"));       // 右傾
  else if(compAngleX < -5) display.print(F("L")); // 左傾
  else display.print(F("-"));                     // 平衡
  
  display.print(F(" |"));
  
  // ========== 運動狀態圖示 ==========
  if(motionState == MOVING_UP) {
    display.print(F(" ^UP"));       // 上升
  } else if(motionState == MOVING_DOWN) {
    display.print(F(" vDN"));       // 下降
  } else {
    display.print(F(" --"));        // 停止
  }
  
  // ========== 警告訊息 ==========
  if(tiltWarning) {
    display.setCursor(100, 54);
    display.print(F("TILT"));       // 傾斜警告
  } else if(currentSystemState != RUNNING) {
    display.setCursor(100, 54);
    display.print(F("LOCK"));       // 鎖定中
  }
  
  display.display();    // 更新螢幕
}

/**
 * 繪製詳細資訊畫面
 */
void drawDetailScreen() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  
  // ========== 速度(含小數點) ==========
  display.setTextSize(3);
  display.setCursor(5, 5);
  display.print(currentSpeed, 1);   // 顯示一位小數
  display.setTextSize(1);
  display.setCursor(90, 15);
  display.print(F("km/h"));
  
  // ========== 角度(含小數點) ==========
  display.setTextSize(2);
  display.setCursor(5, 32);
  display.print(F("A:"));
  display.print(compAngleX, 1);     // 顯示一位小數
  
  // ========== 狀態文字 ==========
  display.setTextSize(1);
  display.setCursor(5, 54);
  
  if(motionState == MOVING_UP) 
    display.print(F("MOVING UP  "));
  else if(motionState == MOVING_DOWN) 
    display.print(F("MOVING DOWN"));
  else 
    display.print(F("STOPPED    "));
  
  // ========== PWM 值 ==========
  display.setCursor(90, 54);
  display.print(targetPWM);
  
  display.display();    // 更新螢幕
}

// ============================================================================
// 紅外線遙控器處理
// ============================================================================

/**
 * 處理紅外線遙控器指令
 * @param code 紅外線代碼
 */
void handleIRCommand(uint32_t code) {
  // 防抖動:如果距離上次按壓不到 0.5 秒就忽略
  if(millis() - lastIRCode > IR_DELAY) {
    lastIRCode = millis();
    
    // 解除限位鎖定(按遙控器可以解除鎖定狀態)
    if(limitUpLocked || limitDownLocked) {
      limitUpLocked = false;
      limitDownLocked = false;
      Serial.println(F("IR: Unlock limits"));
    }
    
    // 切換目標狀態(上 ↔ 下)
    wheelDown = !wheelDown;
    manualMode = true;      // 進入手動模式
    ctrlSrc = 2;            // 標記控制來源為紅外線
    
    beepStart(NOTE_A5, 50); // 發出確認音
    Serial.print(F("IR: Target "));
    Serial.println(wheelDown ? F("DOWN") : F("UP"));
  }
}

// ============================================================================
// 運動控制核心(往復式致動器專用)
// ============================================================================

/**
 * 執行推桿運動控制的主要函式
 * 這是整個系統最重要的函式!
 */
void executeMotion() {
  // ========== 讀取磁簧開關狀態 ==========
  // 磁簧開關被觸發時會變成 LOW(接地)
  bool limitUpTriggered = (digitalRead(LIMIT_UP_PIN) == LOW);      // 上限被觸發了嗎?
  bool limitDownTriggered = (digitalRead(LIMIT_DOWN_PIN) == LOW);  // 下限被觸發了嗎?
  
  // ========== 磁簧觸發檢查(只有在運動中才鎖定) ==========
  // 上限觸發:如果正在上升且碰到上限磁簧,就鎖定並停止
  if(limitUpTriggered && !limitUpLocked && motionState == MOVING_UP) {
    limitUpLocked = true;       // 鎖定上限
    motionState = IDLE;         // 改為停止狀態
    ctrlSrc = 1;                // 標記控制來源為磁簧開關
    beepStart(NOTE_C6, 50);     // 發出高音確認
    Serial.println(F("<<< UP LIMIT"));
  }
  
  // 下限觸發:如果正在下降且碰到下限磁簧,就鎖定並停止
  if(limitDownTriggered && !limitDownLocked && motionState == MOVING_DOWN) {
    limitDownLocked = true;     // 鎖定下限
    motionState = IDLE;         // 改為停止狀態
    ctrlSrc = 1;                // 標記控制來源為磁簧開關
    beepStart(NOTE_G5, 50);     // 發出中音確認
    Serial.println(F("<<< DOWN LIMIT"));
  }
  
  // ========== 狀態機(State Machine) ==========
  // 根據目前狀態決定下一步要做什麼
  switch(motionState) {
    
    // ========== 狀態 1: IDLE 停止狀態 ==========
    case IDLE:
      // 只有在系統正常運作時才允許啟動
      if(currentSystemState == RUNNING) {
        
        // 條件:需要下降且還沒到下限
        if(wheelDown && !limitDownLocked) {
          digitalWrite(RELAY_PIN, LOW);   // 設定繼電器為下降方向
          motionState = MOVING_DOWN;      // 改變狀態為下降中
          limitDownLocked = false;        // 解除鎖定,允許運動
          Serial.println(F(">>> START DOWN"));
        }
        // 條件:需要上升且還沒到上限
        else if(!wheelDown && !limitUpLocked) {
          digitalWrite(RELAY_PIN, HIGH);  // 設定繼電器為上升方向
          motionState = MOVING_UP;        // 改變狀態為上升中
          limitUpLocked = false;          // 解除鎖定,允許運動
          Serial.println(F(">>> START UP"));
        }
      }
      targetPWM = 0;    // 停止狀態 PWM 為 0(不轉)
      break;
      
    // ========== 狀態 2: MOVING_UP 上升中 ==========
    case MOVING_UP:
      // 檢查是否碰到上限
      if(limitUpLocked) {
        motionState = IDLE;         // 碰到了就停止
        targetPWM = 0;
        Serial.println(F("<<< UP COMPLETE"));
      }
      // 檢查目標是否改變(想要下降了)
      else if(wheelDown) {
        motionState = IDLE;         // 先停止
        targetPWM = 0;
        Serial.println(F("<<< UP ABORT"));
      }
      // 繼續上升:讀取速度旋鈕決定轉速
      else {
        int potValue = analogRead(SPEED_POT_PIN);           // 讀取旋鈕(0-1023)
        targetPWM = map(potValue, 0, 1023, 120, 255);      // 對應到 PWM(120-255)
      }
      break;
      
    // ========== 狀態 3: MOVING_DOWN 下降中 ==========
    case MOVING_DOWN:
      // 檢查是否碰到下限
      if(limitDownLocked) {
        motionState = IDLE;         // 碰到了就停止
        targetPWM = 0;
        Serial.println(F("<<< DOWN COMPLETE"));
      }
      // 檢查目標是否改變(想要上升了)
      else if(!wheelDown) {
        motionState = IDLE;         // 先停止
        targetPWM = 0;
        Serial.println(F("<<< DOWN ABORT"));
      }
      // 繼續下降:讀取速度旋鈕決定轉速
      else {
        int potValue = analogRead(SPEED_POT_PIN);           // 讀取旋鈕(0-1023)
        targetPWM = map(potValue, 0, 1023, 120, 255);      // 對應到 PWM(120-255)
      }
      break;
  }
  
  // ========== 緊急停止保護 ==========
  // 如果系統不是正常狀態,強制停止
  if(currentSystemState != RUNNING) {
    motionState = IDLE;
    targetPWM = 0;
  }
  
  // ========== 輸出 PWM 到馬達 ==========
  // 這是唯一真正控制馬達轉動的地方
  analogWrite(PWM_CONTROL_PIN, targetPWM);
}

// ============================================================================
// Setup 初始化函式(Arduino 開機時執行一次)
// ============================================================================
void setup() {
  // ========== 序列埠初始化 ==========
  Serial.begin(115200);     // 開啟序列埠,鮑率 115200
  delay(100);
  
  // ========== I2C 初始化 ==========
  Wire.begin();             // 啟動 I2C 通訊
  Wire.setClock(100000);    // 設定 I2C 時脈 100kHz
  
  // ========== 腳位模式設定 ==========
  pinMode(BUZZER_PIN, OUTPUT);              // 蜂鳴器:輸出
  pinMode(RELAY_PIN, OUTPUT);               // 繼電器:輸出
  pinMode(LIMIT_UP_PIN, INPUT_PULLUP);      // 上限開關:輸入+內部上拉
  pinMode(LIMIT_DOWN_PIN, INPUT_PULLUP);    // 下限開關:輸入+內部上拉
  pinMode(HALL_SPEED_PIN, INPUT_PULLUP);    // 霍爾感測器:輸入+內部上拉
  pinMode(EMERGENCY_STOP_PIN, INPUT_PULLUP);// 緊急按鈕:輸入+內部上拉
  pinMode(PWM_CONTROL_PIN, OUTPUT);         // PWM 控制:輸出
  pinMode(RGB_RED_PIN, OUTPUT);             // RGB 紅:輸出
  pinMode(RGB_GREEN_PIN, OUTPUT);           // RGB 綠:輸出
  pinMode(RGB_BLUE_PIN, OUTPUT);            // RGB 藍:輸出
  
  // ========== 初始狀態設定 ==========
  digitalWrite(RELAY_PIN, LOW);             // 繼電器初始:LOW(下降方向)
  analogWrite(PWM_CONTROL_PIN, 0);          // PWM 初始:0(停止)
  setRGBColor(RGB_OFF);                     // RGB LED 關閉
  
  // ========== 啟動紅外線接收器 ==========
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
  
  // ========== 設定霍爾感測器中斷 ==========
  // 當 HALL_SPEED_PIN 從 HIGH 變 LOW 時,自動執行 speedInterrupt 函式
  attachInterrupt(digitalPinToInterrupt(HALL_SPEED_PIN), speedInterrupt, FALLING);
  
  // ========== OLED 螢幕初始化 ==========
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("OLED Fail"));  // 如果初始化失敗
    while(1);                         // 卡在這裡(無限迴圈)
  }
  
  // ========== 感測器初始化 ==========
  ina226.init();        // 初始化電流感測器
  setupMPU();           // 初始化陀螺儀/加速度計
  calibrateIMU();       // 校正 IMU(需要靜止)
  
  // ========== 開機畫面與音效 ==========
  showStartupPro();             // 顯示開機畫面
  beepStart(NOTE_C6, 100);      // 發出開機音效
  
  // ========== 開機訊息 ==========
  Serial.println(F("============================"));
  Serial.println(F("  BIKE ASSIST v4.5 READY"));
  Serial.println(F("  往復式致動器專用版"));
  Serial.println(F("============================"));
  
  lastLoopTime = millis();      // 記錄啟動時間(看門狗用)
}

// ============================================================================
// Loop 主迴圈(Arduino 會不斷重複執行)
// ============================================================================
void loop() {
  uint32_t currentMillis = millis();    // 取得目前時間(毫秒)
  
  // ========== 看門狗(防當機核心機制) ==========
  // 如果主迴圈超過 2 秒沒執行,代表可能當機了
  if(currentMillis - lastLoopTime > WATCHDOG_TIMEOUT) {
    Serial.println(F(""));
    Serial.println(F("!!!!!!!!!!!!!!!!!!!!!!!!"));
    Serial.println(F("!!! WATCHDOG RESET !!!"));
    Serial.println(F("!!!!!!!!!!!!!!!!!!!!!!!!"));
    
    // ========== 強制回到安全狀態 ==========
    motionState = IDLE;                     // 停止運動
    currentSystemState = RUNNING;           // 恢復正常狀態
    targetPWM = 0;                          // PWM 歸零
    analogWrite(PWM_CONTROL_PIN, 0);        // 馬達停止
    digitalWrite(RELAY_PIN, LOW);           // 繼電器重置
    
    // 重置控制變數
    tiltWarning = false;
    manualMode = false;
    
    // 發出警告音
    beepStart(NOTE_A5, 300);
  }
  lastLoopTime = currentMillis;     // 更新看門狗計時器
  
  // ========== 非阻塞音效更新 ==========
  beepUpdate();     // 檢查是否該停止蜂鳴器
  
  // ========== 讀取感測器資料 ==========
  float ax, gx;
  readIMU(ax, gx);      // 讀取陀螺儀/加速度計(更新 compAngleX)
  
  // 讀取電流
  current_A = ina226.getCurrent_A();
  if(isnan(current_A)) current_A = 0.0;     // 如果讀取失敗就設為 0
  
  // 更新速度
  if(newSpeedData) {
    currentSpeed = calculateSpeed();        // 計算新的速度
    newSpeedData = false;
    lastSpeedUpdate = millis();
  }
  
  // 速度超時檢查(2 秒沒更新就歸零)
  if(millis() - lastSpeedUpdate > SPEED_TIMEOUT) {
    currentSpeed = 0.0;
  }
  
  // ========== 紅外線遙控器檢查 ==========
  if(IrReceiver.decode()) {
    handleIRCommand(IrReceiver.decodedIRData.decodedRawData);   // 處理紅外線指令
    IrReceiver.resume();                                        // 準備接收下一個訊號
  }
  
  // ========== 緊急停止檢查 ==========
  bool emergencyStop = false;
  // 如果電流過大且沒有忽略電流感測器
  if(!IGNORE_CURRENT_SENSOR && current_A > MAX_CURRENT_A) {
    emergencyStop = true;
  }
  
  // 觸發緊急停止
  if(emergencyStop && currentSystemState == RUNNING) {
    currentSystemState = COOLING_DOWN;      // 進入冷卻狀態
    haltTime = currentMillis;               // 記錄停止時間
    motionState = IDLE;                     // 停止運動
    targetPWM = 0;
    analogWrite(PWM_CONTROL_PIN, 0);        // 馬達停止
    beepStart(NOTE_C6, 200);                // 發出警告音
    Serial.println(F("EMERGENCY!"));
  }
  
  // ========== 冷卻恢復機制 ==========
  if(currentSystemState == COOLING_DOWN) {
    // 如果已經冷卻 5 秒,恢復正常
    if(currentMillis - haltTime >= RESTART_DELAY_MS) {
      currentSystemState = RUNNING;         // 恢復正常狀態
      limitUpLocked = false;                // 解除限位鎖定
      limitDownLocked = false;
      manualMode = false;                   // 離開手動模式
      Serial.println(F("<<< RECOVERED >>>"));
      beepStart(NOTE_C5, 100);              // 發出恢復音效
    }
  }
  
  // ========== 傾斜保護系統 ==========
  if(currentSystemState == RUNNING) {
    
    // 【等級 1】緊急傾斜(超過 35 度)
    if(abs(compAngleX) > EMERGENCY_ANGLE) {
      currentSystemState = COOLING_DOWN;    // 進入冷卻狀態
      haltTime = currentMillis;
      motionState = IDLE;
      targetPWM = 0;
      analogWrite(PWM_CONTROL_PIN, 0);
      wheelDown = true;                     // 強制要求下降(放下輔助輪)
      beepStart(NOTE_C6, 300);              // 發出警告音
      Serial.println(F("!!! EMERGENCY TILT !!!"));
    }
    
    // 【等級 2】傾斜警告(超過 20 度)
    else if(abs(compAngleX) > SAFE_ANGLE) {
      // 記錄開始傾斜的時間
      if(tiltStartTime == 0) tiltStartTime = millis();
      
      // 如果傾斜超過 0.5 秒,觸發警告
      if(!tiltWarning && (millis() - tiltStartTime >= TILT_TRIGGER_TIME)) {
        tiltWarning = true;                 // 啟動傾斜警告
        wheelDown = true;                   // 要求下降輔助輪
        manualMode = false;                 // 離開手動模式
        ctrlSrc = 4;                        // 標記控制來源為傾斜保護
        limitDownLocked = false;            // 解除鎖定,允許下降
        Serial.println(F(">>> TILT WARN"));
      }
    } 
    // 【等級 3】角度正常(小於 20 度)
    else {
      tiltStartTime = 0;                    // 重置傾斜計時
      
      // 如果之前有傾斜警告且現在角度小於 12 度
      if(tiltWarning && abs(compAngleX) < TILT_SAFE) {
        // 記錄回到安全角度的時間
        if(safeStartTime == 0) safeStartTime = millis();
        
        // 如果安全狀態維持 3 秒,解除警告
        if(millis() - safeStartTime >= AUTO_UP_DELAY) {
          tiltWarning = false;              // 解除傾斜警告
          if(!manualMode) {
            wheelDown = false;              // 允許收回輔助輪
            limitUpLocked = false;          // 解除鎖定,允許上升
          }
          Serial.println(F("<<< TILT OK"));
          safeStartTime = 0;
        }
      } else {
        safeStartTime = 0;                  // 重置安全計時
      }
    }
  }
  
  // ========== 自動速度控制 ==========
  // 條件:系統正常 + 非手動模式 + 無傾斜警告
  if(currentSystemState == RUNNING && !manualMode && !tiltWarning) {
    
    // 速度太慢(< 10 km/h)且輔助輪還在上面 → 放下輔助輪
    if(currentSpeed < SPEED_THRESHOLD && !wheelDown && !limitDownLocked) {
      wheelDown = true;                     // 設定目標為下降
      ctrlSrc = 3;                          // 標記控制來源為自動
      limitDownLocked = false;              // 解除鎖定
      Serial.println(F("Auto: DOWN (speed < 10)"));
    } 
    // 速度夠快(>= 10 km/h)且輔助輪在下面 → 收起輔助輪
    else if(currentSpeed >= SPEED_THRESHOLD && wheelDown && !limitUpLocked) {
      wheelDown = false;                    // 設定目標為上升
      ctrlSrc = 3;                          // 標記控制來源為自動
      limitUpLocked = false;                // 解除鎖定
      Serial.println(F("Auto: UP (speed >= 10)"));
    }
  }
  
  // ========== 執行運動控制(核心函式) ==========
  executeMotion();
  
  // ========== RGB LED 狀態顯示 ==========
  setRGBColor(decideRGB());     // 根據系統狀態決定 LED 顏色
  
  // ========== OLED 螢幕更新(每 0.2 秒更新一次) ==========
  if(millis() - lastDisplayTime > 200) {
    lastDisplayTime = millis();
    
    // 每 5 秒切換一次顯示模式
    int displayMode = (millis() / 5000) % 2;
    if(displayMode == 0) 
      drawDashboard();          // 模式 0:儀表板
    else 
      drawDetailScreen();       // 模式 1:詳細資訊
  }
  
  // ========== 序列埠輸出(每 1 秒輸出一次,方便除錯) ==========
  if(millis() - lastPrintTime > 1000) {
    lastPrintTime = millis();
    
    // ===== 系統狀態 =====
    Serial.print(F("Sys:"));
    if(currentSystemState == RUNNING) 
      Serial.print(F("RUN"));           // 正常運作
    else if(currentSystemState == COOLING_DOWN) 
      Serial.print(F("COOL"));          // 冷卻中
    else 
      Serial.print(F("EMRG"));          // 緊急
    
    // ===== 運動狀態 =====
    Serial.print(F(" |Motion:"));
    if(motionState == MOVING_UP) 
      Serial.print(F("^UP^"));          // 上升中
    else if(motionState == MOVING_DOWN) 
      Serial.print(F("vDNv"));          // 下降中
    else 
      Serial.print(F("IDLE"));          // 停止
    
    // ===== 目標狀態 =====
    Serial.print(F(" |Target:"));
    Serial.print(wheelDown ? F("DN") : F("UP"));
    
    // ===== 磁簧開關狀態 =====
    Serial.print(F(" |LimUP:"));
    if(limitUpLocked) 
      Serial.print(F("LOCK"));          // 上限鎖定
    else 
      Serial.print(digitalRead(LIMIT_UP_PIN) == LOW ? F("ON") : F("--"));
    
    Serial.print(F(" |LimDN:"));
    if(limitDownLocked) 
      Serial.print(F("LOCK"));          // 下限鎖定
    else 
      Serial.print(digitalRead(LIMIT_DOWN_PIN) == LOW ? F("ON") : F("--"));
    
    // ===== 速度與角度 =====
    Serial.print(F(" |Spd:"));
    Serial.print(currentSpeed, 1);      // 速度(1 位小數)
    Serial.print(F(" |Ang:"));
    Serial.print(compAngleX, 1);        // 角度(1 位小數)
    
    // ===== PWM 輸出值 =====
    Serial.print(F(" |PWM:"));
    Serial.print(targetPWM);            // 0-255
    
    // ===== 電流 =====
    Serial.print(F(" |I:"));
    Serial.print(current_A, 2);         // 電流(2 位小數)
    Serial.print(F("A"));
    
    // ===== 警告標記 =====
    if(tiltWarning) 
      Serial.print(F(" [TILT!]"));      // 傾斜警告
    if(manualMode) 
      Serial.print(F(" [MANUAL]"));     // 手動模式
    
    // ===== 控制來源 =====
    Serial.print(F(" Src:"));
    switch(ctrlSrc) {
      case 0: Serial.print(F("--")); break;     // 無
      case 1: Serial.print(F("LIM")); break;    // 磁簧開關
      case 2: Serial.print(F("IR")); break;     // 紅外線
      case 3: Serial.print(F("AUTO")); break;   // 自動
      case 4: Serial.print(F("TILT")); break;   // 傾斜保護
    }
    
    Serial.println();   // 換行
  }
  
  // ========== 小延遲防止 CPU 滿載 ==========
  delay(5);     // 延遲 5 毫秒,讓 CPU 休息一下
}
