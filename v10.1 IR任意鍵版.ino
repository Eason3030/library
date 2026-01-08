// ============================================================================
// 輔助輪 v10.1 - IR 任意鍵版
// ============================================================================
// 基於 v10.0，僅修改 IR 處理邏輯：
// ✓ IR 任何按鍵都可觸發（不檢查特定碼）
// ✓ 其他所有邏輯與 v10.0 完全相同
// ============================================================================

#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <IRremote.h>

// ============================================================================
// OLED 設定
// ============================================================================
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// ============================================================================
// 腳位定義
// ============================================================================
// 控制輸出
#define RELAY_PIN         22   // 繼電器（馬達電源開關）
#define BUZZER_PIN        8    // 蜂鳴器
#define RGB_RED           5    // RGB 紅
#define RGB_GREEN         6    // RGB 綠
#define RGB_BLUE          7    // RGB 藍

// 感測器輸入
#define UPPER_SWITCH      13   // 上磁簧（內建上拉，觸發=LOW）
#define LOWER_SWITCH      53   // 下磁簧（內建上拉，觸發=LOW）
#define HALL_SENSOR       2    // 霍爾速度感測（中斷腳）
#define IR_RECEIVER       3    // 紅外線接收
#define BUTTON_PIN        12   // 按鈕（OLED 換頁）

// ============================================================================
// 音符定義
// ============================================================================
#define NOTE_C4  262
#define NOTE_E4  330
#define NOTE_G4  392
#define NOTE_C5  523
#define NOTE_E5  659
#define NOTE_G5  784
#define NOTE_A5  880

// ============================================================================
// 系統常數
// ============================================================================
namespace Config {
  // 繼電器邏輯（高電平觸發）
  const byte RELAY_ON = HIGH;
  const byte RELAY_OFF = LOW;

  // 磁簧開關邏輯（觸發 = LOW）
  const byte SWITCH_TRIGGERED = LOW;
  const byte SWITCH_RELEASED = HIGH;

  // 速度計算
  const float WHEEL_CIRCUMFERENCE_MM = 2042.0;  // 輪周長 (mm)
  const float SPEED_THRESHOLD_KMH = 15.0;        // 上升速度門檻 (km/h)
  const uint32_t MIN_PULSE_INTERVAL_US = 40000;  // 最小脈衝間隔 (40ms)

  // 時間保護
  const unsigned long MOTION_TIMEOUT_MS = 10000;      // 動作超時 10 秒
  const unsigned long SPEED_TIMEOUT_MS = 2000;        // 速度超時 2 秒
  const unsigned long DEBOUNCE_MS = 50;               // 防彈跳時間
  const unsigned long INIT_HOLD_MS = 500;             // 初始化到位後等待時間

  // RGB 閃爍
  const unsigned long RGB_BLINK_MS = 300;

  // OLED 更新
  const unsigned long DISPLAY_UPDATE_MS = 100;

  // 狀態輸出
  const unsigned long STATUS_PRINT_MS = 1000;
}

// ============================================================================
// 系統狀態定義
// ============================================================================
enum SystemState : byte {
  INIT,           // 初始化中（自動校正）
  IDLE,           // 待機狀態
  MOVING_DOWN,    // 下降中
  MOVING_UP,      // 上升中
  ERROR_STATE     // 異常停止
};

// ============================================================================
// 初始化階段
// ============================================================================
enum InitPhase : byte {
  INIT_START,           // 開始
  INIT_SEARCHING,       // 搜尋限位中
  INIT_UPPER_FOUND,     // 找到上限位
  INIT_MOVING_TO_LOWER, // 移動至下限位
  INIT_COMPLETED,       // 完成
  INIT_FAILED           // 失敗
};

// ============================================================================
// 錯誤類型
// ============================================================================
enum ErrorType : byte {
  NO_ERROR = 0,
  TIMEOUT_ERROR,        // 超時
  BOTH_SWITCHES_ERROR,  // 兩個磁簧同時觸發
  INIT_TIMEOUT_ERROR    // 初始化超時
};

// ============================================================================
// 資料結構
// ============================================================================

// 繼電器控制
struct RelayControl {
  bool isOn = false;
  unsigned long lastChangeTime = 0;

  void turnOn() {
    digitalWrite(RELAY_PIN, Config::RELAY_ON);
    isOn = true;
    lastChangeTime = millis();
    Serial.println(F("[繼電器] 通電"));
  }

  void turnOff() {
    digitalWrite(RELAY_PIN, Config::RELAY_OFF);
    isOn = false;
    lastChangeTime = millis();
    Serial.println(F("[繼電器] 斷電"));
  }
};

// 磁簧開關狀態
struct SwitchState {
  bool upperTriggered = false;
  bool lowerTriggered = false;
  unsigned long lastDebounceTime = 0;

  bool needsDebounce() {
    return (millis() - lastDebounceTime) < Config::DEBOUNCE_MS;
  }

  void updateDebounceTime() {
    lastDebounceTime = millis();
  }
};

// 速度測量
struct SpeedMeasurement {
  volatile uint32_t lastPulseTime = 0;
  volatile uint32_t pulseInterval = 0;
  volatile bool hasNewData = false;
  float currentSpeed = 0.0;
  unsigned long lastUpdateTime = 0;

  bool isTimedOut() {
    return (millis() - lastUpdateTime) > Config::SPEED_TIMEOUT_MS;
  }

  void markUpdated() {
    lastUpdateTime = millis();
  }
};

// 動作計時器
struct MotionTimer {
  unsigned long startTime = 0;
  bool isRunning = false;

  void start() {
    startTime = millis();
    isRunning = true;
  }

  void stop() {
    isRunning = false;
  }

  unsigned long elapsed() {
    return isRunning ? (millis() - startTime) : 0;
  }

  bool hasTimeout() {
    return isRunning && (elapsed() > Config::MOTION_TIMEOUT_MS);
  }
};

// 初始化狀態
struct InitState {
  InitPhase phase = INIT_START;
  unsigned long phaseStartTime = 0;
  bool upperSeen = false;

  void reset() {
    phase = INIT_START;
    phaseStartTime = millis();
    upperSeen = false;
  }
};

// IR 控制狀態
struct IRControl {
  bool autoUpEnabled = false;      // 自動上升模式開關
  unsigned long lastPressTime = 0; // 上次按下時間
  const unsigned long PRESS_DELAY = 500; // 防止連續觸發 (ms)
};

// 顯示狀態
struct DisplayState {
  byte currentPage = 0;
  unsigned long lastUpdateTime = 0;

  bool needsUpdate() {
    return (millis() - lastUpdateTime) >= Config::DISPLAY_UPDATE_MS;
  }

  void markUpdated() {
    lastUpdateTime = millis();
  }

  void nextPage() {
    currentPage = (currentPage + 1) % 2;  // 兩頁：主頁 / 除錯頁
  }
};

// ============================================================================
// 全域變數
// ============================================================================
SystemState g_state = INIT;
ErrorType g_errorType = NO_ERROR;

RelayControl g_relay;
SwitchState g_switches;
SpeedMeasurement g_speed;
MotionTimer g_motionTimer;
InitState g_init;
IRControl g_irControl;
DisplayState g_display;

// 輸入狀態
struct {
  bool lastButtonState = HIGH;
  unsigned long lastButtonPress = 0;
} g_input;

unsigned long g_lastStatusPrint = 0;

// ============================================================================
// 速度中斷
// ============================================================================
void speedInterrupt() {
  uint32_t now = micros();
  uint32_t interval = now - g_speed.lastPulseTime;

  if (interval > Config::MIN_PULSE_INTERVAL_US) {
    g_speed.pulseInterval = interval;
    g_speed.lastPulseTime = now;
    g_speed.hasNewData = true;
  }
}

// ============================================================================
// RGB LED 控制
// ============================================================================
void setRGB(byte r, byte g, byte b) {
  analogWrite(RGB_RED, r);
  analogWrite(RGB_GREEN, g);
  analogWrite(RGB_BLUE, b);
}

void updateRGBByState() {
  static unsigned long lastBlink = 0;
  static bool blinkState = false;

  switch(g_state) {
    case INIT:
      // 紫色閃爍
      if (millis() - lastBlink > Config::RGB_BLINK_MS) {
        lastBlink = millis();
        blinkState = !blinkState;
        setRGB(blinkState ? 255 : 0, 0, blinkState ? 255 : 0);
      }
      break;

    case IDLE:
      // 青色（待機）
      setRGB(0, 255, 255);
      break;

    case MOVING_DOWN:
      // 紅色（下降）
      setRGB(255, 0, 0);
      break;

    case MOVING_UP:
      // 綠色（上升）
      setRGB(0, 255, 0);
      break;

    case ERROR_STATE:
      // 紅色閃爍
      if (millis() - lastBlink > Config::RGB_BLINK_MS) {
        lastBlink = millis();
        blinkState = !blinkState;
        setRGB(blinkState ? 255 : 0, 0, 0);
      }
      break;
  }
}

// ============================================================================
// 音效函數
// ============================================================================
void playTone(uint16_t frequency, uint16_t duration) {
  tone(BUZZER_PIN, frequency, duration);
  delay(duration + 20);
  noTone(BUZZER_PIN);
}

void playStartup() {
  playTone(NOTE_C4, 100);
  playTone(NOTE_E4, 100);
  playTone(NOTE_G4, 100);
  playTone(NOTE_C5, 200);
}

void playInitComplete() {
  playTone(NOTE_G4, 150);
  playTone(NOTE_C5, 150);
  playTone(NOTE_E5, 200);
}

void playMoveDown() {
  playTone(NOTE_C5, 100);
}

void playMoveUp() {
  playTone(NOTE_E5, 100);
}

void playComplete() {
  playTone(NOTE_G5, 150);
}

void playError() {
  for(int i = 0; i < 3; i++) {
    tone(BUZZER_PIN, NOTE_A5, 100);
    delay(150);
  }
  noTone(BUZZER_PIN);
}

// ============================================================================
// 速度測量
// ============================================================================
void updateSpeedMeasurement() {
  if (g_speed.hasNewData) {
    if (g_speed.pulseInterval > 0) {
      // 速度 (km/h) = (周長(mm) / 間隔(μs)) * 3.6
      float timeInSec = g_speed.pulseInterval / 1000000.0;
      float rawSpeed = (Config::WHEEL_CIRCUMFERENCE_MM / timeInSec) * 0.0036;

      // 合理性檢查 (0~50 km/h)
      if (rawSpeed >= 0 && rawSpeed <= 50.0) {
        g_speed.currentSpeed = rawSpeed;
      }
    }
    g_speed.hasNewData = false;
    g_speed.markUpdated();
  }

  // 超時歸零
  if (g_speed.isTimedOut()) {
    g_speed.currentSpeed = 0.0;
  }
}

// ============================================================================
// 磁簧開關讀取
// ============================================================================
void updateSwitches() {
  if (g_switches.needsDebounce()) return;
  g_switches.updateDebounceTime();

  bool upperRaw = (digitalRead(UPPER_SWITCH) == Config::SWITCH_TRIGGERED);
  bool lowerRaw = (digitalRead(LOWER_SWITCH) == Config::SWITCH_TRIGGERED);

  // 除錯輸出
  static bool lastUpper = false;
  static bool lastLower = false;
  if (upperRaw != lastUpper || lowerRaw != lastLower) {
    Serial.print(F("[磁簧] 上:"));
    Serial.print(upperRaw ? "ON" : "off");
    Serial.print(F(" 下:"));
    Serial.println(lowerRaw ? "ON" : "off");
  }
  lastUpper = upperRaw;
  lastLower = lowerRaw;

  g_switches.upperTriggered = upperRaw;
  g_switches.lowerTriggered = lowerRaw;
}

// ============================================================================
// 初始化流程（自動校正）
// ============================================================================
void processInitialization() {
  switch(g_init.phase) {
    case INIT_START:
      Serial.println(F("\n[初始化] 開始自動校正"));
      Serial.println(F("  策略：搜尋上限位 → 移動至下限位"));
      g_relay.turnOn();
      g_motionTimer.start();
      g_init.phase = INIT_SEARCHING;
      g_init.phaseStartTime = millis();
      break;

    case INIT_SEARCHING:
      // 搜尋上限位
      if (g_switches.upperTriggered && !g_init.upperSeen) {
        Serial.println(F("[初始化] 找到上限位，繼續移動"));
        g_init.upperSeen = true;
        g_init.phase = INIT_MOVING_TO_LOWER;
      }

      // 如果直接碰到下限位（啟動時就在下方）
      if (g_switches.lowerTriggered) {
        Serial.println(F("[初始化] 啟動時已在下限位"));
        g_relay.turnOff();
        g_motionTimer.stop();
        delay(Config::INIT_HOLD_MS);
        g_init.phase = INIT_COMPLETED;
        g_state = IDLE;
        playInitComplete();
      }

      // 超時檢查
      if (g_motionTimer.hasTimeout()) {
        Serial.println(F("[初始化] 失敗 - 超時未找到限位"));
        g_relay.turnOff();
        g_init.phase = INIT_FAILED;
        g_state = ERROR_STATE;
        g_errorType = INIT_TIMEOUT_ERROR;
        playError();
      }
      break;

    case INIT_MOVING_TO_LOWER:
      // 等待下限位觸發
      if (g_switches.lowerTriggered) {
        Serial.println(F("[初始化] 到達下限位"));
        g_relay.turnOff();
        g_motionTimer.stop();
        delay(Config::INIT_HOLD_MS);
        g_init.phase = INIT_COMPLETED;
        g_state = IDLE;
        playInitComplete();
        Serial.println(F("[初始化] 完成！系統進入待機"));
      }

      // 超時檢查
      if (g_motionTimer.hasTimeout()) {
        Serial.println(F("[初始化] 失敗 - 超時未到達下限位"));
        g_relay.turnOff();
        g_init.phase = INIT_FAILED;
        g_state = ERROR_STATE;
        g_errorType = INIT_TIMEOUT_ERROR;
        playError();
      }
      break;

    case INIT_COMPLETED:
    case INIT_FAILED:
      // 不做任何事，等待主循環處理
      break;
  }
}

// ============================================================================
// 下降動作
// ============================================================================
void startMovingDown() {
  if (g_state != IDLE) {
    Serial.println(F("[下降] 拒絕 - 非待機狀態"));
    return;
  }

  Serial.println(F("[下降] 啟動"));
  g_state = MOVING_DOWN;
  g_relay.turnOn();
  g_motionTimer.start();
  playMoveDown();
}

void processMovingDown() {
  // 檢查下限位
  if (g_switches.lowerTriggered) {
    Serial.println(F("[下降] 到達下限位"));
    g_relay.turnOff();
    g_motionTimer.stop();
    g_state = IDLE;
    playComplete();
    return;
  }

  // 超時保護
  if (g_motionTimer.hasTimeout()) {
    Serial.println(F("[下降] 錯誤 - 超時未到達限位"));
    g_relay.turnOff();
    g_motionTimer.stop();
    g_state = ERROR_STATE;
    g_errorType = TIMEOUT_ERROR;
    playError();
  }
}

// ============================================================================
// 上升動作
// ============================================================================
void startMovingUp() {
  if (g_state != IDLE) {
    Serial.println(F("[上升] 拒絕 - 非待機狀態"));
    return;
  }

  Serial.println(F("[上升] 啟動"));
  g_state = MOVING_UP;
  g_relay.turnOn();
  g_motionTimer.start();
  playMoveUp();
}

void processMovingUp() {
  // 檢查上限位
  if (g_switches.upperTriggered) {
    Serial.println(F("[上升] 到達上限位"));
    g_relay.turnOff();
    g_motionTimer.stop();
    g_state = IDLE;
    playComplete();
    return;
  }

  // 超時保護
  if (g_motionTimer.hasTimeout()) {
    Serial.println(F("[上升] 錯誤 - 超時未到達限位"));
    g_relay.turnOff();
    g_motionTimer.stop();
    g_state = ERROR_STATE;
    g_errorType = TIMEOUT_ERROR;
    playError();
  }
}

// ============================================================================
// 自動上升檢查（當速度達標時）
// ============================================================================
void checkAutoUpCondition() {
  if (g_state != IDLE) return;
  if (!g_irControl.autoUpEnabled) return;

  // 速度達到門檻且在下限位
  if (g_speed.currentSpeed >= Config::SPEED_THRESHOLD_KMH &&
      g_switches.lowerTriggered) {
    Serial.print(F("[自動] 速度達標 ("));
    Serial.print(g_speed.currentSpeed, 1);
    Serial.println(F(" km/h) - 啟動上升"));
    startMovingUp();
  }
}

// ============================================================================
// 異常檢查
// ============================================================================
void checkForErrors() {
  // 兩個磁簧同時觸發
  if (g_switches.upperTriggered && g_switches.lowerTriggered) {
    if (g_state != ERROR_STATE) {
      Serial.println(F("[錯誤] 兩個磁簧同時觸發！"));
      g_relay.turnOff();
      g_motionTimer.stop();
      g_state = ERROR_STATE;
      g_errorType = BOTH_SWITCHES_ERROR;
      playError();
    }
  }
}

// ============================================================================
// IR 遙控處理 - v10.1 修改版（任何按鍵都可用）
// ============================================================================
void handleIRCommand() {
  if (!IrReceiver.decode()) return;

  uint32_t irCode = IrReceiver.decodedIRData.decodedRawData;
  IrReceiver.resume();

  // 忽略無效碼和重複碼
  if (irCode == 0 || irCode == 0xFFFFFFFF) return;

  // 防止連續觸發
  if (millis() - g_irControl.lastPressTime < g_irControl.PRESS_DELAY) return;
  g_irControl.lastPressTime = millis();

  Serial.print(F("[IR] 接收到信號 (0x"));
  Serial.print(irCode, HEX);
  Serial.println(F(")"));

  // ============================================================================
  // 【修改重點】所有按鍵都使用相同邏輯（不檢查特定碼）
  // ============================================================================

  // 根據當前狀態決定動作
  switch(g_state) {
    case INIT:
      // 初始化中不響應 IR
      Serial.println(F("[IR] 初始化中，忽略指令"));
      playTone(NOTE_C4, 100);
      break;

    case IDLE:
      // 待機狀態：切換上升/下降
      if (g_switches.lowerTriggered) {
        // 在下限位 → 上升
        Serial.println(F("[IR] 切換為上升"));
        startMovingUp();
      }
      else if (g_switches.upperTriggered) {
        // 在上限位 → 下降
        Serial.println(F("[IR] 切換為下降"));
        startMovingDown();
      }
      else {
        // 在中間位置 → 預設下降
        Serial.println(F("[IR] 預設執行下降"));
        startMovingDown();
      }
      break;

    case MOVING_DOWN:
      // 下降中 → 切換為上升
      Serial.println(F("[IR] 取消下降，改為上升"));
      g_relay.turnOff();
      g_motionTimer.stop();
      g_state = IDLE;
      delay(100); // 短暫停頓
      startMovingUp();
      break;

    case MOVING_UP:
      // 上升中 → 切換為下降
      Serial.println(F("[IR] 取消上升，改為下降"));
      g_relay.turnOff();
      g_motionTimer.stop();
      g_state = IDLE;
      delay(100); // 短暫停頓
      startMovingDown();
      break;

    case ERROR_STATE:
      // 錯誤狀態：需重啟系統
      Serial.println(F("[IR] 錯誤狀態，需重啟系統"));
      playError();
      break;
  }
}

// ============================================================================
// 按鈕處理（短按切換頁面，長按切換自動模式）
// ============================================================================
void handleButton() {
  bool buttonState = digitalRead(BUTTON_PIN);
  static unsigned long buttonPressStart = 0;
  static bool longPressHandled = false;

  // 按下瞬間
  if (buttonState == LOW && g_input.lastButtonState == HIGH) {
    buttonPressStart = millis();
    longPressHandled = false;
  }

  // 按住中
  if (buttonState == LOW && g_input.lastButtonState == LOW) {
    // 長按 2 秒切換自動上升模式
    if (!longPressHandled && (millis() - buttonPressStart > 2000)) {
      longPressHandled = true;
      g_irControl.autoUpEnabled = !g_irControl.autoUpEnabled;
      playTone(g_irControl.autoUpEnabled ? NOTE_E5 : NOTE_C4, 300);
      Serial.print(F("[按鈕] 自動上升模式: "));
      Serial.println(g_irControl.autoUpEnabled ? "開啟" : "關閉");
    }
  }

  // 放開瞬間
  if (buttonState == HIGH && g_input.lastButtonState == LOW) {
    unsigned long pressDuration = millis() - buttonPressStart;

    // 短按（< 2 秒）切換頁面
    if (pressDuration < 2000 && millis() - g_input.lastButtonPress > 200) {
      g_input.lastButtonPress = millis();
      g_display.nextPage();
      playTone(NOTE_A5, 50);
      Serial.print(F("[按鈕] 切換頁面: "));
      Serial.println(g_display.currentPage);
    }
  }

  g_input.lastButtonState = buttonState;
}

// ============================================================================
// OLED 顯示
// ============================================================================
void drawMainPage() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_unifont_t_chinese1);

  // 標題
  u8g2.setCursor(0, 10);
  u8g2.print("輔助輪 v10.1");

  u8g2.drawLine(0, 12, 127, 12);

  // 狀態
  u8g2.setCursor(0, 28);
  switch(g_state) {
    case INIT:
      u8g2.print("初始化中");
      break;
    case IDLE:
      u8g2.print("待機");
      break;
    case MOVING_DOWN:
      u8g2.print("下降中");
      break;
    case MOVING_UP:
      u8g2.print("上升中");
      break;
    case ERROR_STATE:
      u8g2.print("異常停止");
      break;
  }

  // 自動模式指示
  if (g_irControl.autoUpEnabled) {
    u8g2.setCursor(90, 28);
    u8g2.setFont(u8g2_font_6x10_tr);
    u8g2.print("[AUTO]");
    u8g2.setFont(u8g2_font_unifont_t_chinese1);
  }

  u8g2.drawLine(0, 30, 127, 30);

  // 速度
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.setCursor(0, 42);
  u8g2.print("Speed:");
  u8g2.setFont(u8g2_font_10x20_tr);
  u8g2.setCursor(40, 45);
  u8g2.print((int)g_speed.currentSpeed);
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.setCursor(70, 42);
  u8g2.print("km/h");

  // 磁簧狀態
  u8g2.setCursor(0, 56);
  u8g2.print("UP:");
  u8g2.print(g_switches.upperTriggered ? "ON" : "off");

  u8g2.setCursor(60, 56);
  u8g2.print("DN:");
  u8g2.print(g_switches.lowerTriggered ? "ON" : "off");

  // 繼電器狀態
  u8g2.setCursor(0, 63);
  u8g2.print("Relay:");
  u8g2.print(g_relay.isOn ? "ON" : "OFF");

  u8g2.sendBuffer();
}

void drawDebugPage() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_unifont_t_chinese1);

  u8g2.setCursor(0, 10);
  u8g2.print("除錯資訊");
  u8g2.drawLine(0, 12, 127, 12);

  u8g2.setFont(u8g2_font_6x10_tr);

  u8g2.setCursor(0, 22);
  u8g2.print("State: ");
  u8g2.print(g_state);

  u8g2.setCursor(0, 32);
  u8g2.print("Init Phase: ");
  u8g2.print(g_init.phase);

  u8g2.setCursor(0, 42);
  u8g2.print("Motion Timer: ");
  if (g_motionTimer.isRunning) {
    u8g2.print(g_motionTimer.elapsed() / 1000);
    u8g2.print("s");
  } else {
    u8g2.print("---");
  }

  u8g2.setCursor(0, 52);
  u8g2.print("Error Type: ");
  u8g2.print(g_errorType);

  u8g2.setCursor(0, 62);
  u8g2.print("Auto Up: ");
  u8g2.print(g_irControl.autoUpEnabled ? "ON" : "OFF");

  u8g2.sendBuffer();
}

void updateDisplay() {
  if (!g_display.needsUpdate()) return;
  g_display.markUpdated();

  if (g_display.currentPage == 0) {
    drawMainPage();
  } else {
    drawDebugPage();
  }
}

// ============================================================================
// 狀態輸出
// ============================================================================
void printStatus() {
  if (millis() - g_lastStatusPrint < Config::STATUS_PRINT_MS) return;
  g_lastStatusPrint = millis();

  Serial.print(F("[狀態] State:"));
  Serial.print(g_state);
  Serial.print(F(" Relay:"));
  Serial.print(g_relay.isOn ? "ON" : "OFF");
  Serial.print(F(" Speed:"));
  Serial.print(g_speed.currentSpeed, 1);
  Serial.print(F(" UP:"));
  Serial.print(g_switches.upperTriggered ? "1" : "0");
  Serial.print(F(" DN:"));
  Serial.print(g_switches.lowerTriggered ? "1" : "0");
  Serial.print(F(" Auto:"));
  Serial.println(g_irControl.autoUpEnabled ? "ON" : "OFF");
}

// ============================================================================
// Setup
// ============================================================================
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  Serial.println(F("\n========================================"));
  Serial.println(F("輔助輪系統 v10.1 啟動"));
  Serial.println(F("IR 任意鍵版"));
  Serial.println(F("========================================"));

  // 腳位設定
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(RGB_RED, OUTPUT);
  pinMode(RGB_GREEN, OUTPUT);
  pinMode(RGB_BLUE, OUTPUT);

  pinMode(UPPER_SWITCH, INPUT_PULLUP);
  pinMode(LOWER_SWITCH, INPUT_PULLUP);
  pinMode(HALL_SENSOR, INPUT_PULLUP);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // 確保繼電器關閉
  digitalWrite(RELAY_PIN, Config::RELAY_OFF);
  Serial.println(F("[初始化] 繼電器已關閉"));

  // I2C 初始化
  Wire.begin();
  Wire.setClock(400000);
  Serial.println(F("[初始化] I2C 已啟動"));

  // OLED 初始化
  u8g2.begin();
  u8g2.enableUTF8Print();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_unifont_t_chinese1);
  u8g2.setCursor(0, 20);
  u8g2.print("輔助輪系統");
  u8g2.setCursor(0, 40);
  u8g2.print("版本 10.1");
  u8g2.sendBuffer();
  Serial.println(F("[初始化] OLED 已啟動"));

  // IR 初始化
  IrReceiver.begin(IR_RECEIVER, ENABLE_LED_FEEDBACK);
  Serial.println(F("[初始化] IR 接收器已啟動"));

  // 速度中斷
  attachInterrupt(digitalPinToInterrupt(HALL_SENSOR), speedInterrupt, FALLING);
  Serial.println(F("[初始化] 速度感測已設定"));

  // 啟動音效
  playStartup();

  // RGB 測試
  setRGB(255, 0, 0); delay(200);
  setRGB(0, 255, 0); delay(200);
  setRGB(0, 0, 255); delay(200);
  setRGB(0, 0, 0);

  Serial.println(F("[初始化] 完成！\n"));
  Serial.println(F("系統進入初始化校正模式..."));
  Serial.println(F("\n【IR 功能】任何按鍵都可觸發上升/下降切換"));
}

// ============================================================================
// Loop
// ============================================================================
void loop() {
  // 讀取感測器
  updateSwitches();
  updateSpeedMeasurement();

  // 處理輸入
  handleIRCommand();
  handleButton();

  // 異常檢查
  checkForErrors();

  // 狀態機處理
  switch(g_state) {
    case INIT:
      processInitialization();
      break;

    case IDLE:
      checkAutoUpCondition();
      break;

    case MOVING_DOWN:
      processMovingDown();
      break;

    case MOVING_UP:
      processMovingUp();
      break;

    case ERROR_STATE:
      // 錯誤狀態，需要重啟
      break;
  }

  // 更新輸出
  updateRGBByState();
  updateDisplay();
  printStatus();
}
