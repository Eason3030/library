// ============================================================================
// Smart Training Wheel Control System v9.1 - Fixed Version
// ============================================================================
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <IRremote.h>
#include <avr/wdt.h>

// ============================================================================
// OLED Settings
// ============================================================================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ============================================================================
// Music Notes
// ============================================================================
#define NOTE_C4  262
#define NOTE_E4  330
#define NOTE_G4  392
#define NOTE_C5  523
#define NOTE_A5  880
#define NOTE_C6  1047

// ============================================================================
// MPU6050 Settings
// ============================================================================
#define MPU_ADDR 0x69
#define PWR_MGMT_1 0x6B
#define ACCEL_XOUT_H 0x3B

// ============================================================================
// Pin Definitions
// ============================================================================
const byte BUZZER_PIN = 8;
const byte IR_RECEIVE_PIN = 3;
const byte PWM_PIN = 9;

// Limit Switches (NO type, LOW = triggered)
const byte LIMIT_UP_PIN = 52;
const byte LIMIT_DOWN_PIN = 53;

// Speed Sensor
const byte HALL_SPEED_PIN = 2;
const byte SPEED_CONTROL_PIN = A0;

// RGB LED
const byte RGB_RED_PIN = 5;
const byte RGB_GREEN_PIN = 7;
const byte RGB_BLUE_PIN = 6;

// Physical Button - Page Switch
const byte BUTTON_PIN = 12;

// ============================================================================
// System States
// ============================================================================
enum SystemState : byte {
  AT_TOP,      // At upper limit
  AT_BOTTOM,   // At lower limit
  MOVING_UP,   // Moving up
  MOVING_DOWN  // Moving down
};

// ============================================================================
// Global Variables
// ============================================================================
SystemState currentState = AT_BOTTOM;  

// Limit Switches
bool upperLimit = false;
bool lowerLimit = false;
unsigned long lastLimitCheck = 0;
const unsigned long LIMIT_DEBOUNCE = 50;

// System Lock
bool systemLocked = false;
unsigned long lockStartTime = 0;
const unsigned long LOCK_DURATION = 500;

// PWM Control
int pwmSpeed = 190;
unsigned long lastSpeedRead = 0;

// PWM Manual Override
bool pwmManualOverride = false;

// Speed Measurement
#define WHEEL_CIRCUMFERENCE 204.2
volatile uint32_t lastSpeedTrigger = 0;
volatile uint32_t speedPulseInterval = 0;
volatile bool newSpeedData = false;
float currentSpeed = 0.0;
unsigned long lastSpeedUpdate = 0;
const unsigned long SPEED_TIMEOUT = 2000;

// Auto Lift Logic
const float AUTO_LIFT_SPEED = 15.0;
const unsigned long SPEED_HOLD_TIME = 1000;
unsigned long speedAboveThresholdTime = 0;
bool speedTimerRunning = false;
bool autoLiftDone = false;

// IR Control
unsigned long lastIRTime = 0;
const unsigned long IR_DELAY = 500;

// OLED Display - 3 Pages
byte currentPage = 0;
unsigned long lastDisplayUpdate = 0;

// Button Control
bool lastButtonState = HIGH;
unsigned long lastButtonPress = 0;
const unsigned long BUTTON_DEBOUNCE = 200;

// Serial Output
unsigned long lastSerialPrint = 0;

// IMU Data
float tiltAngle = 0.0;
unsigned long lastIMURead = 0;
const unsigned long IMU_READ_INTERVAL = 100;
const float TILT_WARNING = 10.0;
const float TILT_DANGER = 15.0;
bool lastTiltWarning = false;

// ============================================================================
// Speed Interrupt
// ============================================================================
void speedInterrupt() {
  uint32_t now = micros();
  uint32_t interval = now - lastSpeedTrigger;
  
  if (interval > 40000) {
    speedPulseInterval = interval;
    lastSpeedTrigger = now;
    newSpeedData = true;
  }
}

// ============================================================================
// RGB LED
// ============================================================================
void setRGB(byte r, byte g, byte b) {
  analogWrite(RGB_RED_PIN, r);
  analogWrite(RGB_GREEN_PIN, g);
  analogWrite(RGB_BLUE_PIN, b);
}

void updateRGBByState() {
  if (pwmManualOverride) {
    // PWM Manual OFF - Blinking Red Warning
    static unsigned long lastBlink = 0;
    static bool blinkState = false;
    if (millis() - lastBlink > 300) {
      lastBlink = millis();
      blinkState = !blinkState;
      setRGB(blinkState ? 255 : 0, 0, 0);
    }
    return;
  }
  
  switch(currentState) {
    case AT_TOP:
      setRGB(0, 255, 255);  // Cyan
      break;
    case AT_BOTTOM:
      setRGB(255, 200, 0);  // Yellow
      break;
    case MOVING_UP:
      setRGB(0, 255, 0);    // Green
      break;
    case MOVING_DOWN:
      setRGB(255, 0, 0);    // Red
      break;
  }
}

// ============================================================================
// Sound Effects
// ============================================================================
void playStartupMusic() {
  tone(BUZZER_PIN, NOTE_C4, 150); delay(180);
  tone(BUZZER_PIN, NOTE_E4, 150); delay(180);
  tone(BUZZER_PIN, NOTE_G4, 150); delay(180);
  tone(BUZZER_PIN, NOTE_C5, 300); delay(350);
  noTone(BUZZER_PIN);
}

void playBeep(uint16_t freq, uint16_t duration) {
  tone(BUZZER_PIN, freq, duration);
  delay(duration + 20);
  noTone(BUZZER_PIN);
}

void playWarning() {
  for(int i = 0; i < 3; i++) {
    tone(BUZZER_PIN, NOTE_C6, 100);
    delay(150);
  }
  noTone(BUZZER_PIN);
}

// ============================================================================
// IMU Functions
// ============================================================================
void setupIMU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0x00);
  byte error = Wire.endTransmission();
  
  if (error == 0) {
    Serial.println(F("IMU Init OK"));
  } else {
    Serial.println(F("IMU Init Failed"));
  }
  delay(50);
}

void readIMU() {
  if (millis() - lastIMURead < IMU_READ_INTERVAL) {
    return;
  }
  lastIMURead = millis();
  
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_XOUT_H);
  byte error = Wire.endTransmission(false);
  
  if (error != 0) {
    tiltAngle = 0.0;
    return;
  }
  
  Wire.requestFrom((byte)MPU_ADDR, (byte)4);
  if (Wire.available() >= 4) {
    int16_t ax = (Wire.read() << 8) | Wire.read();
    int16_t ay = (Wire.read() << 8) | Wire.read();
    
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(ACCEL_XOUT_H + 4);
    Wire.endTransmission(false);
    Wire.requestFrom((byte)MPU_ADDR, (byte)2);
    int16_t az = (Wire.read() << 8) | Wire.read();
    
    float accelX = ax / 16384.0;
    float accelZ = az / 16384.0;
    
    tiltAngle = atan2(accelX, accelZ) * 57.2958;
    
    if (tiltAngle > 90) tiltAngle = 90;
    if (tiltAngle < -90) tiltAngle = -90;
  }
}

void checkTiltWarning() {
  float absTilt = abs(tiltAngle);
  bool currentWarning = (absTilt >= TILT_WARNING);
  
  if (currentWarning && !lastTiltWarning) {
    if (absTilt >= TILT_DANGER) {
      playWarning();
      Serial.print(F("[WARN] Danger Tilt: "));
      Serial.println(tiltAngle, 1);
    } else {
      tone(BUZZER_PIN, NOTE_A5, 200);
      delay(220);
      noTone(BUZZER_PIN);
      Serial.print(F("[WARN] Tilt Warning: "));
      Serial.println(tiltAngle, 1);
    }
  }
  
  lastTiltWarning = currentWarning;
}

// ============================================================================
// Speed Calculation
// ============================================================================
void updateSpeed() {
  if (newSpeedData) {
    if (speedPulseInterval > 0) {
      float timeInSec = speedPulseInterval / 1000000.0;
      currentSpeed = (WHEEL_CIRCUMFERENCE / timeInSec) * 0.036;
    }
    newSpeedData = false;
    lastSpeedUpdate = millis();
  }
  
  if (millis() - lastSpeedUpdate > SPEED_TIMEOUT) {
    currentSpeed = 0.0;
  }
}

// ============================================================================
// Limit Switches Check
// ============================================================================
void checkLimitSwitches() {
  if (millis() - lastLimitCheck < LIMIT_DEBOUNCE) {
    return;
  }
  lastLimitCheck = millis();
  
  bool upperRaw = (digitalRead(LIMIT_UP_PIN) == LOW);
  bool lowerRaw = (digitalRead(LIMIT_DOWN_PIN) == LOW);
  
  if (upperRaw && !lowerRaw) {
    analogWrite(PWM_PIN, 0);
    currentState = AT_TOP;
    systemLocked = true;
    lockStartTime = millis();
    upperLimit = true;
    lowerLimit = false;
    
    autoLiftDone = false;
    speedTimerRunning = false;
    
    Serial.println(F("[LIMIT] Reached TOP"));
    
  } else if (lowerRaw && !upperRaw) {
    analogWrite(PWM_PIN, 0);
    currentState = AT_BOTTOM;
    systemLocked = true;
    lockStartTime = millis();
    upperLimit = false;
    lowerLimit = true;
    
    Serial.println(F("[LIMIT] Reached BOTTOM"));
    
  } else if (upperRaw && lowerRaw) {
    analogWrite(PWM_PIN, 0);
    currentState = AT_TOP;
    systemLocked = true;
    lockStartTime = millis();
    
    Serial.println(F("[ERROR] Both limits triggered!"));
    playWarning();
    
  } else {
    upperLimit = false;
    lowerLimit = false;
  }
}

// ============================================================================
// Motor Control
// ============================================================================
void startMotorUp() {
  if (pwmManualOverride) {
    Serial.println(F("[CTRL] PWM Manual OFF"));
    return;
  }
  
  if (systemLocked) {
    Serial.println(F("[CTRL] System Locked"));
    return;
  }
  
  if (currentState == MOVING_UP) {
    Serial.println(F("[CTRL] Already Moving Up"));
    return;
  }
  
  if (currentState == AT_TOP) {
    Serial.println(F("[CTRL] Already at TOP"));
    return;
  }
  
  analogWrite(PWM_PIN, pwmSpeed);
  currentState = MOVING_UP;
  systemLocked = true;
  lockStartTime = millis();
  
  playBeep(NOTE_A5, 100);
  Serial.println(F("[MOTOR] Start Moving UP"));
}

void startMotorDown() {
  if (pwmManualOverride) {
    Serial.println(F("[CTRL] PWM Manual OFF"));
    return;
  }
  
  if (systemLocked) {
    Serial.println(F("[CTRL] System Locked"));
    return;
  }
  
  if (currentState == MOVING_DOWN) {
    Serial.println(F("[CTRL] Already Moving Down"));
    return;
  }
  
  if (currentState == AT_BOTTOM) {
    Serial.println(F("[CTRL] Already at BOTTOM"));
    return;
  }
  
  analogWrite(PWM_PIN, pwmSpeed);
  currentState = MOVING_DOWN;
  systemLocked = true;
  lockStartTime = millis();
  
  playBeep(NOTE_A5, 100);
  Serial.println(F("[MOTOR] Start Moving DOWN"));
}

void stopMotor() {
  analogWrite(PWM_PIN, 0);
  Serial.println(F("[MOTOR] Stop"));
}

// ============================================================================
// Auto Lift Logic
// ============================================================================
void checkAutoLift() {
  if (currentState != AT_BOTTOM) {
    speedTimerRunning = false;
    speedAboveThresholdTime = 0;
    return;
  }
  
  if (autoLiftDone) {
    return;
  }
  
  if (pwmManualOverride) {
    return;
  }
  
  if (currentSpeed >= AUTO_LIFT_SPEED) {
    if (!speedTimerRunning) {
      speedTimerRunning = true;
      speedAboveThresholdTime = millis();
      Serial.println(F("[AUTO] Speed threshold reached, timer started"));
    } else {
      unsigned long elapsed = millis() - speedAboveThresholdTime;
      if (elapsed >= SPEED_HOLD_TIME) {
        Serial.println(F("[AUTO] Held 1 sec, starting lift!"));
        startMotorUp();
        autoLiftDone = true;
        speedTimerRunning = false;
        playBeep(NOTE_C5, 200);
      }
    }
  } else {
    if (speedTimerRunning) {
      Serial.println(F("[AUTO] Speed dropped, reset timer"));
    }
    speedTimerRunning = false;
    speedAboveThresholdTime = 0;
  }
}

// ============================================================================
// IR Control
// ============================================================================
void handleIR(uint32_t code) {
  if (millis() - lastIRTime < IR_DELAY) return;
  lastIRTime = millis();
  
  if (code != 0 && code != 0xFFFFFFFF) {
    pwmManualOverride = !pwmManualOverride;
    
    if (pwmManualOverride) {
      analogWrite(PWM_PIN, 0);
      playBeep(NOTE_C4, 200);
      Serial.println(F("[IR] PWM Manual OFF (Safe Mode)"));
    } else {
      playBeep(NOTE_C5, 200);
      Serial.println(F("[IR] PWM Auto Mode Restored"));
    }
  }
}

// ============================================================================
// Physical Button
// ============================================================================
void checkButton() {
  bool buttonState = digitalRead(BUTTON_PIN);
  
  if (buttonState == LOW && lastButtonState == HIGH) {
    if (millis() - lastButtonPress > BUTTON_DEBOUNCE) {
      lastButtonPress = millis();
      
      currentPage = (currentPage + 1) % 3;
      
      playBeep(NOTE_A5, 50);
      
      Serial.print(F("[BTN] Switch to Page "));
      Serial.println(currentPage);
    }
  }
  
  lastButtonState = buttonState;
}

// ============================================================================
// OLED Display - 3 Pages
// ============================================================================
void drawMainPage() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  
  // Title
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(F("WHEEL v9.1"));
  
  if (pwmManualOverride) {
    display.setCursor(80, 0);
    display.print(F("[SAFE]"));
  }
  
  display.drawLine(0, 9, 127, 9, SSD1306_WHITE);
  
  // State (Large)
  display.setTextSize(2);
  display.setCursor(0, 12);
  switch(currentState) {
    case AT_TOP:
      display.print(F("AT TOP"));
      break;
    case AT_BOTTOM:
      display.print(F("BOTTOM"));
      break;
    case MOVING_UP:
      display.print(F("UP"));
      break;
    case MOVING_DOWN:
      display.print(F("DOWN"));
      break;
  }
  
  display.drawLine(0, 28, 127, 28, SSD1306_WHITE);
  
  // Speed + Angle (Side by side)
  display.setTextSize(1);
  display.setCursor(0, 31);
  display.print(F("Speed:"));
  
  display.setTextSize(2);
  display.setCursor(0, 40);
  int speedInt = (int)currentSpeed;
  if (speedInt < 10) display.print(F(" "));
  display.print(speedInt);
  
  display.setTextSize(1);
  display.setCursor(30, 44);
  display.print(F("km/h"));
  
  // Angle
  display.setCursor(70, 31);
  display.print(F("Angle:"));
  
  display.setTextSize(2);
  display.setCursor(70, 40);
  int angleInt = (int)tiltAngle;
  if (angleInt >= 0 && angleInt < 10) display.print(F(" "));
  display.print(angleInt);
  
  display.setTextSize(1);
  display.setCursor(100, 44);
  display.print(F("deg"));
  
  // Warning Indicator
  if (abs(tiltAngle) >= TILT_DANGER) {
    display.fillRect(110, 31, 18, 10, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
    display.setCursor(112, 32);
    display.print(F("!!"));
    display.setTextColor(SSD1306_WHITE);
  } else if (abs(tiltAngle) >= TILT_WARNING) {
    display.setCursor(112, 32);
    display.print(F("!"));
  }
  
  display.drawLine(0, 52, 127, 52, SSD1306_WHITE);
  
  // Auto Lift Indicator + PWM
  display.setCursor(0, 54);
  display.print(F("PWM:"));
  display.print(pwmSpeed);
  
  if (speedTimerRunning) {
    unsigned long elapsed = millis() - speedAboveThresholdTime;
    unsigned long remaining = (SPEED_HOLD_TIME - elapsed) / 100;
    
    display.setCursor(60, 54);
    display.print(F("AUTO:"));
    display.print(remaining / 10);
    display.print(F("."));
    display.print(remaining % 10);
  }
  
  display.display();
}

void drawDebugPage() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(F("DEBUG PAGE"));
  
  display.drawLine(0, 9, 127, 9, SSD1306_WHITE);
  
  // Limit Switch Status
  display.setCursor(0, 12);
  display.print(F("Limit: ["));
  display.print(upperLimit ? F("U") : F("-"));
  display.print(F("]["));
  display.print(lowerLimit ? F("D") : F("-"));
  display.print(F("]"));
  
  // System Lock
  display.setCursor(0, 22);
  display.print(F("Lock: "));
  display.print(systemLocked ? F("YES") : F("NO"));
  
  // Auto Lift Status
  display.setCursor(0, 32);
  display.print(F("Auto: "));
  if (autoLiftDone) {
    display.print(F("DONE"));
  } else if (speedTimerRunning) {
    display.print(F("TIMING"));
  } else {
    display.print(F("READY"));
  }
  
  // Speed Detail
  display.setCursor(0, 42);
  display.print(F("Speed: "));
  display.print(currentSpeed, 1);
  display.print(F(" km/h"));
  
  // Angle Detail
  display.setCursor(0, 52);
  display.print(F("Tilt:  "));
  display.print(tiltAngle, 1);
  display.print(F(" deg"));
  
  if (abs(tiltAngle) >= TILT_DANGER) {
    display.print(F(" DNG"));
  } else if (abs(tiltAngle) >= TILT_WARNING) {
    display.print(F(" WRN"));
  }
  
  display.display();
}

void drawSystemInfoPage() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(F("SYSTEM INFO"));
  
  display.drawLine(0, 9, 127, 9, SSD1306_WHITE);
  
  // PWM Mode Status
  display.setCursor(0, 12);
  display.print(F("PWM Mode: "));
  if (pwmManualOverride) {
    display.print(F("MANUAL"));
    display.setCursor(0, 22);
    display.print(F("(Safety OFF)"));
  } else {
    display.print(F("AUTO"));
  }
  
  // Uptime
  display.setCursor(0, 32);
  display.print(F("Uptime: "));
  unsigned long seconds = millis() / 1000;
  unsigned long minutes = seconds / 60;
  unsigned long hours = minutes / 60;
  display.print(hours);
  display.print(F("h "));
  display.print(minutes % 60);
  display.print(F("m"));
  
  // PWM Value
  display.setCursor(0, 42);
  display.print(F("PWM Value: "));
  display.print(pwmSpeed);
  
  // Version
  display.setCursor(0, 52);
  display.print(F("Version: 9.1"));
  
  display.display();
}

void updateDisplay() {
  if (millis() - lastDisplayUpdate < 250) return;
  lastDisplayUpdate = millis();
  
  switch(currentPage) {
    case 0:
      drawMainPage();
      break;
    case 1:
      drawDebugPage();
      break;
    case 2:
      drawSystemInfoPage();
      break;
  }
}

// ============================================================================
// Setup
// ============================================================================
void setup() {
  wdt_disable();
  Serial.begin(115200);
  delay(100);
  
  // Pin Initialization
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(LIMIT_UP_PIN, INPUT_PULLUP);
  pinMode(LIMIT_DOWN_PIN, INPUT_PULLUP);
  pinMode(HALL_SPEED_PIN, INPUT_PULLUP);
  pinMode(SPEED_CONTROL_PIN, INPUT);
  pinMode(RGB_RED_PIN, OUTPUT);
  pinMode(RGB_GREEN_PIN, OUTPUT);
  pinMode(RGB_BLUE_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  // Ensure motor is stopped
  analogWrite(PWM_PIN, 0);
  setRGB(0, 0, 0);
  
  // Interrupts & Peripherals
  attachInterrupt(digitalPinToInterrupt(HALL_SPEED_PIN), speedInterrupt, FALLING);
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
  
  // OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("OLED Init Failed!"));
    while(1);
  }
  
  // Startup Screen
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);
  display.setCursor(10, 10);
  display.print(F("WHEEL"));
  display.setTextSize(1);
  display.setCursor(10, 35);
  display.print(F("v9.1 Simple"));
  display.display();
  delay(1500);
  
  playStartupMusic();
  
  // IMU Init
  setupIMU();
  
  Serial.println(F("===================================="));
  Serial.println(F("  Smart Training Wheel v9.1"));
  Serial.println(F("===================================="));
  Serial.println(F("\n[Core Features]"));
  Serial.println(F("- Speed >= 15 km/h for 1 sec -> Auto Lift"));
  Serial.println(F("- Limit switches absolute priority"));
  Serial.println(F("- Tilt > 10deg warning / > 15deg danger"));
  Serial.println(F("- Single lock mechanism"));
  Serial.println(F("- 3-page OLED (Main/Debug/System)"));
  Serial.println(F("===================================="));
  Serial.println(F("\n[Controls]"));
  Serial.println(F("IR Remote -> Any key toggles PWM ON/OFF"));
  Serial.println(F("Button(Pin12) -> Switch OLED page"));
  Serial.println(F("====================================\n"));
  
  // ============================================
  // PWM Module Self-Test
  // ============================================
  Serial.println(F("\n[PWM Module Self-Test]"));
  
  // Check limit switch status
  bool upperRaw = (digitalRead(LIMIT_UP_PIN) == LOW);
  bool lowerRaw = (digitalRead(LIMIT_DOWN_PIN) == LOW);
  
  if (upperRaw || lowerRaw) {
    Serial.println(F("WARNING: Limit switch triggered, skip PWM test"));
    if (upperRaw) Serial.println(F("  -> Currently at UPPER limit"));
    if (lowerRaw) Serial.println(F("  -> Currently at LOWER limit"));
  } else {
    Serial.println(F(">> Starting PWM test (200 PWM, 100ms)"));
    
    // White LED during test
    setRGB(255, 255, 255);
    
    // Start PWM
    analogWrite(PWM_PIN, 200);
    delay(100);
    
    // Stop PWM
    analogWrite(PWM_PIN, 0);
    
    // Turn off LED
    setRGB(0, 0, 0);
    
    Serial.println(F("OK: PWM test complete"));
    Serial.println(F("  If motor briefly activated, module is working"));
    
    delay(500);
  }
  
  Serial.println(F("===================================="));
  Serial.println(F("System Ready!\n"));
  
  wdt_enable(WDTO_2S);
}

// ============================================================================
// Main Loop
// ============================================================================
void loop() {
  wdt_reset();
  
  // ========== Priority 1: Safety Check ==========
  checkLimitSwitches();
  
  if (upperLimit || lowerLimit) {
    updateDisplay();
    updateRGBByState();
    delay(10);
    return;
  }
  
  // ========== Priority 2: Update Data ==========
  updateSpeed();
  readIMU();
  
  // Read PWM setting
  if (millis() - lastSpeedRead > 100) {
    lastSpeedRead = millis();
    int raw = analogRead(SPEED_CONTROL_PIN);
    pwmSpeed = map(raw, 0, 1023, 100, 255);
    pwmSpeed = constrain(pwmSpeed, 100, 255);
  }
  
  // ========== Priority 3: Unlock Check ==========
  if (systemLocked && millis() - lockStartTime > LOCK_DURATION) {
    systemLocked = false;
    Serial.println(F("[SYS] Unlocked"));
  }
  
  // ========== Priority 4: Auto Lift ==========
  checkAutoLift();
  
  // ========== Priority 5: IR Control ==========
  if (IrReceiver.decode()) {
    handleIR(IrReceiver.decodedIRData.decodedRawData);
    IrReceiver.resume();
  }
  
  // ========== Priority 6: Physical Button ==========
  checkButton();
  
  // ========== Priority 7: Tilt Warning ==========
  checkTiltWarning();
  
  // ========== Interface Update ==========
  updateDisplay();
  updateRGBByState();
  
  // ========== Serial Output (Every 1 sec) ==========
  if (millis() - lastSerialPrint > 1000) {
    lastSerialPrint = millis();
    
    Serial.print(F("State:"));
    switch(currentState) {
      case AT_TOP: Serial.print(F("TOP")); break;
      case AT_BOTTOM: Serial.print(F("BTM")); break;
      case MOVING_UP: Serial.print(F("UP^")); break;
      case MOVING_DOWN: Serial.print(F("DWN")); break;
    }
    
    Serial.print(F(" |Limit:["));
    Serial.print(upperLimit ? F("U") : F("-"));
    Serial.print(F("]["));
    Serial.print(lowerLimit ? F("D") : F("-"));
    Serial.print(F("]"));
    
    Serial.print(F(" |Speed:"));
    Serial.print(currentSpeed, 1);
    Serial.print(F("km/h"));
    
    Serial.print(F(" |Angle:"));
    Serial.print(tiltAngle, 1);
    Serial.print(F("deg"));
    
    // PWM Diagnostics
    Serial.print(F(" |PWM_Set:"));
    Serial.print(pwmSpeed);
    
    Serial.print(F(" |Pot_Raw:"));
    Serial.print(analogRead(SPEED_CONTROL_PIN));
    
    if (speedTimerRunning) {
      unsigned long elapsed = millis() - speedAboveThresholdTime;
      Serial.print(F(" |Timer:"));
      Serial.print(elapsed / 100);
      Serial.print(F("/10"));
    }
    
    Serial.print(F(" |Lock:"));
    Serial.print(systemLocked ? F("Y") : F("N"));
    
    if (pwmManualOverride) {
      Serial.print(F(" |PWM:MANUAL_OFF"));
    }
    
    Serial.print(F(" |Page:"));
    Serial.print(currentPage);
    
    Serial.println();
  }
  
  delay(5);
}
