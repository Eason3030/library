#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <LiquidCrystal_PCF8574.h>

#define MPU_6050 0x68 //設定mpu6050位置
#define PWR_MGMT_1 0x6B//寬度調變 
#define ACCEL_XOUT_H 0x3B //加速度計
#define GYRO_XOUT_H  0x43 //陀螺儀

// OLED
#define SCREEN_WIDTH 128//定義寬度128*64
#define SCREEN_HEIGHT 64//高度永遠保持64
#define OLED_RESET -1//不需要復位接角(就是不須RESET)使用其他方式RESET
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// LCD
LiquidCrystal_PCF8574 lcd(0x27);

// PIN設定
const int BUZZER_PIN = 8;
const int MOTOR_PIN_A = 9;
const int MOTOR_PIN_B = 10;
const int RELAY_PIN = 11;
const int IR_PIN = 2;//紅外線遙控接角

unsigned long lastMicros = 0;//定義MICROS
float dt = 0.01f;
const float TILT_THRESHOLD = 25.0f;
const float alpha = 0.98f;

float compAngleX = 0.0f;
float gyroAngleX = 0.0f;
float accelBiasX = 0.0f;
float gyroBiasX = 0.0f;
float velX = 0.0f;  // 模擬用速度

// ---------- I2C ----------
void mpuReadBytes(uint8_t reg, uint8_t count, uint8_t* dest) {
  Wire.beginTransmission(MPU_6050);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_6050, (uint8_t)count);
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

void calibrateIMU(unsigned int samples = 200) {
  float sumAx = 0.0f, sumGx = 0.0f;
  for (unsigned int i = 0; i < samples; ++i) {
    int16_t rawAx = read16(ACCEL_XOUT_H);
    int16_t rawGx = read16(GYRO_XOUT_H);
    sumAx += rawAx / 16384.0f;
    sumGx += rawGx / 131.0f;
    delay(5);
  }
  accelBiasX = sumAx / samples;
  gyroBiasX = sumGx / samples;
}

void readIMU(float &ax_g, float &ay_g, float &az_g, float &gx_dps, float &gy_dps, float &gz_dps) {
  int16_t rawAx = read16(ACCEL_XOUT_H);
  int16_t rawAy = read16(ACCEL_XOUT_H + 2);
  int16_t rawAz = read16(ACCEL_XOUT_H + 4);
  int16_t rawGx = read16(GYRO_XOUT_H);
  int16_t rawGy = read16(GYRO_XOUT_H + 2);
  int16_t rawGz = read16(GYRO_XOUT_H + 4);

  ax_g = rawAx / 16384.0f;
  ay_g = rawAy / 16384.0f;
  az_g = rawAz / 16384.0f;
  gx_dps = rawGx / 131.0f;
  gy_dps = rawGy / 131.0f;
  gz_dps = rawGz / 131.0f;
}

void showStartupScreen() {
  display.clearDisplay();
  //設定起始畫面
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(25, 20);
  display.print("System");
  display.setCursor(10, 45);
  display.print("Starting...");
  display.display();
  delay(2500);
  display.clearDisplay();
  display.display();
}

void showStatus(const char* statusText) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 10);
  display.print("System Status:");
  display.setCursor(0, 30);
  display.print(statusText);
  display.display();
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  setupMPU();

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(MOTOR_PIN_A, OUTPUT);
  pinMode(MOTOR_PIN_B, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(IR_PIN, INPUT);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED init failed");
    while (1);
  }

  lcd.begin(16, 2);
  lcd.setBacklight(1);
  showStartupScreen();

  Serial.println("Calibrating IMU...");
  calibrateIMU(300);
  lastMicros = micros();
  Serial.println("Setup complete");
}

void loop() {
  unsigned long now = micros();
  dt = (now - lastMicros) / 1000000.0f;
  if (dt <= 0.0f) dt = 0.01f;
  lastMicros = now;

  float ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps;
  readIMU(ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps);

  float gx_corrected = gx_dps - gyroBiasX;
  float accelAngleX = atan2(ay_g, az_g) * 57.2958f;
  gyroAngleX += gx_corrected * dt;
  compAngleX = alpha * (compAngleX + gx_corrected * dt) + (1.0f - alpha) * accelAngleX;

  // ---- 模擬速度 (根據傾斜角度變化模擬前進速度) ----
  float targetSpeed = constrain(abs(compAngleX) / TILT_THRESHOLD * 10.0f, 0.0f, 10.0f); // 0~10 m/s
  velX = 0.9f * velX + 0.1f * targetSpeed;  // 慢速平滑

  // ---- 狀態判斷 ----
  if (digitalRead(IR_PIN) == LOW) {
    showStatus("IR Triggered: Assist Down!");
    digitalWrite(RELAY_PIN, HIGH);
    digitalWrite(BUZZER_PIN, HIGH);
  } else if (abs(compAngleX) > TILT_THRESHOLD) {
    showStatus("TILT ALERT!");
    digitalWrite(RELAY_PIN, HIGH);
    digitalWrite(BUZZER_PIN, HIGH);
  } else {
    showStatus("Normal Operation");
    digitalWrite(RELAY_PIN, LOW);
    digitalWrite(BUZZER_PIN, LOW);
  }

  // ---- LCD 顯示 ----
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Speed:");
  lcd.setCursor(7, 0);
  lcd.print(velX, 2);
  lcd.print("m/s");

  lcd.setCursor(0, 1);
  lcd.print("Angle:");
  lcd.print(compAngleX, 1);
  lcd.print((char)223);

  delay(100);
}
//後來把LCD刪除了
