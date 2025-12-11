#include <Wire.h>
#include <NewPing.h>
#include <Adafruit_SSD1306.h>
#include <IRremote.h>

// ----------------- Buzzer -----------------
#include "pitches.h"
const int BUZZER_PIN = 8;
void beep(int f, int d) {
  tone(BUZZER_PIN, f, d);
  delay(d * 1.3);
  noTone(BUZZER_PIN);
}
void soundDown()    { beep(NOTE_G5,60); beep(NOTE_E5,60); beep(NOTE_C5,100); }
void soundUp()      { beep(NOTE_C5,60); beep(NOTE_E5,60); beep(NOTE_G5,100); }
void soundError()   { beep(NOTE_DS5,50); beep(NOTE_C5,50); beep(NOTE_G4,50); }

// ----------------- Constants -----------------
const float TARGET_ANGLE     = -6.0f;
const float KP = 2.00f, KI = 0.05f, KD=0.06f;
const int FORWARD=9, BACKWARD=10;
const int TRIGGER_PIN = 5, ECHO_PIN=4;
const int MAX_DISTANCE = 250;
NewPing sonar(TRIGGER_PIN,ECHO_PIN,MAX_DISTANCE);
bool safeToMove = true;
float safetyThreshold = 15;

// IMU
float gyroBiasX = 0;
float currentAngle = TARGET_ANGLE;
float currentSpeed = 0;

// OLED
Adafruit_SSD1306 oled(128, 64, &Wire);

// IR
const int IR_PIN = 11;
uint32_t lastCode = 0;

// Hall sensor
const int HALL_PIN = 2;
volatile bool wheelDown = false;

// PID
float pidIntegral=0, pidPrevError=0;

// Flags
bool autoMode = true;
bool lowSpeedFlag = false;
const float SPEED_THRESHOLD = 12.0f;

// ----------------- IMU Base Functions -----------------
float read16(int reg) {
  Wire.beginTransmission(0x68);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 2, true);
  int16_t v = (Wire.read()<<8) | Wire.read();
  return v;
}

void readIMU(float& angle, float& speed) {
  float g = read16(0x43)/131.0f - gyroBiasX;
  speed = g;
  angle += speed * 0.01f;
}

void calibrateIMU() {
  float sum=0;
  for(int i=0;i<50;i++){
    sum+= read16(0x43)/131.0f;
    delay(5);
  }
  gyroBiasX = sum/50.0f;
}

// ----------------- Display -----------------
void showData(float angle, float speed) {
  oled.clearDisplay();
  oled.setCursor(0,0);
  oled.setTextSize(1);
  if(autoMode) oled.print("Mode: AUTO ");
  else         oled.print("Mode: MANUAL ");

  oled.print("Safe:");
  oled.println(safeToMove ? "Y":"N");

  oled.print("Ang:"); oled.print(angle,1);
  oled.print(" Spd:"); oled.println(speed,1);

  oled.print("Dist:");
  oled.println(sonar.ping_cm());
  oled.display();
}

// ----------------- Safety Check -----------------
bool checkObstacle() {
  unsigned long d = sonar.ping_cm();
  if(d>0 && d <= safetyThreshold){
    safeToMove = false;
    return false;
  }
  safeToMove = true;
  return true;
}

// ----------------- IR Control (IRremote 3.x) -----------------
void checkIR() {
  if(IrReceiver.decode()){
    uint32_t code = IrReceiver.decodedIRData.decodedRawData;
    lastCode = code;

    switch(code){
      case 0xFFA25D:  // Power = auto/manual toggle
        autoMode = !autoMode;
        autoMode ? soundUp() : soundDown();
        break;

      case 0xFF02FD:  // Up
        if(safeToMove) digitalWrite(FORWARD, HIGH);
        break;

      case 0xFF22DD:  // Left
        digitalWrite(FORWARD, HIGH);
        break;

      case 0xFFC23D:  // Right
        digitalWrite(BACKWARD, HIGH);
        break;

      case 0xFFE21D:  // Down
        digitalWrite(BACKWARD, HIGH);
        break;

      default:
        digitalWrite(FORWARD, LOW);
        digitalWrite(BACKWARD, LOW);
        break;
    }
    IrReceiver.resume();
  }
}

// ----------------- Auto Mode -----------------
void autoControl(float angle, float speed){
  lowSpeedFlag = speed < SPEED_THRESHOLD;

  if(!safeToMove){
    digitalWrite(FORWARD, LOW);
    digitalWrite(BACKWARD, LOW);
    return;
  }

  // wheelDown 代表希望倒車避障
  if(lowSpeedFlag && !wheelDown){
    wheelDown = true;
    soundDown();
  }

  if(wheelDown){
    digitalWrite(FORWARD, LOW);
    digitalWrite(BACKWARD, HIGH);
    return;
  }

  // 正常 PID 控制前進
  float error = TARGET_ANGLE - angle;
  pidIntegral += error * 0.01f;
  float derivative = (error - pidPrevError)/0.01f;
  float output = KP*error + KI*pidIntegral + KD*derivative;

  pidPrevError = error;

  if(output > 0){
    digitalWrite(FORWARD, HIGH);
    digitalWrite(BACKWARD, LOW);
  }else{
    digitalWrite(FORWARD, LOW);
    digitalWrite(BACKWARD, HIGH);
  }
}

// ----------------- Hall Interrupt -----------------
void IRAM_ATTR wheelHallISR() {
  wheelDown = true;
}

// ----------------- Setup -----------------
void setup() {
  Wire.begin();
  Serial.begin(9600);

  pinMode(FORWARD, OUTPUT);
  pinMode(BACKWARD,OUTPUT);
  pinMode(BUZZER_PIN,OUTPUT);
  pinMode(HALL_PIN,INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(HALL_PIN), wheelHallISR, FALLING);

  if(!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)){
    soundError();
    while(1);
  }
  oled.clearDisplay();
  oled.setTextColor(1);

  IrReceiver.begin(IR_PIN, ENABLE_LED_FEEDBACK);

  calibrateIMU();
  soundUp();
}

// ----------------- Loop -----------------
void loop() {
  readIMU(currentAngle, currentSpeed);
  checkObstacle();
  checkIR();

  if(autoMode){
    autoControl(currentAngle, currentSpeed);
  }

  showData(currentAngle, currentSpeed);
  delay(10);
}
