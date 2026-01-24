#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <IRremote.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

const byte BUZZER_PIN = 8;
const byte IR_PIN = 3;
const byte RELAY_PIN = 4;
const byte LIMIT_UP = 52;
const byte LIMIT_DOWN = 53;
const byte RGB_RED = 5;
const byte RGB_GREEN = 7;
const byte RGB_BLUE = 6;

#define NOTE_C4  262
#define NOTE_E4  330
#define NOTE_G4  392
#define NOTE_C5  523

bool justStarted = false;
bool isIdle = true;
unsigned long lastRainbow = 0;
int rainbowStep = 0;

void setRGB(byte r, byte g, byte b) {
  analogWrite(RGB_RED, r);
  analogWrite(RGB_GREEN, g);
  analogWrite(RGB_BLUE, b);
}

void playTone(uint16_t frequency, uint16_t durationMs) {
  tone(BUZZER_PIN, frequency, durationMs);
  delay(durationMs + 20);
  noTone(BUZZER_PIN);
}

void rainbowEffect() {
  switch(rainbowStep) {
    case 0: setRGB(255, 0, 0); break;
    case 1: setRGB(255, 255, 0); break;
    case 2: setRGB(0, 255, 0); break;
    case 3: setRGB(0, 255, 255); break;
    case 4: setRGB(0, 0, 255); break;
    case 5: setRGB(255, 0, 255); break;
  }
  rainbowStep = (rainbowStep + 1) % 6;
}

void setup() {
  Serial.begin(115200);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(LIMIT_UP, INPUT_PULLUP);
  pinMode(LIMIT_DOWN, INPUT_PULLUP);
  pinMode(RGB_RED, OUTPUT);
  pinMode(RGB_GREEN, OUTPUT);
  pinMode(RGB_BLUE, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  
  digitalWrite(RELAY_PIN, LOW);
  
  playTone(NOTE_C4, 150);
  playTone(NOTE_E4, 150);
  playTone(NOTE_G4, 150);
  playTone(NOTE_C5, 300);
  
  Wire.begin();
  Wire.setClock(400000);
  
  // OLED 只顯示一次
  if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(10, 20);
    display.println("BIKE");
    display.setCursor(10, 40);
    display.println("ASSIST");
    display.display();
    Serial.println("OLED 啟動成功");
  }
  
  delay(100);
  
  IrReceiver.begin(IR_PIN, ENABLE_LED_FEEDBACK);
  
  Serial.println("系統啟動完成");
}

void loop() {
  // 彩虹效果
  if (isIdle && millis() - lastRainbow > 500) {
    rainbowEffect();
    lastRainbow = millis();
  }
  
  // 磁簧偵測
  if (!justStarted) {
    if (digitalRead(LIMIT_DOWN) == LOW) {
      digitalWrite(RELAY_PIN, LOW);
      isIdle = false;
      setRGB(255, 0, 0);
      Serial.println("下限!");
    }
    
    if (digitalRead(LIMIT_UP) == LOW) {
      digitalWrite(RELAY_PIN, LOW);
      isIdle = false;
      setRGB(0, 0, 255);
      Serial.println("上限!");
    }
  }
  
  // IR 控制
  if (IrReceiver.decode()) {
    uint32_t cmd = IrReceiver.decodedIRData.decodedRawData;
    
    if (cmd == 0xB946FF00) {
      digitalWrite(RELAY_PIN, HIGH);
      isIdle = false;
      setRGB(255, 255, 0);
      justStarted = true;
      Serial.println("IR:上鍵");
      delay(500);
      justStarted = false;
    }
    else if (cmd == 0xEA15FF00) {
      digitalWrite(RELAY_PIN, LOW);
      isIdle = true;
      Serial.println("IR:下鍵");
    }
    
    IrReceiver.resume();
  }
}
