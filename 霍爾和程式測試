// 霍爾感測器最簡單測試程式
// 只測試霍爾感測器和繼電器

const int HALL_SENSOR_PIN = 4;  // 霍爾感測器
const int RELAY_PIN = 11;       // 繼電器（用回原本的 PIN 11）
const int BUZZER_PIN = 8;       // 蜂鳴器

bool lastHallState = HIGH;

void setup() {
  Serial.begin(115200);
  delay(500);
  
  pinMode(HALL_SENSOR_PIN, INPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  
  digitalWrite(RELAY_PIN, LOW);   // 初始關閉
  digitalWrite(BUZZER_PIN, LOW);
  
  Serial.println("=== Hall Sensor Test ===");
  Serial.println("Pin 4: Hall Sensor");
  Serial.println("Pin 11: Relay");
  Serial.println("Relay logic: HIGH=ON, LOW=OFF");
  Serial.println("Bring magnet close to test");
  Serial.println("========================\n");
}

void loop() {
  int hallState = digitalRead(HALL_SENSOR_PIN);
  
  // 狀態改變時才處理
  if (hallState != lastHallState) {
    Serial.println("------------------------");
    
    if (hallState == LOW) {
      // 偵測到磁場：啟動繼電器（HIGH = ON）
      digitalWrite(RELAY_PIN, HIGH);
      digitalWrite(BUZZER_PIN, HIGH);
      delay(100);
      digitalWrite(BUZZER_PIN, LOW);
      
      Serial.println("🧲 MAGNET DETECTED!");
      Serial.println("   Relay: ON (HIGH)");
    } else {
      // 沒有磁場：關閉繼電器（LOW = OFF）
      digitalWrite(RELAY_PIN, LOW);
      digitalWrite(BUZZER_PIN, HIGH);
      delay(50);
      digitalWrite(BUZZER_PIN, LOW);
      
      Serial.println("⭕ No magnet");
      Serial.println("   Relay: OFF (LOW)");
    }
    
    lastHallState = hallState;
  }
  
  delay(50);
}
