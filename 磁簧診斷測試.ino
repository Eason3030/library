// ============================================================================
// 磁簧診斷測試程式
// 用途：確認磁簧開關硬體連接是否正常
// ============================================================================

#define UPPER_SWITCH  13   // 上磁簧
#define LOWER_SWITCH  53   // 下磁簧

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(UPPER_SWITCH, INPUT_PULLUP);
  pinMode(LOWER_SWITCH, INPUT_PULLUP);

  Serial.println(F("\n========================================"));
  Serial.println(F("  磁簧開關診斷測試"));
  Serial.println(F("========================================"));
  Serial.println(F("腳位設定："));
  Serial.print(F("  上磁簧：PIN "));
  Serial.println(UPPER_SWITCH);
  Serial.print(F("  下磁簧：PIN "));
  Serial.println(LOWER_SWITCH);
  Serial.println(F("\n正常狀態："));
  Serial.println(F("  沒磁鐵靠近 → 顯示 1 (HIGH)"));
  Serial.println(F("  磁鐵靠近   → 顯示 0 (LOW)"));
  Serial.println(F("========================================\n"));

  delay(1000);
}

void loop() {
  // 直接讀取原始腳位狀態
  int upperRaw = digitalRead(UPPER_SWITCH);
  int lowerRaw = digitalRead(LOWER_SWITCH);

  // 清晰輸出
  Serial.print(F("[原始讀值] UP pin = "));
  Serial.print(upperRaw);
  Serial.print(F("  |  DN pin = "));
  Serial.print(lowerRaw);

  // 狀態解釋
  Serial.print(F("  -->  上磁簧:"));
  Serial.print(upperRaw == HIGH ? "未觸發" : "【觸發】");
  Serial.print(F("  下磁簧:"));
  Serial.println(lowerRaw == HIGH ? "未觸發" : "【觸發】");

  delay(500);
}
