void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  if (Serial.available() > 0) {
    digitalWrite(LED_BUILTIN, HIGH);
//    delay(20);
    digitalWrite(LED_BUILTIN, LOW);
    char c = Serial.read();
    Serial.print(c);
  }
}
