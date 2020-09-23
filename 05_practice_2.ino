#define PIN_LED 7

void setup() {
  pinMode(PIN_LED, OUTPUT);
}

void loop() {
  digitalWrite(PIN_LED, 0);
  delay(1000);
  for (int fadeValue = 1; fadeValue <= 5; fadeValue += 1) {
    digitalWrite(PIN_LED, 1);
    delay(100);
    digitalWrite(PIN_LED, 0);
    delay(100);
  }
  while (1) {
    digitalWrite(PIN_LED, 1);
  }
}
