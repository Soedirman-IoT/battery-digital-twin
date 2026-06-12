const int relayPin = 5;
unsigned long previousMillis = 0;
const long interval = 10000; // 3 detik

bool relayState = false;

void setup() {
  pinMode(relayPin, OUTPUT);
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    relayState = !relayState; // toggle state
    digitalWrite(relayPin, relayState ? HIGH : LOW);
  }
}