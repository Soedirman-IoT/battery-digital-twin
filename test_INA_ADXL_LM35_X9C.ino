#include <Wire.h>
#include <Adafruit_INA219.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

// ================= I2C =================
#define I2C_SDA 21
#define I2C_SCL 22

// ================= INA219 =================
#define INA_U_ADDR 0x40
#define INA_V_ADDR 0x41
#define INA_W_ADDR 0x44

Adafruit_INA219 inaU(INA_U_ADDR);
Adafruit_INA219 inaV(INA_V_ADDR);
Adafruit_INA219 inaW(INA_W_ADDR);

bool inaUReady = false;
bool inaVReady = false;
bool inaWReady = false;

// ================= LM35 =================
#define LM35_PIN 32

const float ADC_REF_VOLTAGE = 3.3;
const int ADC_MAX_VALUE = 4095;

// ================= ADXL345 =================
Adafruit_ADXL345_Unified adxl = Adafruit_ADXL345_Unified(12345);
bool adxlReady = false;

// ================= X9C103S =================
#define X9C_INC_PIN 18
#define X9C_UD_PIN  19
#define X9C_CS_PIN  23

int x9cCurrentStep = 0;

void x9cPulseINC() {
  digitalWrite(X9C_INC_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(X9C_INC_PIN, HIGH);
  delayMicroseconds(5);
}

void x9cMoveOneStep(bool increase) {
  digitalWrite(X9C_CS_PIN, LOW);
  digitalWrite(X9C_UD_PIN, increase ? HIGH : LOW);

  delayMicroseconds(5);

  x9cPulseINC();

  digitalWrite(X9C_CS_PIN, HIGH);

  delay(2);
}

void setX9CStep(int target) {

  target = constrain(target, 0, 99);

  while (x9cCurrentStep < target) {
    x9cMoveOneStep(true);
    x9cCurrentStep++;
  }

  while (x9cCurrentStep > target) {
    x9cMoveOneStep(false);
    x9cCurrentStep--;
  }
}

void resetX9C() {

  for (int i = 0; i < 100; i++) {
    x9cMoveOneStep(false);
  }

  x9cCurrentStep = 0;
}

void setup() {

  Serial.begin(115200);

  Wire.begin(I2C_SDA, I2C_SCL);

  analogReadResolution(12);
  analogSetPinAttenuation(LM35_PIN, ADC_11db);

  pinMode(X9C_INC_PIN, OUTPUT);
  pinMode(X9C_UD_PIN, OUTPUT);
  pinMode(X9C_CS_PIN, OUTPUT);

  digitalWrite(X9C_INC_PIN, HIGH);
  digitalWrite(X9C_CS_PIN, HIGH);

  // INA219
  inaUReady = inaU.begin();
  inaVReady = inaV.begin();
  inaWReady = inaW.begin();

  if (inaUReady) inaU.setCalibration_32V_2A();
  if (inaVReady) inaV.setCalibration_32V_2A();
  if (inaWReady) inaW.setCalibration_32V_2A();

  // ADXL345
  adxlReady = adxl.begin();

  if (adxlReady) {
    adxl.setRange(ADXL345_RANGE_16_G);
  }

  // X9C
  resetX9C();
  setX9CStep(50);

  Serial.println("===== TEST SENSOR =====");

  Serial.print("INA219 U : ");
  Serial.println(inaUReady ? "OK" : "NOT FOUND");

  Serial.print("INA219 V : ");
  Serial.println(inaVReady ? "OK" : "NOT FOUND");

  Serial.print("INA219 W : ");
  Serial.println(inaWReady ? "OK" : "NOT FOUND");

  Serial.print("ADXL345  : ");
  Serial.println(adxlReady ? "OK" : "NOT FOUND");

  Serial.println("Ketik:");
  Serial.println("POT=0 s/d POT=99");
}

void loop() {

  // ================= X9C COMMAND =================
  if (Serial.available()) {

    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.startsWith("POT=")) {

      int value = cmd.substring(4).toInt();

      setX9CStep(value);

      Serial.print("X9C Step = ");
      Serial.println(x9cCurrentStep);
    }
  }

  // ================= INA219 =================
  float VU = 0, IU = 0, PU = 0;
  float VV = 0, IV = 0, PV = 0;
  float VW = 0, IW = 0, PW = 0;

  if (inaUReady) {
    VU = inaU.getBusVoltage_V();
    IU = inaU.getCurrent_mA()/1000.0;
    PU = inaU.getPower_mW()/1000.0;
  }

  if (inaVReady) {
    VV = inaV.getBusVoltage_V();
    IV = inaV.getCurrent_mA()/1000.0;
    PV = inaV.getPower_mW()/1000.0;
  }

  if (inaWReady) {
    VW = inaW.getBusVoltage_V();
    IW = inaW.getCurrent_mA()/1000.0;
    PW = inaW.getPower_mW()/1000.0;
  }

  // ================= LM35 =================
  int adc = analogRead(LM35_PIN);

  float voltage = adc * ADC_REF_VOLTAGE / ADC_MAX_VALUE;
  float tempC = voltage * 100.0;

  // ================= ADXL345 =================
  float ax = 0;
  float ay = 0;
  float az = 0;
  float vibration = 0;

  if (adxlReady) {

    sensors_event_t event;
    adxl.getEvent(&event);

    ax = event.acceleration.x;
    ay = event.acceleration.y;
    az = event.acceleration.z;

    vibration = sqrt(
      ax * ax +
      ay * ay +
      az * az
    );
  }

  // ================= PRINT =================
  Serial.println();
  Serial.println("===== SENSOR DATA =====");

  Serial.printf("U : %.2f V | %.3f A | %.3f W\n", VU, IU, PU);
  Serial.printf("V : %.2f V | %.3f A | %.3f W\n", VV, IV, PV);
  Serial.printf("W : %.2f V | %.3f A | %.3f W\n", VW, IW, PW);

  Serial.printf("LM35 : %.2f C\n", tempC);

  Serial.printf("ADXL X : %.3f\n", ax);
  Serial.printf("ADXL Y : %.3f\n", ay);
  Serial.printf("ADXL Z : %.3f\n", az);

  Serial.printf("Getaran : %.3f\n", vibration);

  Serial.printf("X9C Step : %d\n", x9cCurrentStep);

  delay(2000);
}