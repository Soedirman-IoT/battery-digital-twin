#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include "esp_eap_client.h"
#include <ESP32Encoder.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

// =====================================================
// ESP2 - MOTOR + DST-WLTC CONTROLLER
// 1) Subscribe perintah dari ESP1: skripsi/motor01/cmd
// 2) Kontrol speed BLDC via X9C103S
// 3) Baca RPM + posisi encoder
// 4) Baca 3 INA219 pada fasa U/V/W:
//    - iu, iv, iw dari current INA219
//    - Vu, Vv, Vw dari bus/load voltage INA219
//    - Vuv, Vuw, Vvw dihitung dari selisih Vu/Vv/Vw
// 5) Baca suhu LM35 dan vibrasi ADXL345
// 6) Publish MQTT untuk dashboard + database
// =====================================================

// ================= WIFI WPA2 ENTERPRISE + MQTT CONFIG =================
const char* WIFI_SSID = "eduroam";

#define EAP_IDENTITY "dimas.anhar@mhs.unsoed.ac.id"
#define EAP_USERNAME "dimas.anhar@mhs.unsoed.ac.id"
#define EAP_PASSWORD "Raihananhar##23076"

const char* MQTT_HOST = "36fbe5a880964afa839f722d0eb4f7f5.s1.eu.hivemq.cloud";
const uint16_t MQTT_PORT = 8883;

const char* MQTT_CLIENT_ID = "esp32-motor-01";
const char* MQTT_USERNAME = "hivemq.webclient.1780374811805";
const char* MQTT_PASSWORD = "oHO:S4<Gqdcj#W839hQ>";

const char* MQTT_TOPIC_MOTOR_CMD    = "skripsi/motor01/cmd";
const char* MQTT_TOPIC_MOTOR_DATA   = "skripsi/motor01/data";
const char* MQTT_TOPIC_MOTOR_STATUS = "skripsi/motor01/status";
const char* MQTT_TOPIC_MEASUREMENT_STREAM = "measurement_stream";

unsigned long wifiDisconnectedSince = 0;
const unsigned long WIFI_RESTART_TIMEOUT_MS = 120000; // 2 menit
const unsigned long MQTT_PUBLISH_INTERVAL_MS = 1000;
const unsigned long WIFI_RECONNECT_INTERVAL_MS = 5000;
const unsigned long MQTT_RECONNECT_INTERVAL_MS = 5000;
const unsigned long MOTOR_CMD_TIMEOUT_MS = 10000;

WiFiClientSecure wifiClient;
PubSubClient mqttClient(wifiClient);

unsigned long lastWifiReconnectAttempt = 0;
unsigned long lastMqttReconnectAttempt = 0;
unsigned long lastMqttPublish = 0;
unsigned long lastMotorCmdTime = 0;

// ================= DIGITAL POTENTIOMETER X9C103S CONFIG =================
const int X9C_CS_PIN  = 18;
const int X9C_INC_PIN = 19;
const int X9C_UD_PIN  = 21;
const bool POT_STEP_HIGHER_MEANS_HIGHER_SPEED = false;
const int POT_MIN_STEP = 0;
const int POT_MAX_STEP = 99;
int currentPotStep = 0;
int targetPotStep = 0;

const int MOTOR_ENABLE_PIN = -1;
const bool MOTOR_ENABLE_ACTIVE_HIGH = true;

// ================= ENCODER CONFIG =================
const int ENCODER_A_PIN = 34;
const int ENCODER_B_PIN = 35;
const int PPR = 1000;
const int QUAD_MULTIPLIER = 4;
const int ENCODER_COUNTS_PER_REV = PPR * QUAD_MULTIPLIER;

ESP32Encoder encoder;
long lastEncoderCount = 0;
unsigned long lastRpmCalcMs = 0;
float rpmMeasured = 0.0;
float rpmFiltered = 0.0;
const float RPM_FILTER_ALPHA = 0.25;

long encoderCountNow = 0;
long encoderCountInOneRev = 0;
float positionDegree = 0.0;
float totalRevolution = 0.0;

// ================= SENSOR CONFIG =================
const int I2C_SDA_PIN = 26;
const int I2C_SCL_PIN = 27;

// Pastikan address INA219 berbeda via jumper A0/A1.
const uint8_t INA219_U_ADDR = 0x40;
const uint8_t INA219_V_ADDR = 0x41;
const uint8_t INA219_W_ADDR = 0x44;

Adafruit_INA219 ina219U(INA219_U_ADDR);
Adafruit_INA219 ina219V(INA219_V_ADDR);
Adafruit_INA219 ina219W(INA219_W_ADDR);

bool ina219UReady = false;
bool ina219VReady = false;
bool ina219WReady = false;

// Tegangan per fasa terhadap referensi/common yang sama.
// Data ini dipakai untuk menghitung estimasi tegangan antar fasa.
float phaseUVoltageAvg = 0.0;
float phaseVVoltageAvg = 0.0;
float phaseWVoltageAvg = 0.0;

// Arus line per fasa.
float phaseUCurrentAvg = 0.0;
float phaseVCurrentAvg = 0.0;
float phaseWCurrentAvg = 0.0;

float phaseUPowerAvg = 0.0;
float phaseVPowerAvg = 0.0;
float phaseWPowerAvg = 0.0;
float phaseTotalPowerAvg = 0.0;

float phaseUShuntMv = 0.0;
float phaseVShuntMv = 0.0;
float phaseWShuntMv = 0.0;

// Tegangan antar fasa hasil perhitungan selisih.
// Ini bukan differential measurement langsung, melainkan estimasi nilai rata-rata.
float vuvVoltageAvg = 0.0;
float vuwVoltageAvg = 0.0;
float vvwVoltageAvg = 0.0;

String motorCondition = "NORMAL";
// ambang perbedaan arus (A)
const float CURRENT_FAULT_THRESHOLD = 0.5;

// LM35 = 10 mV/°C. Gunakan ADC1 karena WiFi aktif.
const int LM35_PIN = 32;
const float ADC_REF_V = 3.30;
const float ADC_MAX_COUNT = 4095.0;
const int LM35_SAMPLE_COUNT = 60;
float motorDcTempC = 0.0;

Adafruit_ADXL345_Unified adxl = Adafruit_ADXL345_Unified(12345);
bool adxlReady = false;
float accelX = 0.0;
float accelY = 0.0;
float accelZ = 0.0;
float accelMagnitude = 0.0;
float vibrationRms = 0.0;
float vibrationPeak = 0.0;
float accelMagnitudeBaseline = 9.81;
const float VIBRATION_BASELINE_ALPHA = 0.01;

// ================= WLTC / DST PROFILE CONFIG =================
const float WLTC_MAX_SPEED_KMH = 131.0;
const float MOTOR_MAX_RPM = 4000.0;
const float MOTOR_MIN_RPM = 0.0;
const unsigned long WLTC_CYCLE_SECONDS = 1800;

struct WltcPoint {
  uint16_t t;
  float v;
};

const WltcPoint WLTC_PROFILE[] = {
  {0,0},{20,42},{50,12},{80,38},{105,0},{135,0},{160,32},{190,16},{225,56},{250,18},
  {280,48},{315,24},{350,28},{390,0},{420,30},{445,0},{500,0},{520,22},{550,0},{590,0},
  {620,45},{650,56},{675,12},{700,38},{730,14},{760,48},{790,65},{810,18},{850,58},{880,76},
  {910,60},{950,50},{975,25},{1000,0},{1040,0},{1060,52},{1080,12},{1110,65},{1140,15},{1160,30},
  {1190,82},{1220,94},{1250,98},{1290,92},{1320,74},{1345,82},{1370,60},{1400,28},{1425,52},{1450,0},
  {1480,0},{1510,72},{1530,60},{1550,98},{1570,124},{1600,106},{1625,116},{1645,104},{1670,126},{1700,128},
  {1725,131},{1750,90},{1775,60},{1800,0}
};
const int WLTC_POINT_COUNT = sizeof(WLTC_PROFILE) / sizeof(WLTC_PROFILE[0]);

// ================= COMMAND STATE FROM ESP1 =================
bool motorEnableFromEsp1 = false;
bool batterySafeFromEsp1 = false;
String loadStage = "REST";
String esp1SystemMode = "UNKNOWN";

bool motorActuallyEnabled = false;
float wltcSpeedKmh = 0.0;
float rpmSetpoint = 0.0;
unsigned long wltcSecond = 0;

// ================= HELPER =================
float clampFloat(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

int clampInt(int x, int lo, int hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

String extractJsonString(const String& src, const String& key, const String& fallback) {
  String pattern = "\"" + key + "\"";
  int keyIndex = src.indexOf(pattern);
  if (keyIndex < 0) return fallback;

  int colon = src.indexOf(':', keyIndex);
  if (colon < 0) return fallback;

  int firstQuote = src.indexOf('"', colon + 1);
  if (firstQuote < 0) return fallback;

  int secondQuote = src.indexOf('"', firstQuote + 1);
  if (secondQuote < 0) return fallback;

  return src.substring(firstQuote + 1, secondQuote);
}

bool extractJsonBool(const String& src, const String& key, bool fallback) {
  String pattern = "\"" + key + "\"";
  int keyIndex = src.indexOf(pattern);
  if (keyIndex < 0) return fallback;

  int colon = src.indexOf(':', keyIndex);
  if (colon < 0) return fallback;

  String tail = src.substring(colon + 1);
  tail.trim();
  if (tail.startsWith("true")) return true;
  if (tail.startsWith("false")) return false;
  return fallback;
}

void setMotorEnablePin(bool on) {
  motorActuallyEnabled = on;
  if (MOTOR_ENABLE_PIN < 0) return;
  digitalWrite(MOTOR_ENABLE_PIN, MOTOR_ENABLE_ACTIVE_HIGH ? (on ? HIGH : LOW) : (on ? LOW : HIGH));
}

// ================= X9C103S CONTROL =================
void x9cPulseInc() {
  digitalWrite(X9C_INC_PIN, HIGH);
  delayMicroseconds(5);
  digitalWrite(X9C_INC_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(X9C_INC_PIN, HIGH);
  delayMicroseconds(5);
}

void x9cMoveOneStep(bool increaseStep) {
  bool udHigh = POT_STEP_HIGHER_MEANS_HIGHER_SPEED ? increaseStep : !increaseStep;
  digitalWrite(X9C_UD_PIN, udHigh ? HIGH : LOW);
  delayMicroseconds(5);
  x9cPulseInc();
}

void x9cSetStep(int step) {
  step = clampInt(step, POT_MIN_STEP, POT_MAX_STEP);
  if (step == currentPotStep) return;

  digitalWrite(X9C_CS_PIN, LOW);
  delayMicroseconds(5);
  while (currentPotStep < step) {
    x9cMoveOneStep(true);
    currentPotStep++;
  }
  while (currentPotStep > step) {
    x9cMoveOneStep(false);
    currentPotStep--;
  }
  digitalWrite(X9C_CS_PIN, HIGH);
  delay(10);
}

void x9cResetToZero() {
  digitalWrite(X9C_CS_PIN, LOW);
  delayMicroseconds(5);
  for (int i = 0; i < 110; i++) x9cMoveOneStep(false);
  digitalWrite(X9C_CS_PIN, HIGH);
  delay(10);
  currentPotStep = POT_MIN_STEP;
}

// ================= WLTC PROFILE =================
float interpolateWltcSpeed(unsigned long secInCycle) {
  if (secInCycle >= WLTC_CYCLE_SECONDS) secInCycle %= WLTC_CYCLE_SECONDS;

  for (int i = 0; i < WLTC_POINT_COUNT - 1; i++) {
    uint16_t t0 = WLTC_PROFILE[i].t;
    uint16_t t1 = WLTC_PROFILE[i + 1].t;
    if (secInCycle >= t0 && secInCycle <= t1) {
      float v0 = WLTC_PROFILE[i].v;
      float v1 = WLTC_PROFILE[i + 1].v;
      if (t1 == t0) return v1;
      float ratio = (float)(secInCycle - t0) / (float)(t1 - t0);
      return v0 + ratio * (v1 - v0);
    }
  }
  return 0.0;
}

float mapSpeedToRpm(float speedKmh) {
  speedKmh = clampFloat(speedKmh, 0.0, WLTC_MAX_SPEED_KMH);
  return (speedKmh / WLTC_MAX_SPEED_KMH) * MOTOR_MAX_RPM;
}

int mapRpmToPotStep(float rpm) {
  rpm = clampFloat(rpm, MOTOR_MIN_RPM, MOTOR_MAX_RPM);
  float ratio = rpm / MOTOR_MAX_RPM;
  int step = round(POT_MIN_STEP + ratio * (POT_MAX_STEP - POT_MIN_STEP));
  return clampInt(step, POT_MIN_STEP, POT_MAX_STEP);
}

void updateWltcControl() {
  bool cmdTimeout = (lastMotorCmdTime == 0) || (millis() - lastMotorCmdTime > MOTOR_CMD_TIMEOUT_MS);
  bool allowedByEsp1 =
    !cmdTimeout &&
    motorEnableFromEsp1 &&
    batterySafeFromEsp1 &&
    loadStage != "REST" &&
    esp1SystemMode == "LOAD";

  if (!allowedByEsp1) {
    wltcSpeedKmh = 0.0;
    rpmSetpoint = 0.0;
    targetPotStep = POT_MIN_STEP;
    setMotorEnablePin(false);
    x9cSetStep(targetPotStep);
    return;
  }

  setMotorEnablePin(true);

  if (loadStage == "FULL_SPEED"){
    // Motor selalu berputar maksimum
    wltcSpeedKmh = WLTC_MAX_SPEED_KMH;   // hanya untuk tampilan Grafana
    rpmSetpoint = MOTOR_MAX_RPM;
  }
  else if (loadStage == "WLTC" || loadStage == "WLTC_DUMMY"){
    // Jalankan profil WLTC seperti biasa
    wltcSecond = (millis() / 1000UL) % WLTC_CYCLE_SECONDS;

    wltcSpeedKmh = interpolateWltcSpeed(wltcSecond);

    rpmSetpoint = mapSpeedToRpm(wltcSpeedKmh);
  }
  else{
    rpmSetpoint = 0;
    wltcSpeedKmh = 0;
  }

  targetPotStep = mapRpmToPotStep(rpmSetpoint);
  x9cSetStep(targetPotStep);
}

// ================= SENSOR READING =================
void setupIna219() {
  ina219UReady = ina219U.begin();
  ina219VReady = ina219V.begin();
  ina219WReady = ina219W.begin();

  if (ina219UReady) ina219U.setCalibration_32V_2A();
  if (ina219VReady) ina219V.setCalibration_32V_2A();
  if (ina219WReady) ina219W.setCalibration_32V_2A();

  Serial.print("INA219 U: "); Serial.println(ina219UReady ? "OK" : "NOT FOUND");
  Serial.print("INA219 V: "); Serial.println(ina219VReady ? "OK" : "NOT FOUND");
  Serial.print("INA219 W: "); Serial.println(ina219WReady ? "OK" : "NOT FOUND");
}

float readInaLoadVoltage(Adafruit_INA219& sensor) {
  float shuntMv = sensor.getShuntVoltage_mV();
  float busV = sensor.getBusVoltage_V();
  return busV + (shuntMv / 1000.0);
}

void updateIna219Sensors() {
  if (ina219UReady) {
    phaseUShuntMv = ina219U.getShuntVoltage_mV();
    phaseUVoltageAvg = ina219U.getBusVoltage_V() + (phaseUShuntMv / 1000.0);
    phaseUCurrentAvg = ina219U.getCurrent_mA() / 1000.0;
    phaseUPowerAvg = phaseUVoltageAvg * phaseUCurrentAvg;
  } else {
    phaseUShuntMv = phaseUVoltageAvg = phaseUCurrentAvg = phaseUPowerAvg = 0.0;
  }

  if (ina219VReady) {
    phaseVShuntMv = ina219V.getShuntVoltage_mV();
    phaseVVoltageAvg = ina219V.getBusVoltage_V() + (phaseVShuntMv / 1000.0);
    phaseVCurrentAvg = ina219V.getCurrent_mA() / 1000.0;
    phaseVPowerAvg = phaseVVoltageAvg * phaseVCurrentAvg;
  } else {
    phaseVShuntMv = phaseVVoltageAvg = phaseVCurrentAvg = phaseVPowerAvg = 0.0;
  }

  if (ina219WReady) {
    phaseWShuntMv = ina219W.getShuntVoltage_mV();
    phaseWVoltageAvg = ina219W.getBusVoltage_V() + (phaseWShuntMv / 1000.0);
    phaseWCurrentAvg = ina219W.getCurrent_mA() / 1000.0;
    phaseWPowerAvg = phaseWVoltageAvg * phaseWCurrentAvg;
  } else {
    phaseWShuntMv = phaseWVoltageAvg = phaseWCurrentAvg = phaseWPowerAvg = 0.0;
  }

  // Bagian yang diminta dosen:
  // Vuv, Vuw, Vvw diambil dari selisih tegangan rata-rata per fasa.
  // abs() dipakai agar nilai tegangan antar fasa selalu positif di dashboard.
  vuvVoltageAvg = fabs(phaseUVoltageAvg - phaseVVoltageAvg);
  vuwVoltageAvg = fabs(phaseUVoltageAvg - phaseWVoltageAvg);
  vvwVoltageAvg = fabs(phaseVVoltageAvg - phaseWVoltageAvg);

  phaseTotalPowerAvg = phaseUPowerAvg + phaseVPowerAvg + phaseWPowerAvg;
}

void updateMotorCondition() {

  float c_u = phaseUCurrentAvg;
  float c_v = phaseVCurrentAvg;
  float c_w = phaseWCurrentAvg;

  if ((c_u - c_v) > CURRENT_FAULT_THRESHOLD) {

    motorCondition = "UV Fault";

  }
  else if ((c_v - c_w) > CURRENT_FAULT_THRESHOLD) {

    motorCondition = "VW Fault";

  }
  else if ((c_w - c_u) > CURRENT_FAULT_THRESHOLD) {

    motorCondition = "WU Fault";

  }
  else {

    motorCondition = "NORMAL";

  }
}

void updateLm35Sensor() {
  uint32_t rawSum = 0;
  for (int i = 0; i < LM35_SAMPLE_COUNT; i++) {
    rawSum += analogRead(LM35_PIN);
    delayMicroseconds(5000);
  }
  float rawAvg = (float)rawSum / (float)LM35_SAMPLE_COUNT;
  float voltage = (rawAvg / ADC_MAX_COUNT) * ADC_REF_V;
  float tempBaru = (voltage * 100.0) + 13.2;

  motorDcTempC = tempBaru;
}

void setupAdxl345() {
  adxlReady = adxl.begin();
  if (!adxlReady) {
    Serial.println("ADXL345 not detected. Check wiring/address.");
    return;
  }
  adxl.setRange(ADXL345_RANGE_16_G);
  Serial.println("ADXL345 detected and initialized.");
}

void updateAdxl345Sensor() {
  if (!adxlReady) {
    accelX = accelY = accelZ = accelMagnitude = vibrationRms = vibrationPeak = 0.0;
    return;
  }

  const int sampleCount = 25;
  float sumSq = 0.0;
  float peak = 0.0;
  sensors_event_t event;

  for (int i = 0; i < sampleCount; i++) {
    adxl.getEvent(&event);
    float x = event.acceleration.x;
    float y = event.acceleration.y;
    float z = event.acceleration.z;
    float mag = sqrt(x * x + y * y + z * z);
    float vib = mag - accelMagnitudeBaseline;

    sumSq += vib * vib;
    if (fabs(vib) > peak) peak = fabs(vib);

    if (i == sampleCount - 1) {
      accelX = x;
      accelY = y;
      accelZ = z;
      accelMagnitude = mag;
    }
    delayMicroseconds(1000);
  }

  accelMagnitudeBaseline =
    (VIBRATION_BASELINE_ALPHA * accelMagnitude) +
    ((1.0 - VIBRATION_BASELINE_ALPHA) * accelMagnitudeBaseline);

  vibrationRms = sqrt(sumSq / (float)sampleCount);
  vibrationPeak = peak;
}

void updateEncoderPositionFromCount(long countNow) {
  encoderCountNow = countNow;
  totalRevolution = (float)encoderCountNow / (float)ENCODER_COUNTS_PER_REV;

  long countMod = encoderCountNow % ENCODER_COUNTS_PER_REV;
  if (countMod < 0) countMod += ENCODER_COUNTS_PER_REV;
  encoderCountInOneRev = countMod;
  positionDegree = ((float)encoderCountInOneRev * 360.0) / (float)ENCODER_COUNTS_PER_REV;
}

void resetEncoderPosition() {
  encoder.clearCount();
  lastEncoderCount = 0;
  lastRpmCalcMs = 0;
  rpmMeasured = 0.0;
  rpmFiltered = 0.0;
  updateEncoderPositionFromCount(0);
}

void updateRpm() {
  unsigned long now = millis();
  long countNow = encoder.getCount();
  updateEncoderPositionFromCount(countNow);

  if (lastRpmCalcMs == 0) {
    lastRpmCalcMs = now;
    lastEncoderCount = countNow;
    return;
  }

  unsigned long dt = now - lastRpmCalcMs;
  if (dt < 500) return;

  long diff = countNow - lastEncoderCount;
  float rev = (float)diff / (float)ENCODER_COUNTS_PER_REV;
  rpmMeasured = (rev * 60000.0) / (float)dt;
  rpmFiltered = RPM_FILTER_ALPHA * rpmMeasured + (1.0 - RPM_FILTER_ALPHA) * rpmFiltered;

  lastEncoderCount = countNow;
  lastRpmCalcMs = now;
}

bool isSlipOrLoadAnomaly() {
  if (!motorActuallyEnabled) return false;
  if (rpmSetpoint < 300.0) return false;
  return fabs(rpmFiltered) < (0.35 * rpmSetpoint);
}

// ================= WIFI + MQTT =================
void setupWiFi() {
  Serial.println();
  Serial.println("Connecting to WPA2-Enterprise WiFi...");

  WiFi.disconnect(true);
  delay(1000);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  esp_eap_client_set_identity((uint8_t*)EAP_IDENTITY, strlen(EAP_IDENTITY));
  esp_eap_client_set_username((uint8_t*)EAP_USERNAME, strlen(EAP_USERNAME));
  esp_eap_client_set_password((uint8_t*)EAP_PASSWORD, strlen(EAP_PASSWORD));
  esp_wifi_sta_enterprise_enable();
  WiFi.begin(WIFI_SSID);

  Serial.print("Connecting WiFi");
  unsigned long startAttempt = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 20000) {
    Serial.print(".");
    delay(500);
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.print("WiFi connected. IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println();
    Serial.println("WiFi not connected. Motor remains safe until MQTT command is received.");
  }
}

void maintainWiFi() {

  // Jika WiFi sudah tersambung
  if (WiFi.status() == WL_CONNECTED) {

    wifiDisconnectedSince = 0;
    return;
  }

  // Catat kapan mulai disconnect
  if (wifiDisconnectedSince == 0) {
    wifiDisconnectedSince = millis();
    Serial.println("WiFi disconnected!");
  }

  unsigned long now = millis();

  // Coba reconnect setiap 5 detik
  if (now - lastWifiReconnectAttempt >= WIFI_RECONNECT_INTERVAL_MS) {

    lastWifiReconnectAttempt = now;

    Serial.println("Reconnecting WiFi...");

    WiFi.disconnect(true);
    delay(500);

    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);

    // Konfigurasi ulang WPA2 Enterprise
    esp_eap_client_set_identity((uint8_t*)EAP_IDENTITY, strlen(EAP_IDENTITY));
    esp_eap_client_set_username((uint8_t*)EAP_USERNAME, strlen(EAP_USERNAME));
    esp_eap_client_set_password((uint8_t*)EAP_PASSWORD, strlen(EAP_PASSWORD));
    esp_wifi_sta_enterprise_enable();

    WiFi.begin(WIFI_SSID);
  }

  // Sudah gagal reconnect lebih dari 2 menit?
  if (now - wifiDisconnectedSince >= WIFI_RESTART_TIMEOUT_MS) {

    Serial.println("==================================");
    Serial.println("WiFi reconnect timeout!");
    Serial.println("Restarting ESP32...");
    Serial.println("==================================");

    delay(1000);

    ESP.restart();
  }
}

void onMqttMessage(char* topic, byte* payload, unsigned int length) {
  if (String(topic) != MQTT_TOPIC_MOTOR_CMD) return;

  char msg[512];
  unsigned int copyLen = length;
  if (copyLen >= sizeof(msg)) copyLen = sizeof(msg) - 1;
  memcpy(msg, payload, copyLen);
  msg[copyLen] = '\0';

  String command = String(msg);
  command.trim();

  motorEnableFromEsp1 = extractJsonBool(command, "motor_enable", motorEnableFromEsp1);
  batterySafeFromEsp1 = extractJsonBool(command, "battery_safe", batterySafeFromEsp1);
  loadStage = extractJsonString(command, "load_stage", loadStage);
  esp1SystemMode = extractJsonString(command, "system_mode", esp1SystemMode);
  loadStage.toUpperCase();
  esp1SystemMode.toUpperCase();

  if (extractJsonBool(command, "reset_position", false)) {
    resetEncoderPosition();
    Serial.println("Encoder position reset to 0 degree.");
  }

  lastMotorCmdTime = millis();
  Serial.println("MQTT CMD from ESP1:");
  Serial.println(command);
}

void setupMQTT() {
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setBufferSize(2048);
  mqttClient.setCallback(onMqttMessage);
}

void maintainMQTT() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (mqttClient.connected()) {
    mqttClient.loop();
    return;
  }

  unsigned long now = millis();
  if (now - lastMqttReconnectAttempt < MQTT_RECONNECT_INTERVAL_MS) return;
  lastMqttReconnectAttempt = now;

  Serial.print("Connecting MQTT... ");
  bool connected = mqttClient.connect(
    MQTT_CLIENT_ID,
    MQTT_USERNAME,
    MQTT_PASSWORD,
    MQTT_TOPIC_MOTOR_STATUS,
    1,
    true,
    "offline"
  );

  if (connected) {
    Serial.println("connected.");
    mqttClient.publish(MQTT_TOPIC_MOTOR_STATUS, "online", true);
    mqttClient.subscribe(MQTT_TOPIC_MOTOR_CMD);
  } else {
    Serial.print("failed, rc=");
    Serial.println(mqttClient.state());
  }
}

void publishMotorData() {
  if (!mqttClient.connected()) return;

  unsigned long now = millis();
  if (now - lastMqttPublish < MQTT_PUBLISH_INTERVAL_MS) return;
  lastMqttPublish = now;

  bool cmdTimeout = (lastMotorCmdTime == 0) || (now - lastMotorCmdTime > MOTOR_CMD_TIMEOUT_MS);

  char payload[2600];
  snprintf(
    payload,
    sizeof(payload),
    "{"
      "\"timestamp_ms\":%lu,"
      "\"motor_cmd_timeout\":%s,"
      "\"motor_enable_from_esp1\":%s,"
      "\"battery_safe_from_esp1\":%s,"
      "\"motor_actually_enabled\":%s,"
      "\"load_stage\":\"%s\","
      "\"esp1_system_mode\":\"%s\","
      "\"wltc_second\":%lu,"
      "\"wltc_speed_kmh\":%.2f,"
      "\"rpm_setpoint\":%.1f,"
      "\"pot_step\":%d,"
      "\"rpm_measured\":%.1f,"
      "\"rpm_filtered\":%.1f,"
      "\"encoder_count\":%ld,"
      "\"encoder_count_in_one_rev\":%ld,"
      "\"position_degree\":%.2f,"
      "\"total_revolution\":%.4f,"
      "\"ina219_u_ready\":%s,"
      "\"ina219_v_ready\":%s,"
      "\"ina219_w_ready\":%s,"
      "\"phase_u_voltage_avg\":%.3f,"
      "\"phase_v_voltage_avg\":%.3f,"
      "\"phase_w_voltage_avg\":%.3f,"
      "\"phase_u_current_avg\":%.3f,"
      "\"phase_v_current_avg\":%.3f,"
      "\"phase_w_current_avg\":%.3f,"
      "\"phase_u_power_avg\":%.3f,"
      "\"phase_v_power_avg\":%.3f,"
      "\"phase_w_power_avg\":%.3f,"
      "\"phase_total_power_avg\":%.3f,"
      "\"phase_u_shunt_mv\":%.3f,"
      "\"phase_v_shunt_mv\":%.3f,"
      "\"phase_w_shunt_mv\":%.3f,"
      "\"vuv_voltage_avg\":%.3f,"
      "\"vuw_voltage_avg\":%.3f,"
      "\"vvw_voltage_avg\":%.3f,"
      "\"motor_dc_temp_c\":%.2f,"
      "\"adxl_ready\":%s,"
      "\"accel_x\":%.3f,"
      "\"accel_y\":%.3f,"
      "\"accel_z\":%.3f,"
      "\"accel_magnitude\":%.3f,"
      "\"vibration_rms\":%.4f,"
      "\"vibration_peak\":%.4f,"
      "\"slip_or_load_anomaly\":%s"
    "}",
    now,
    cmdTimeout ? "true" : "false",
    motorEnableFromEsp1 ? "true" : "false",
    batterySafeFromEsp1 ? "true" : "false",
    motorActuallyEnabled ? "true" : "false",
    loadStage.c_str(),
    esp1SystemMode.c_str(),
    wltcSecond,
    wltcSpeedKmh,
    rpmSetpoint,
    currentPotStep,
    rpmMeasured,
    rpmFiltered,
    encoderCountNow,
    encoderCountInOneRev,
    positionDegree,
    totalRevolution,
    ina219UReady ? "true" : "false",
    ina219VReady ? "true" : "false",
    ina219WReady ? "true" : "false",
    phaseUVoltageAvg,
    phaseVVoltageAvg,
    phaseWVoltageAvg,
    phaseUCurrentAvg,
    phaseVCurrentAvg,
    phaseWCurrentAvg,
    phaseUPowerAvg,
    phaseVPowerAvg,
    phaseWPowerAvg,
    phaseTotalPowerAvg,
    phaseUShuntMv,
    phaseVShuntMv,
    phaseWShuntMv,
    vuvVoltageAvg,
    vuwVoltageAvg,
    vvwVoltageAvg,
    motorDcTempC,
    adxlReady ? "true" : "false",
    accelX,
    accelY,
    accelZ,
    accelMagnitude,
    vibrationRms,
    vibrationPeak,
    isSlipOrLoadAnomaly() ? "true" : "false"
  );

  bool ok = mqttClient.publish(MQTT_TOPIC_MOTOR_DATA, payload, false);
  Serial.print("MQTT motor publish: ");
  Serial.println(ok ? "OK" : "FAILED");

  char measurementPayload[512];
  snprintf(
    measurementPayload,
    sizeof(measurementPayload),
    "{"
      "\"timestamp\":%lu,"
      "\"motor_condition\":\"%s\","
      "\"speed\":%.1f,"
      "\"temperature\":%.2f,"
      "\"current_u\":%.3f,"
      "\"current_v\":%.3f,"
      "\"current_w\":%.3f,"
      "\"voltage_u\":%.3f,"
      "\"voltage_v\":%.3f,"
      "\"voltage_w\":%.3f,"
      "\"vibration_x\":%.3f,"
      "\"vibration_y\":%.3f,"
      "\"vibration_z\":%.3f"
    "}",
    millis(),
    motorCondition.c_str(),
    rpmFiltered,
    motorDcTempC,
    phaseUCurrentAvg,
    phaseVCurrentAvg,
    phaseWCurrentAvg,
    phaseUVoltageAvg,
    phaseVVoltageAvg,
    phaseWVoltageAvg,
    accelX,
    accelY,
    accelZ
  );

  bool ok2 = mqttClient.publish(MQTT_TOPIC_MEASUREMENT_STREAM, measurementPayload, false);
  Serial.print("MQTT MEASUREMENT_STREAM publish: ");
  Serial.println(ok ? "OK" : "FAILED");
}

void printMotorData() {
  Serial.println();
  Serial.println("===== ESP2 MOTOR + DST-WLTC =====");
  Serial.print("WiFi                  : "); Serial.println(WiFi.status() == WL_CONNECTED ? "CONNECTED" : "DISCONNECTED");
  Serial.print("MQTT                  : "); Serial.println(mqttClient.connected() ? "CONNECTED" : "DISCONNECTED");
  Serial.print("Motor CMD Timeout     : "); Serial.println(((lastMotorCmdTime == 0) || (millis() - lastMotorCmdTime > MOTOR_CMD_TIMEOUT_MS)) ? "YES" : "NO");
  Serial.print("ESP1 Motor Enable     : "); Serial.println(motorEnableFromEsp1 ? "YES" : "NO");
  Serial.print("ESP1 Battery Safe     : "); Serial.println(batterySafeFromEsp1 ? "YES" : "NO");
  Serial.print("Load Stage            : "); Serial.println(loadStage);
  Serial.print("ESP1 System Mode      : "); Serial.println(esp1SystemMode);
  Serial.print("Motor Actually Enabled: "); Serial.println(motorActuallyEnabled ? "YES" : "NO");
  Serial.print("WLTC Second           : "); Serial.println(wltcSecond);
  Serial.print("WLTC Speed            : "); Serial.print(wltcSpeedKmh, 2); Serial.println(" km/h");
  Serial.print("RPM Setpoint          : "); Serial.println(rpmSetpoint, 1);
  Serial.print("Pot Step              : "); Serial.println(currentPotStep);
  Serial.print("RPM Filtered          : "); Serial.println(rpmFiltered, 1);
  Serial.print("Position Degree       : "); Serial.print(positionDegree, 2); Serial.println(" deg");

  Serial.println("--- INA219 phase average data ---");
  Serial.print("Vu/Vv/Vw              : "); Serial.print(phaseUVoltageAvg, 3); Serial.print(" / "); Serial.print(phaseVVoltageAvg, 3); Serial.print(" / "); Serial.println(phaseWVoltageAvg, 3);
  Serial.print("iu/iv/iw              : "); Serial.print(phaseUCurrentAvg, 3); Serial.print(" / "); Serial.print(phaseVCurrentAvg, 3); Serial.print(" / "); Serial.println(phaseWCurrentAvg, 3);
  Serial.print("Vuv/Vuw/Vvw           : "); Serial.print(vuvVoltageAvg, 3); Serial.print(" / "); Serial.print(vuwVoltageAvg, 3); Serial.print(" / "); Serial.println(vvwVoltageAvg, 3);
  Serial.print("Ptotal avg            : "); Serial.print(phaseTotalPowerAvg, 3); Serial.println(" W");

  Serial.print("Motor DC Temp LM35    : "); Serial.print(motorDcTempC, 2); Serial.println(" C");
  Serial.print("Vibration RMS         : "); Serial.print(vibrationRms, 4); Serial.println(" m/s^2");
  Serial.print("Slip/Load Anomaly     : "); Serial.println(isSlipOrLoadAnomaly() ? "YES" : "NO");
  Serial.println("=================================");
}

// ================= SETUP + LOOP =================
void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(X9C_CS_PIN, OUTPUT);
  pinMode(X9C_INC_PIN, OUTPUT);
  pinMode(X9C_UD_PIN, OUTPUT);
  digitalWrite(X9C_CS_PIN, HIGH);
  digitalWrite(X9C_INC_PIN, HIGH);
  digitalWrite(X9C_UD_PIN, LOW);

  if (MOTOR_ENABLE_PIN >= 0) {
    pinMode(MOTOR_ENABLE_PIN, OUTPUT);
    setMotorEnablePin(false);
  }

  pinMode(LM35_PIN, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(LM35_PIN, ADC_11db);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  setupIna219();
  setupAdxl345();

  ESP32Encoder::useInternalWeakPullResistors = puType::none;
  encoder.attachFullQuad(ENCODER_A_PIN, ENCODER_B_PIN);
  resetEncoderPosition();

  x9cResetToZero();

  Serial.println("ESP2 MOTOR CONTROLLER - DST-WLTC + X9C103S + 3xINA219 + LM35 + ADXL345 + MQTT");

  wifiClient.setInsecure();
  setupWiFi();
  setupMQTT();
}

void loop() {
  static unsigned long lastControl = 0;
  static unsigned long lastSensor = 0;
  static unsigned long lastPrint = 0;

  maintainWiFi();
  maintainMQTT();

  unsigned long now = millis();

  if (now - lastSensor >= 200) {
    lastSensor = now;
    updateRpm();
    updateIna219Sensors();
    updateMotorCondition();
    updateLm35Sensor();
    updateAdxl345Sensor();
  }

  if (now - lastControl >= 1000) {
    lastControl = now;
    updateWltcControl();
  }

  publishMotorData();

  if (now - lastPrint >= 5000) {
    lastPrint = now;
    printMotorData();
  }
}
