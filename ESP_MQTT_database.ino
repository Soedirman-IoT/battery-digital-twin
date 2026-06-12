#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <NimBLEDevice.h>
#include <ESP32Encoder.h>
#include "esp_eap_client.h"
#include <time.h>

// =====================================================
// WIFI WPA2 ENTERPRISE + MQTT CONFIG
// =====================================================
const char* WIFI_SSID = "eduroam";

#define EAP_IDENTITY "email"
#define EAP_USERNAME "email"
#define EAP_PASSWORD "######"

const char* MQTT_HOST = "36fbe5a880964afa839f722d0eb4f7f5.s1.eu.hivemq.cloud";
const uint16_t MQTT_PORT = 8883;

const char* MQTT_CLIENT_ID = "esp32-bms-01";
const char* MQTT_USERNAME = "hivemq.webclient.1780374811805";
const char* MQTT_PASSWORD = "oHO:S4<Gqdcj#W839hQ>";

const char* MQTT_TOPIC_DATA   = "skripsi/bms01/data";
const char* MQTT_TOPIC_STATUS = "skripsi/bms01/status";
const char* MQTT_TOPIC_CONTROL = "skripsi/bms01/control";

const unsigned long MQTT_PUBLISH_INTERVAL_MS = 1000;
const unsigned long WIFI_RECONNECT_INTERVAL_MS = 5000;
const unsigned long MQTT_RECONNECT_INTERVAL_MS = 5000;

// ================= CONTROL MODE CONFIG =================
// AUTO   : jam 07:00-19:00 beban/motor boleh ON.
//          di luar jam itu beban/motor OFF, tetapi charge tetap boleh ON jika baterai drop.
// MANUAL : sistem mengikuti tombol ON/OFF dari MQTT.
enum ControlMode {
  CONTROL_AUTO,
  CONTROL_MANUAL
};

ControlMode controlMode = CONTROL_AUTO;
bool manualPower = false;
bool scheduleActive = false;
bool systemAllowed = false;

// loadAllowed  : izin untuk menyalakan relay beban/motor.
// chargeAllowed: izin untuk menyalakan relay charge.
// Pada AUTO malam, loadAllowed=false tetapi chargeAllowed=true,
// sehingga motor OFF tetapi baterai tetap bisa recovery charge jika drop.
bool loadAllowed = false;
bool chargeAllowed = false;
bool nightChargeMode = false;

const int SCHEDULE_START_HOUR = 7;
const int SCHEDULE_STOP_HOUR  = 19;
const long GMT_OFFSET_SEC = 7 * 3600;
const int DAYLIGHT_OFFSET_SEC = 0;

const char* controlModeToString() {
  switch (controlMode) {
    case CONTROL_AUTO: return "AUTO";
    case CONTROL_MANUAL: return "MANUAL";
    default: return "UNKNOWN";
  }
}

void updateScheduleState() {
  struct tm timeinfo;

  if (!getLocalTime(&timeinfo, 100)) {
    scheduleActive = false;
    return;
  }

  int hour = timeinfo.tm_hour;
  scheduleActive = (hour >= SCHEDULE_START_HOUR && hour < SCHEDULE_STOP_HOUR);
}

void updateSystemAllowed() {
  updateScheduleState();

  if (controlMode == CONTROL_AUTO) {
    // Siang: beban/motor boleh jalan, charge juga boleh jalan jika baterai drop.
    // Malam: beban/motor OFF, tetapi charge tetap boleh jalan jika baterai drop.
    loadAllowed = scheduleActive;
    chargeAllowed = true;
    nightChargeMode = !scheduleActive;
  } else {
    // Manual OFF benar-benar mematikan sistem.
    // Manual ON memberi izin beban dan charge, tetapi safety BMS tetap berlaku.
    loadAllowed = manualPower;
    chargeAllowed = manualPower;
    nightChargeMode = false;
  }

  systemAllowed = loadAllowed || chargeAllowed;
}


WiFiClientSecure wifiClient;
PubSubClient mqttClient(wifiClient);

unsigned long lastWifiReconnectAttempt = 0;
unsigned long lastMqttReconnectAttempt = 0;
unsigned long lastMqttPublish = 0;

// ================= BMS BLE CONFIG =================
const char* TARGET_ADDR = "20:25:03:32:08:0a";

NimBLEClient* client = nullptr;
NimBLERemoteCharacteristic* dataChar = nullptr;

const int CELL_COUNT = 6;

// ================= ENCODER CONFIG =================
#define ENCODER_A 34
#define ENCODER_B 35

ESP32Encoder encoder;

long last_count = 0;
unsigned long last_encoder_time = 0;

float rpmRaw = 0.0;
float rpm = 0.0;
float positionDeg = 0.0;

const int PPR = 1000;
const int QUAD_MULTIPLIER = 4;
const float RPM_FILTER_ALPHA = 0.25;

// ================= PIN RELAY =================
const int RELAY_LOAD_PIN   = 26;
const int RELAY_CHARGE_PIN = 27;

const bool RELAY_ACTIVE_LOW = true;

// ================= BATTERY DATA =================
float packVoltage = 0.0;
float cellVoltage[CELL_COUNT] = {0};
float minCellVoltage = 0.0;
float maxCellVoltage = 0.0;
int minCellIndex = -1;
int maxCellIndex = -1;

float batteryCurrent = 0.0;
float balanceCurrent = 0.0;

float batteryT1 = 0.0;
float batteryT2 = 0.0;
float mosTemp = 0.0;

bool voltageValid = false;
bool currentValid = false;
bool balanceCurrentValid = false;
bool tempValid = false;

// ================= SAFETY TIMEOUT =================
unsigned long lastBMSDataTime = 0;
const unsigned long BMS_TIMEOUT_MS = 10000;

// ================= RELAY STATE =================
bool loadRelayOn = false;
bool chargeRelayOn = false;

// ================= SYSTEM MODE =================
enum SystemMode {
  MODE_LOAD,
  MODE_CHARGE,
  MODE_TRANSITION_TO_LOAD,
  MODE_TRANSITION_TO_CHARGE,
  MODE_SAFE_OFF
};

SystemMode systemMode = MODE_SAFE_OFF;

// ================= LIMIT TEGANGAN =================
const float CHARGE_START_CELL_V = 3.10;
const float CHARGE_STOP_CELL_V  = 4.15;

const float PACK_CHARGE_START_V = 18.6;
const float PACK_CHARGE_STOP_V  = 24.6;

const float TEMP_OFF_C = 55.0;

const unsigned long RELAY_SWITCH_DELAY_MS = 3000;

// Minimum waktu charge.
// Tujuannya agar setelah relay charge sempat ON, sistem tidak langsung pindah ke relay beban
// hanya karena tegangan baterai naik sesaat / voltage rebound.
const unsigned long MIN_CHARGE_TIME_MS = 5UL * 60UL * 1000UL;  // 5 menit

// Konfirmasi tegangan.
// Tujuannya agar relay tidak pindah mode hanya karena noise sesaat atau voltage rebound.
// LOW dikonfirmasi lebih cepat agar baterai segera dilindungi.
// FULL dikonfirmasi lebih lama agar charge tidak mati karena spike tegangan sesaat.
const unsigned long LOW_VOLTAGE_CONFIRM_MS  = 3000;
const unsigned long FULL_VOLTAGE_CONFIRM_MS = 10000;

unsigned long transitionStart = 0;
unsigned long chargeModeStart = 0;
bool chargeMinTimerActive = false;

// Latch/lock mode charge.
// Jika baterai sudah dikonfirmasi LOW, beban dikunci OFF sampai baterai benar-benar FULL.
// Ini inti perbaikan untuk mencegah relay beban ON-OFF karena voltage rebound.
bool chargeLockActive = false;
bool batteryLowRaw = false;
bool batteryFullRaw = false;
bool batteryLowConfirmed = false;
bool batteryFullConfirmed = false;
unsigned long batteryLowSince = 0;
unsigned long batteryFullSince = 0;

// ================= BMS REQUEST FRAME =================
uint8_t requestSettings[] = {
  0xAA, 0x55, 0x90, 0xEB, 0x96, 0x00, 0x79, 0x62, 0x96, 0xED,
  0xE3, 0xD0, 0x82, 0xA1, 0x9B, 0x5B, 0x3C, 0x9C, 0x4B, 0x5D
};

uint8_t requestDeviceInfo[] = {
  0xAA, 0x55, 0x90, 0xEB, 0x97, 0x00, 0xDF, 0x52, 0x88, 0x67,
  0x9D, 0x0A, 0x09, 0x6B, 0x9A, 0xF6, 0x70, 0x9A, 0x17, 0xFD
};

// ================= HELPER =================
uint16_t readUInt16LE(uint8_t* data, int index) {
  return (uint16_t)data[index] | ((uint16_t)data[index + 1] << 8);
}

int16_t readInt16LE(uint8_t* data, int index) {
  return (int16_t)((uint16_t)data[index] | ((uint16_t)data[index + 1] << 8));
}

bool isReasonableCellVoltage(float v) {
  return v >= 2.0 && v <= 4.5;
}

bool isReasonablePackVoltage(float v) {
  return v >= 10.0 && v <= 30.0;
}

bool isReasonableCurrent(float a) {
  return a >= -100.0 && a <= 100.0;
}

bool isReasonableTemp(float t) {
  return t >= 10.0 && t <= 80.0;
}

bool isBMSTimeout() {
  if (lastBMSDataTime == 0) return true;
  return millis() - lastBMSDataTime > BMS_TIMEOUT_MS;
}

const char* systemModeToString() {
  switch (systemMode) {
    case MODE_LOAD: return "LOAD";
    case MODE_CHARGE: return "CHARGE";
    case MODE_TRANSITION_TO_LOAD: return "TRANSITION_TO_LOAD";
    case MODE_TRANSITION_TO_CHARGE: return "TRANSITION_TO_CHARGE";
    case MODE_SAFE_OFF: return "SAFE_OFF";
    default: return "UNKNOWN";
  }
}

bool isMinimumChargeTimeDone() {
  if (!chargeMinTimerActive) return true;
  return millis() - chargeModeStart >= MIN_CHARGE_TIME_MS;
}

unsigned long getRemainingMinChargeSeconds() {
  if (!chargeMinTimerActive) return 0;

  unsigned long elapsed = millis() - chargeModeStart;
  if (elapsed >= MIN_CHARGE_TIME_MS) return 0;

  return (MIN_CHARGE_TIME_MS - elapsed) / 1000UL;
}

void updateBatteryThresholdState() {
  unsigned long now = millis();

  batteryLowRaw =
    minCellVoltage <= CHARGE_START_CELL_V ||
    packVoltage <= PACK_CHARGE_START_V;

  batteryFullRaw =
    maxCellVoltage >= CHARGE_STOP_CELL_V ||
    packVoltage >= PACK_CHARGE_STOP_V;

  if (batteryLowRaw) {
    if (batteryLowSince == 0) batteryLowSince = now;
  } else {
    batteryLowSince = 0;
  }

  if (batteryFullRaw) {
    if (batteryFullSince == 0) batteryFullSince = now;
  } else {
    batteryFullSince = 0;
  }

  batteryLowConfirmed =
    batteryLowRaw &&
    batteryLowSince != 0 &&
    now - batteryLowSince >= LOW_VOLTAGE_CONFIRM_MS;

  batteryFullConfirmed =
    batteryFullRaw &&
    batteryFullSince != 0 &&
    now - batteryFullSince >= FULL_VOLTAGE_CONFIRM_MS;

  // Begitu LOW valid, kunci sistem ke charge.
  // Jangan buka lock hanya karena tegangan naik sesaat setelah beban OFF.
  if (batteryLowConfirmed) {
    chargeLockActive = true;
  }

  // Lock baru dilepas kalau FULL sudah stabil.
  if (batteryFullConfirmed) {
    chargeLockActive = false;
    chargeMinTimerActive = false;
  }
}

// ================= WIFI + MQTT FUNCTION =================
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
    Serial.println("WiFi not connected. System still runs, MQTT offline.");
  }
}

void maintainWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;

  unsigned long now = millis();
  if (now - lastWifiReconnectAttempt >= WIFI_RECONNECT_INTERVAL_MS) {
    lastWifiReconnectAttempt = now;
    Serial.println("Reconnecting WPA2-Enterprise WiFi...");

    WiFi.disconnect(false);
    delay(500);

    WiFi.begin(WIFI_SSID);
  }
}


void onMqttMessage(char* topic, byte* payload, unsigned int length) {
  if (String(topic) != MQTT_TOPIC_CONTROL) return;

  ControlMode previousControlMode = controlMode;
  bool previousManualPower = manualPower;

  char msg[160];
  unsigned int copyLen = length;

  if (copyLen >= sizeof(msg)) {
    copyLen = sizeof(msg) - 1;
  }

  memcpy(msg, payload, copyLen);
  msg[copyLen] = '\0';

  String command = String(msg);
  command.toLowerCase();

  // Format utama:
  // {"mode":"auto"}
  // {"mode":"manual","power":true}
  // {"mode":"manual","power":false}
  //
  // Format tambahan yang tetap diterima:
  // {"mode":"toggle"}
  // on
  // off

  if (command.indexOf("auto") >= 0) {
    controlMode = CONTROL_AUTO;
  } else if (command.indexOf("manual") >= 0) {
    controlMode = CONTROL_MANUAL;

    if (command.indexOf("true") >= 0 || command.indexOf("on") >= 0) {
      manualPower = true;
    } else if (command.indexOf("false") >= 0 || command.indexOf("off") >= 0) {
      manualPower = false;
    }
  } else if (command.indexOf("toggle") >= 0) {
    controlMode = CONTROL_MANUAL;
    manualPower = !manualPower;
  } else if (command.indexOf("on") >= 0) {
    controlMode = CONTROL_MANUAL;
    manualPower = true;
  } else if (command.indexOf("off") >= 0) {
    controlMode = CONTROL_MANUAL;
    manualPower = false;
  }

  updateSystemAllowed();

  // Saat pindah mode, matikan relay dulu dan paksa state machine evaluasi ulang dari kondisi aman.
  // Ini mencegah relay beban mempertahankan state lama ketika pindah MANUAL -> AUTO.
  if (previousControlMode != controlMode || previousManualPower != manualPower) {
    systemMode = MODE_SAFE_OFF;
    allRelayOff();
    transitionStart = millis();
  }

  if (!systemAllowed) {
    systemMode = MODE_SAFE_OFF;
    allRelayOff();
  }

  Serial.print("Control Mode: ");
  Serial.print(controlModeToString());
  Serial.print(" | Manual Power: ");
  Serial.println(manualPower ? "ON" : "OFF");

  mqttClient.publish(MQTT_TOPIC_STATUS, controlModeToString(), true);
}


void setupMQTT() {
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setBufferSize(1024);
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
    MQTT_TOPIC_STATUS,
    1,
    true,
    "offline"
  );

  if (connected) {
    Serial.println("connected.");
    mqttClient.publish(MQTT_TOPIC_STATUS, "online", true);
    mqttClient.subscribe(MQTT_TOPIC_CONTROL);
  } else {
    Serial.print("failed, rc=");
    Serial.println(mqttClient.state());
  }
}

void publishMQTTData() {
  if (!mqttClient.connected()) return;

  unsigned long now = millis();
  if (now - lastMqttPublish < MQTT_PUBLISH_INTERVAL_MS) return;
  lastMqttPublish = now;

  char payload[1024];

  snprintf(
    payload,
    sizeof(payload),
    "{"
      "\"timestamp_ms\":%lu,"
      "\"bms_timeout\":%s,"
      "\"voltage_valid\":%s,"
      "\"current_valid\":%s,"
      "\"temp_valid\":%s,"
      "\"pack_voltage\":%.3f,"
      "\"cell_1\":%.3f,"
      "\"cell_2\":%.3f,"
      "\"cell_3\":%.3f,"
      "\"cell_4\":%.3f,"
      "\"cell_5\":%.3f,"
      "\"cell_6\":%.3f,"
      "\"min_cell_voltage\":%.3f,"
      "\"max_cell_voltage\":%.3f,"
      "\"delta_cell_voltage\":%.3f,"
      "\"min_cell_index\":%d,"
      "\"max_cell_index\":%d,"
      "\"battery_current\":%.3f,"
      "\"balance_current\":%.3f,"
      "\"battery_t1\":%.1f,"
      "\"battery_t2\":%.1f,"
      "\"mos_temp\":%.1f,"
      "\"rpm_raw\":%.2f,"
      "\"rpm_filtered\":%.2f,"
      "\"position_deg\":%.2f,"
      "\"system_mode\":\"%s\","
      "\"relay_load\":%s,"
      "\"relay_charge\":%s,"
      "\"control_mode\":\"%s\","
      "\"manual_power\":%s,"
      "\"schedule_active\":%s,"
      "\"system_allowed\":%s,"
      "\"load_allowed\":%s,"
      "\"charge_allowed\":%s,"
      "\"night_charge_mode\":%s,"
      "\"battery_low_raw\":%s,"
      "\"battery_low_confirmed\":%s,"
      "\"battery_full_raw\":%s,"
      "\"battery_full_confirmed\":%s,"
      "\"charge_lock_active\":%s,"
      "\"charge_min_timer_active\":%s,"
      "\"charge_min_remaining_sec\":%lu"
    "}",
    now,
    isBMSTimeout() ? "true" : "false",
    voltageValid ? "true" : "false",
    currentValid ? "true" : "false",
    tempValid ? "true" : "false",
    packVoltage,
    cellVoltage[0],
    cellVoltage[1],
    cellVoltage[2],
    cellVoltage[3],
    cellVoltage[4],
    cellVoltage[5],
    minCellVoltage,
    maxCellVoltage,
    maxCellVoltage - minCellVoltage,
    minCellIndex + 1,
    maxCellIndex + 1,
    batteryCurrent,
    balanceCurrent,
    batteryT1,
    batteryT2,
    mosTemp,
    rpmRaw,
    rpm,
    positionDeg,
    systemModeToString(),
    loadRelayOn ? "true" : "false",
    chargeRelayOn ? "true" : "false",
    controlModeToString(),
    manualPower ? "true" : "false",
    scheduleActive ? "true" : "false",
    systemAllowed ? "true" : "false",
    loadAllowed ? "true" : "false",
    chargeAllowed ? "true" : "false",
    nightChargeMode ? "true" : "false",
    batteryLowRaw ? "true" : "false",
    batteryLowConfirmed ? "true" : "false",
    batteryFullRaw ? "true" : "false",
    batteryFullConfirmed ? "true" : "false",
    chargeLockActive ? "true" : "false",
    chargeMinTimerActive ? "true" : "false",
    getRemainingMinChargeSeconds()
  );

  bool ok = mqttClient.publish(MQTT_TOPIC_DATA, payload, false);

  Serial.print("MQTT publish: ");
  Serial.println(ok ? "OK" : "FAILED");
}

// ================= FAIL SAFE =================
void setRelay(int pin, bool on) {
  if (RELAY_ACTIVE_LOW) {
    digitalWrite(pin, on ? LOW : HIGH);
  } else {
    digitalWrite(pin, on ? HIGH : LOW);
  }
}

void updateRelayOutput() {
  setRelay(RELAY_LOAD_PIN, loadRelayOn);
  setRelay(RELAY_CHARGE_PIN, chargeRelayOn);
}

void allRelayOff() {
  loadRelayOn = false;
  chargeRelayOn = false;
  updateRelayOutput();
}

// ================= ENCODER FUNCTION =================
void updateEncoderData() {
  unsigned long now = millis();

  if (now - last_encoder_time >= 100) {
    long count = encoder.getCount();
    long delta = count - last_count;

    rpmRaw = (delta / (float)(PPR * QUAD_MULTIPLIER)) * 600.0;
    rpm = (RPM_FILTER_ALPHA * rpmRaw) + ((1.0 - RPM_FILTER_ALPHA) * rpm);

    long countPerRev = PPR * QUAD_MULTIPLIER;
    long normalizedCount = count % countPerRev;

    if (normalizedCount < 0) normalizedCount += countPerRev;

    positionDeg = normalizedCount * (360.0 / countPerRev);

    last_count = count;
    last_encoder_time = now;
  }
}

// ================= BMS PARSING =================
void updateVoltageData(uint8_t* data, size_t len) {
  if (len < 6 + CELL_COUNT * 2) return;

  float total = 0.0;
  bool valid = true;

  for (int i = 0; i < CELL_COUNT; i++) {
    uint16_t mv = readUInt16LE(data, 6 + i * 2);
    float v = mv / 1000.0;

    if (!isReasonableCellVoltage(v)) valid = false;

    cellVoltage[i] = v;
    total += v;
  }

  if (!valid || !isReasonablePackVoltage(total)) return;

  packVoltage = total;

  minCellVoltage = cellVoltage[0];
  maxCellVoltage = cellVoltage[0];
  minCellIndex = 0;
  maxCellIndex = 0;

  for (int i = 1; i < CELL_COUNT; i++) {
    if (cellVoltage[i] < minCellVoltage) {
      minCellVoltage = cellVoltage[i];
      minCellIndex = i;
    }

    if (cellVoltage[i] > maxCellVoltage) {
      maxCellVoltage = cellVoltage[i];
      maxCellIndex = i;
    }
  }

  voltageValid = true;
  lastBMSDataTime = millis();
}

void updateExtraData(uint8_t* data, size_t len) {
  if (len <= 120) return;

  float current = readInt16LE(data, 8) / 1000.0;
  float balCurrent = readInt16LE(data, 20) / 1000.0;

  float t1 = readInt16LE(data, 12) / 10.0;
  float t2 = readInt16LE(data, 14) / 10.0;
  float mos = readInt16LE(data, 104) / 10.0;

  currentValid = isReasonableCurrent(current);
  balanceCurrentValid = isReasonableCurrent(balCurrent);
  tempValid = isReasonableTemp(t1) && isReasonableTemp(t2) && isReasonableTemp(mos);

  if (currentValid) batteryCurrent = current;
  if (balanceCurrentValid) balanceCurrent = balCurrent;

  if (tempValid) {
    batteryT1 = t1;
    batteryT2 = t2;
    mosTemp = mos;
  }

  lastBMSDataTime = millis();
}

bool isOverTemp() {
  if (!tempValid) return false;

  return batteryT1 >= TEMP_OFF_C ||
         batteryT2 >= TEMP_OFF_C ||
         mosTemp >= TEMP_OFF_C;
}

// ================= RELAY CONTROL =================
void controlRelays() {
  updateSystemAllowed();

  if (!voltageValid || isBMSTimeout()) {
    systemMode = MODE_SAFE_OFF;
    chargeLockActive = false;
    chargeMinTimerActive = false;
    allRelayOff();
    return;
  }

  if (isOverTemp()) {
    systemMode = MODE_SAFE_OFF;
    chargeLockActive = false;
    chargeMinTimerActive = false;
    allRelayOff();
    return;
  }

  // Kalau manual OFF, atau semua izin mati, sistem benar-benar OFF.
  if (!systemAllowed) {
    systemMode = MODE_SAFE_OFF;
    allRelayOff();
    return;
  }

  updateBatteryThresholdState();

  unsigned long now = millis();

  bool chargeMinimumNotDone =
    chargeMinTimerActive &&
    (now - chargeModeStart < MIN_CHARGE_TIME_MS);

  // Inti perbaikan:
  // shouldCharge memakai chargeLockActive, bukan tegangan sesaat.
  // Jadi setelah baterai LOW, sistem tetap CHARGE sampai FULL stabil.
  bool shouldCharge =
    chargeAllowed &&
    !batteryFullConfirmed &&
    (chargeLockActive || chargeMinimumNotDone);

  // Beban hanya boleh ON kalau tidak ada charge lock dan tidak dalam timer minimum charge.
  // Ini mencegah beban hidup lagi akibat voltage rebound.
  bool shouldLoad =
    loadAllowed &&
    !chargeLockActive &&
    !chargeMinimumNotDone &&
    !batteryLowConfirmed;

  switch (systemMode) {
    case MODE_SAFE_OFF:
      allRelayOff();

      if (shouldCharge) {
        transitionStart = now;
        systemMode = MODE_TRANSITION_TO_CHARGE;
      } else if (shouldLoad) {
        transitionStart = now;
        systemMode = MODE_TRANSITION_TO_LOAD;
      } else {
        // AUTO malam dan baterai belum low:
        // relay beban OFF, relay charge OFF, ESP32 tetap monitoring + publish data.
        systemMode = MODE_SAFE_OFF;
      }
      break;

    case MODE_LOAD:
      loadRelayOn = true;
      chargeRelayOn = false;

      // Jika jadwal AUTO sudah lewat jam 19:00, atau manual dimatikan,
      // beban/motor wajib OFF.
      if (!loadAllowed) {
        allRelayOff();

        if (shouldCharge) {
          transitionStart = now;
          systemMode = MODE_TRANSITION_TO_CHARGE;
        } else {
          systemMode = MODE_SAFE_OFF;
        }
        break;
      }

      // Saat LOW sudah confirmed, langsung matikan beban dan masuk transisi charge.
      // Karena chargeLockActive sudah ON, beban tidak akan nyala lagi walaupun tegangan rebound.
      if (shouldCharge) {
        allRelayOff();
        transitionStart = now;
        systemMode = MODE_TRANSITION_TO_CHARGE;
      }
      break;

    case MODE_TRANSITION_TO_CHARGE:
      allRelayOff();

      // Jangan batalkan transisi hanya karena batteryLowRaw hilang akibat rebound.
      // Batalkan hanya kalau charge memang tidak diizinkan lagi atau baterai sudah full confirmed.
      if (!shouldCharge) {
        if (shouldLoad) {
          transitionStart = now;
          systemMode = MODE_TRANSITION_TO_LOAD;
        } else {
          systemMode = MODE_SAFE_OFF;
        }
        break;
      }

      if (now - transitionStart >= RELAY_SWITCH_DELAY_MS) {
        chargeRelayOn = true;
        loadRelayOn = false;

        // Timer dimulai saat benar-benar masuk mode charge.
        // Kalau timer sebelumnya masih aktif, jangan reset.
        if (!chargeMinTimerActive || now - chargeModeStart >= MIN_CHARGE_TIME_MS) {
          chargeModeStart = now;
          chargeMinTimerActive = true;
        }

        systemMode = MODE_CHARGE;
        updateRelayOutput();
      }
      break;

    case MODE_CHARGE:
      chargeRelayOn = true;
      loadRelayOn = false;

      // Kalau izin charge hilang, misalnya manual OFF, charge wajib mati.
      if (!chargeAllowed) {
        allRelayOff();
        systemMode = MODE_SAFE_OFF;
        break;
      }

      // Safety tetap prioritas: charge baru berhenti kalau FULL sudah stabil.
      if (batteryFullConfirmed) {
        chargeLockActive = false;
        chargeMinTimerActive = false;
        allRelayOff();

        if (loadAllowed) {
          transitionStart = now;
          systemMode = MODE_TRANSITION_TO_LOAD;
        } else {
          // AUTO malam: setelah penuh, jangan pindah ke beban.
          systemMode = MODE_SAFE_OFF;
        }
      }

      // Kalau belum full confirmed, sistem tetap CHARGE.
      // Ini sengaja supaya tidak pindah ke LOAD hanya karena voltage rebound.
      break;

    case MODE_TRANSITION_TO_LOAD:
      allRelayOff();

      // Kalau jadwal sudah lewat atau manual OFF, jangan lanjut ke beban.
      if (!loadAllowed) {
        if (shouldCharge) {
          transitionStart = now;
          systemMode = MODE_TRANSITION_TO_CHARGE;
        } else {
          systemMode = MODE_SAFE_OFF;
        }
        break;
      }

      // Kalau charge lock aktif lagi, batalkan transisi ke LOAD.
      if (shouldCharge) {
        transitionStart = now;
        systemMode = MODE_TRANSITION_TO_CHARGE;
        break;
      }

      if (now - transitionStart >= RELAY_SWITCH_DELAY_MS) {
        loadRelayOn = true;
        chargeRelayOn = false;
        systemMode = MODE_LOAD;
        updateRelayOutput();
      }
      break;
  }

  updateRelayOutput();
}

// ================= PRINT =================
void printDataForControl() {
  Serial.println();
  Serial.println("===== DATA BMS + RELAY + ENCODER + MQTT =====");

  Serial.print("WiFi                   : ");
  Serial.println(WiFi.status() == WL_CONNECTED ? "CONNECTED" : "DISCONNECTED");

  Serial.print("MQTT                   : ");
  Serial.println(mqttClient.connected() ? "CONNECTED" : "DISCONNECTED");

  Serial.print("BMS Timeout            : ");
  Serial.println(isBMSTimeout() ? "YES" : "NO");

  if (voltageValid) {
    Serial.print("Tegangan Total Baterai : ");
    Serial.print(packVoltage, 3);
    Serial.println(" V");

    for (int i = 0; i < CELL_COUNT; i++) {
      Serial.print("Tegangan Cell ");
      Serial.print(i + 1);
      Serial.print("        : ");
      Serial.print(cellVoltage[i], 3);
      Serial.println(" V");
    }

    Serial.print("Tegangan Minimum       : Cell ");
    Serial.print(minCellIndex + 1);
    Serial.print(" = ");
    Serial.print(minCellVoltage, 3);
    Serial.println(" V");

    Serial.print("Tegangan Maksimum      : Cell ");
    Serial.print(maxCellIndex + 1);
    Serial.print(" = ");
    Serial.print(maxCellVoltage, 3);
    Serial.println(" V");
  } else {
    Serial.println("Data tegangan          : belum valid");
  }

  if (currentValid) {
    Serial.print("Arus Baterai           : ");
    Serial.print(batteryCurrent, 3);
    Serial.println(" A");
  } else {
    Serial.println("Arus Baterai           : belum valid");
  }

  if (tempValid) {
    Serial.print("Battery T1             : ");
    Serial.print(batteryT1, 1);
    Serial.println(" C");

    Serial.print("Battery T2             : ");
    Serial.print(batteryT2, 1);
    Serial.println(" C");

    Serial.print("MOS Temp               : ");
    Serial.print(mosTemp, 1);
    Serial.println(" C");
  } else {
    Serial.println("Temperatur             : belum valid");
  }

  Serial.print("Mode Kontrol           : ");
  Serial.println(controlModeToString());

  Serial.print("Manual Power           : ");
  Serial.println(manualPower ? "ON" : "OFF");

  Serial.print("Jadwal Aktif           : ");
  Serial.println(scheduleActive ? "YES" : "NO");

  Serial.print("System Allowed         : ");
  Serial.println(systemAllowed ? "YES" : "NO");

  Serial.print("Load Allowed           : ");
  Serial.println(loadAllowed ? "YES" : "NO");

  Serial.print("Charge Allowed         : ");
  Serial.println(chargeAllowed ? "YES" : "NO");

  Serial.print("Night Charge Mode      : ");
  Serial.println(nightChargeMode ? "YES" : "NO");

  Serial.print("Battery Low Raw        : ");
  Serial.println(batteryLowRaw ? "YES" : "NO");

  Serial.print("Battery Low Confirmed  : ");
  Serial.println(batteryLowConfirmed ? "YES" : "NO");

  Serial.print("Battery Full Raw       : ");
  Serial.println(batteryFullRaw ? "YES" : "NO");

  Serial.print("Battery Full Confirmed : ");
  Serial.println(batteryFullConfirmed ? "YES" : "NO");

  Serial.print("Charge Lock Active     : ");
  Serial.println(chargeLockActive ? "YES" : "NO");

  Serial.print("Mode Sistem            : ");
  Serial.println(systemModeToString());

  Serial.print("Relay Beban            : ");
  Serial.println(loadRelayOn ? "ON" : "OFF");

  Serial.print("Relay Charge           : ");
  Serial.println(chargeRelayOn ? "ON" : "OFF");

  Serial.print("Timer Minimum Charge   : ");
  Serial.print(chargeMinTimerActive ? "ACTIVE" : "INACTIVE");
  Serial.print(" | Sisa ");
  Serial.print(getRemainingMinChargeSeconds());
  Serial.println(" detik");

  Serial.print("RPM Filtered           : ");
  Serial.println(rpm, 2);

  Serial.print("Posisi Encoder         : ");
  Serial.print(positionDeg, 2);
  Serial.println(" derajat");

  Serial.println("==============================================");
}

// ================= BLE CALLBACK =================
void parseNotify(uint8_t* data, size_t len) {
  if (len < 20) return;

  bool isVoltageFrame =
    data[0] == 0x55 &&
    data[1] == 0xAA &&
    data[2] == 0xEB &&
    data[3] == 0x90 &&
    data[4] == 0x02;

  if (isVoltageFrame) {
    updateVoltageData(data, len);
    return;
  }

  bool isExtraFrame =
    !(data[0] == 0x55 && data[1] == 0xAA) &&
    len > 120;

  if (isExtraFrame) {
    updateExtraData(data, len);
    return;
  }
}

void notifyCallback(
  NimBLERemoteCharacteristic* ch,
  uint8_t* data,
  size_t len,
  bool isNotify
) {
  parseNotify(data, len);
}

// ================= BLE CONNECT =================
bool connectToBMS() {
  Serial.println("Scanning...");

  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->setActiveScan(true);
  scan->setInterval(100);
  scan->setWindow(99);

  NimBLEScanResults results = scan->getResults(15000);

  const NimBLEAdvertisedDevice* target = nullptr;

  for (int i = 0; i < results.getCount(); i++) {
    const NimBLEAdvertisedDevice* dev = results.getDevice(i);
    String addr = dev->getAddress().toString().c_str();

    if (addr.equalsIgnoreCase(TARGET_ADDR)) {
      target = dev;
      Serial.println("BMS found.");
      break;
    }
  }

  if (target == nullptr) {
    Serial.println("BMS not found.");
    scan->clearResults();
    return false;
  }

  client = NimBLEDevice::createClient();

  Serial.println("Connecting...");
  if (!client->connect(target)) {
    Serial.println("Connect failed.");
    scan->clearResults();
    return false;
  }

  Serial.println("Connected.");

  NimBLERemoteService* service = client->getService("ffe0");
  if (!service) {
    Serial.println("Service FFE0 not found.");
    client->disconnect();
    scan->clearResults();
    return false;
  }

  dataChar = service->getCharacteristic("ffe1");
  if (!dataChar) {
    Serial.println("FFE1 not found.");
    client->disconnect();
    scan->clearResults();
    return false;
  }

  if (!dataChar->subscribe(true, notifyCallback)) {
    Serial.println("Subscribe failed.");
    client->disconnect();
    scan->clearResults();
    return false;
  }

  Serial.println("Subscribed to notify.");
  scan->clearResults();
  return true;
}

void sendFrame(uint8_t* frame, size_t len, const char* label) {
  if (!client || !client->isConnected() || !dataChar) return;

  bool ok = dataChar->writeValue(frame, len, true);

  Serial.print(label);
  Serial.print(" sent: ");
  Serial.println(ok ? "OK" : "FAILED");
}

void sendInitialSequence() {
  sendFrame(requestSettings, sizeof(requestSettings), "Request 0x96 Settings");
  delay(1000);
  sendFrame(requestDeviceInfo, sizeof(requestDeviceInfo), "Request 0x97 DeviceInfo");
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(RELAY_LOAD_PIN, OUTPUT);
  pinMode(RELAY_CHARGE_PIN, OUTPUT);

  allRelayOff();

  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encoder.attachFullQuad(ENCODER_A, ENCODER_B);
  encoder.clearCount();

  last_count = 0;
  last_encoder_time = millis();

  Serial.println("Encoder PCNT ready");
  Serial.println("JK BMS BLE + SAFETY RELAY CONTROL + ENCODER + MQTT");

  wifiClient.setInsecure();

  setupWiFi();
  configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, "pool.ntp.org", "time.google.com");
  updateSystemAllowed();
  setupMQTT();

  NimBLEDevice::init("");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  if (connectToBMS()) {
    delay(1000);
    sendInitialSequence();
  }
}

// ================= LOOP =================
void loop() {
  static unsigned long lastRequest = 0;
  static unsigned long lastPrint = 0;
  static unsigned long lastControl = 0;

  maintainWiFi();
  maintainMQTT();

  updateEncoderData();

  if (client && client->isConnected()) {
    if (millis() - lastRequest > 30000) {
      lastRequest = millis();
      sendInitialSequence();
    }

    if (millis() - lastControl > 500) {
      lastControl = millis();
      controlRelays();
    }

    publishMQTTData();

    if (millis() - lastPrint > 5000) {
      lastPrint = millis();
      printDataForControl();
    }
  } else {
    systemMode = MODE_SAFE_OFF;
    allRelayOff();

    Serial.println("BMS disconnected. Relay OFF. Restart ESP32 or improve BLE connection.");
    delay(5000);
  }
}
