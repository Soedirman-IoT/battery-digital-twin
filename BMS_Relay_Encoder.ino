#include <NimBLEDevice.h>
#include <ESP32Encoder.h>

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

// Filter RPM
const float RPM_FILTER_ALPHA = 0.25;

// ================= PIN RELAY =================
const int RELAY_LOAD_PIN   = 26;
const int RELAY_CHARGE_PIN = 27;

const bool RELAY_ACTIVE_LOW = false;

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
const float CHARGE_STOP_CELL_V  = 4.10;

const float PACK_CHARGE_START_V = 18.6;
const float PACK_CHARGE_STOP_V  = 24.6;

const float TEMP_OFF_C = 55.0;

const unsigned long RELAY_SWITCH_DELAY_MS = 3000;
unsigned long transitionStart = 0;

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

bool isBMSTimeout() {
  if (lastBMSDataTime == 0) return true;
  return millis() - lastBMSDataTime > BMS_TIMEOUT_MS;
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

    if (normalizedCount < 0) {
      normalizedCount += countPerRev;
    }

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
  if (!voltageValid || isBMSTimeout()) {
    systemMode = MODE_SAFE_OFF;
    allRelayOff();
    return;
  }

  if (isOverTemp()) {
    systemMode = MODE_SAFE_OFF;
    allRelayOff();
    return;
  }

  bool batteryLow =
    minCellVoltage <= CHARGE_START_CELL_V ||
    packVoltage <= PACK_CHARGE_START_V;

  bool batteryFull =
    maxCellVoltage >= CHARGE_STOP_CELL_V ||
    packVoltage >= PACK_CHARGE_STOP_V;

  unsigned long now = millis();

  switch (systemMode) {
    case MODE_SAFE_OFF:
      allRelayOff();

      if (voltageValid && !isBMSTimeout() && !isOverTemp()) {
        if (batteryLow) {
          transitionStart = now;
          systemMode = MODE_TRANSITION_TO_CHARGE;
        } else {
          transitionStart = now;
          systemMode = MODE_TRANSITION_TO_LOAD;
        }
      }
      break;

    case MODE_LOAD:
      loadRelayOn = true;
      chargeRelayOn = false;

      if (batteryLow) {
        allRelayOff();
        transitionStart = now;
        systemMode = MODE_TRANSITION_TO_CHARGE;
      }
      break;

    case MODE_TRANSITION_TO_CHARGE:
      allRelayOff();

      if (now - transitionStart >= RELAY_SWITCH_DELAY_MS) {
        chargeRelayOn = true;
        loadRelayOn = false;
        systemMode = MODE_CHARGE;
        updateRelayOutput();
      }
      break;

    case MODE_CHARGE:
      chargeRelayOn = true;
      loadRelayOn = false;

      if (batteryFull) {
        allRelayOff();
        transitionStart = now;
        systemMode = MODE_TRANSITION_TO_LOAD;
      }
      break;

    case MODE_TRANSITION_TO_LOAD:
      allRelayOff();

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
  Serial.println("===== DATA BMS + RELAY + ENCODER =====");

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

  Serial.print("Mode Sistem            : ");
  switch (systemMode) {
    case MODE_LOAD:
      Serial.println("LOAD");
      break;
    case MODE_CHARGE:
      Serial.println("CHARGE");
      break;
    case MODE_TRANSITION_TO_LOAD:
      Serial.println("TRANSITION TO LOAD");
      break;
    case MODE_TRANSITION_TO_CHARGE:
      Serial.println("TRANSITION TO CHARGE");
      break;
    case MODE_SAFE_OFF:
      Serial.println("SAFE OFF");
      break;
  }

  Serial.print("Relay Beban            : ");
  Serial.println(loadRelayOn ? "ON" : "OFF");

  Serial.print("Relay Charge           : ");
  Serial.println(chargeRelayOn ? "ON" : "OFF");

  Serial.print("RPM Raw                : ");
  Serial.println(rpmRaw, 2);

  Serial.print("RPM Filtered           : ");
  Serial.println(rpm, 2);

  Serial.print("Posisi Encoder         : ");
  Serial.print(positionDeg, 2);
  Serial.println(" derajat");

  Serial.println("=======================================");
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
  Serial.println("JK BMS BLE + SAFETY RELAY CONTROL + ENCODER");

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

    if (millis() - lastPrint > 5000) {
      lastPrint = millis();
      printDataForControl();
    }
  } else {
    systemMode = MODE_SAFE_OFF;
    allRelayOff();

    Serial.println("Disconnected. Relay OFF. Restart ESP32.");
    delay(5000);
  }
}