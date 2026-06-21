/*
  jk_bms_ble_modified.ino

  Modifikasi dari konsep jk_bms_ble agar bisa konek ke JK-BMS milik Dimas
  berdasarkan koneksi yang sudah berhasil di BMS_Relay_Encoder(1).ino.

  Koneksi BLE:
  - Target MAC      : 20:25:03:32:08:0a
  - Service UUID    : FFE0
  - Characteristic  : FFE1
  - Request realtime: AA 55 90 EB 96 ...
  - Request info    : AA 55 90 EB 97 ...

  Library:
  - NimBLE-Arduino
*/

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <vector>

// ================= BLE CONFIG DARI BMS_RELAY_ENCODER =================
static const char* TARGET_ADDR = "20:25:03:32:08:0a";
static const char* JK_SERVICE_UUID = "ffe0";
static const char* JK_CHAR_UUID    = "ffe1";

NimBLEClient* client = nullptr;
NimBLERemoteCharacteristic* dataChar = nullptr;

// ================= JK-BMS REQUEST FRAME YANG SUDAH BERHASIL =================
uint8_t requestCellInfo[] = {
  0xAA, 0x55, 0x90, 0xEB, 0x96, 0x00, 0x79, 0x62, 0x96, 0xED,
  0xE3, 0xD0, 0x82, 0xA1, 0x9B, 0x5B, 0x3C, 0x9C, 0x4B, 0x5D
};

uint8_t requestDeviceInfo[] = {
  0xAA, 0x55, 0x90, 0xEB, 0x97, 0x00, 0xDF, 0x52, 0x88, 0x67,
  0x9D, 0x0A, 0x09, 0x6B, 0x9A, 0xF6, 0x70, 0x9A, 0x17, 0xFD
};

// ================= FRAME BUFFER =================
static const uint16_t JK_FRAME_SIZE = 300;
static const uint16_t MAX_FRAME_SIZE = 400;

std::vector<uint8_t> frameBuffer;

unsigned long lastRequestMs = 0;
unsigned long lastDataMs = 0;
const unsigned long REQUEST_INTERVAL_MS = 2000;
const unsigned long BMS_TIMEOUT_MS = 10000;

// ================= DATA BMS =================
const uint8_t CELL_COUNT = 6;

float cellVoltage[CELL_COUNT] = {0};
float packVoltage = 0.0;
float currentA = 0.0;
float temp1C = 0.0;
float temp2C = 0.0;
float mosTempC = 0.0;
float balanceCurrentA = 0.0;

uint8_t socRaw141 = 0;
uint8_t socRaw173 = 0;
float socFromCapacity = 0.0;
float capacityRemainAh = 0.0;
float fullCapacityAh = 0.0;

bool validFrameReceived = false;

// ================= HELPER =================
uint8_t crc8_sum(const uint8_t* data, uint16_t len) {
  uint8_t crc = 0;
  for (uint16_t i = 0; i < len; i++) crc += data[i];
  return crc;
}

uint16_t u16le(const std::vector<uint8_t>& data, size_t i) {
  return (uint16_t)data[i] | ((uint16_t)data[i + 1] << 8);
}

int16_t s16le(const std::vector<uint8_t>& data, size_t i) {
  return (int16_t)u16le(data, i);
}

uint32_t u32le(const std::vector<uint8_t>& data, size_t i) {
  return (uint32_t)u16le(data, i) | ((uint32_t)u16le(data, i + 2) << 16);
}

int32_t s32le(const std::vector<uint8_t>& data, size_t i) {
  return (int32_t)u32le(data, i);
}

bool isReasonableSoc(uint8_t soc) {
  return soc <= 100;
}

bool isReasonablePackVoltage(float v) {
  return v >= 10.0 && v <= 30.0;
}

bool isReasonableCurrent(float a) {
  return a >= -100.0 && a <= 100.0;
}

void printHex(const uint8_t* data, size_t len) {
  for (size_t i = 0; i < len; i++) {
    if (data[i] < 0x10) Serial.print('0');
    Serial.print(data[i], HEX);
    Serial.print(' ');
  }
  Serial.println();
}

// ================= PARSER JK02 CELL INFO =================
// Parser ini memakai acuan jk_bms_ble.cpp:
// - frame 0x02
// - SoC 24S: byte 141
// - SoC 32S: byte 173
// Untuk BMS 6S milik lu, tegangan sel tetap di byte 6, 8, 10, dst.
void parseJk02CellInfo(const std::vector<uint8_t>& data) {
  if (data.size() < JK_FRAME_SIZE) return;

  if (!(data[0] == 0x55 && data[1] == 0xAA && data[2] == 0xEB && data[3] == 0x90)) {
    Serial.println("Header salah, frame diabaikan.");
    return;
  }

  if (data[4] != 0x02) {
    Serial.print("Bukan frame realtime/cell info. Type: 0x");
    Serial.println(data[4], HEX);
    return;
  }

  uint8_t computed = crc8_sum(data.data(), JK_FRAME_SIZE - 1);
  uint8_t remote = data[JK_FRAME_SIZE - 1];

  if (computed != remote) {
    Serial.print("CRC gagal. computed=0x");
    Serial.print(computed, HEX);
    Serial.print(" remote=0x");
    Serial.println(remote, HEX);
    return;
  }

  // Cell voltage: byte 6 + i*2, satuan 0.001 V
  packVoltage = 0.0;
  for (uint8_t i = 0; i < CELL_COUNT; i++) {
    cellVoltage[i] = u16le(data, 6 + i * 2) * 0.001f;
    packVoltage += cellVoltage[i];
  }

  // Dua kandidat layout:
  // JK02 24S: offset akhir = 0
  // JK02 32S: offset akhir = 32
  uint8_t offset24 = 0;
  uint8_t offset32 = 32;

  socRaw141 = data[141 + offset24];
  socRaw173 = data[141 + offset32];

  // Pakai layout yang paling masuk akal.
  // Untuk BMS 6S lu, biasanya byte 141 lebih mungkin benar.
  uint8_t chosenOffset = 0;
  uint8_t chosenSoc = socRaw141;

  if (!isReasonableSoc(chosenSoc) && isReasonableSoc(socRaw173)) {
    chosenOffset = 32;
    chosenSoc = socRaw173;
  }

  // Field utama dengan offset terpilih.
  // Kalau offset salah, nilai voltage/current dari field ini biasanya ikut ngawur.
  float totalVoltageFromFrame = u32le(data, 118 + chosenOffset) * 0.001f;
  float currentFromFrame = s32le(data, 126 + chosenOffset) * 0.001f;

  temp1C = s16le(data, 130 + chosenOffset) * 0.1f;
  temp2C = s16le(data, 132 + chosenOffset) * 0.1f;
  mosTempC = s16le(data, 134 + chosenOffset) * 0.1f;
  balanceCurrentA = s16le(data, 138 + chosenOffset) * 0.001f;

  capacityRemainAh = u32le(data, 142 + chosenOffset) * 0.001f;
  fullCapacityAh = u32le(data, 146 + chosenOffset) * 0.001f;

  if (fullCapacityAh > 0.1f) {
    socFromCapacity = (capacityRemainAh / fullCapacityAh) * 100.0f;
    if (socFromCapacity > 100.0f) socFromCapacity = 100.0f;
  }

  // Validasi voltage/current frame. Kalau field pack voltage ngawur,
  // tetap pakai penjumlahan cell sebagai packVoltage.
  if (isReasonablePackVoltage(totalVoltageFromFrame)) {
    packVoltage = totalVoltageFromFrame;
  }

  if (isReasonableCurrent(currentFromFrame)) {
    currentA = currentFromFrame;
  }

  validFrameReceived = true;
  lastDataMs = millis();

  Serial.println();
  Serial.println("===== JK-BMS DATA VALID =====");
  Serial.print("Offset terpilih      : ");
  Serial.println(chosenOffset == 0 ? "24S / byte SoC 141" : "32S / byte SoC 173");

  Serial.print("SoC byte 141         : ");
  Serial.print(socRaw141);
  Serial.println(" %");

  Serial.print("SoC byte 173         : ");
  Serial.print(socRaw173);
  Serial.println(" %");

  Serial.print("SoC dari kapasitas   : ");
  Serial.print(socFromCapacity, 1);
  Serial.println(" %");

  Serial.print("Pack voltage         : ");
  Serial.print(packVoltage, 3);
  Serial.println(" V");

  Serial.print("Current              : ");
  Serial.print(currentA, 3);
  Serial.println(" A");

  Serial.print("Capacity remain      : ");
  Serial.print(capacityRemainAh, 3);
  Serial.println(" Ah");

  Serial.print("Full capacity        : ");
  Serial.print(fullCapacityAh, 3);
  Serial.println(" Ah");

  Serial.print("Temperature T1/T2/MOS: ");
  Serial.print(temp1C, 1);
  Serial.print(" / ");
  Serial.print(temp2C, 1);
  Serial.print(" / ");
  Serial.print(mosTempC, 1);
  Serial.println(" C");

  Serial.print("Cell voltage         : ");
  for (uint8_t i = 0; i < CELL_COUNT; i++) {
    Serial.print(cellVoltage[i], 3);
    Serial.print(i == CELL_COUNT - 1 ? "\n" : " | ");
  }
}

// ================= BLE ASSEMBLY =================
void assembleFrame(uint8_t* data, size_t len) {
  if (len == 0) return;

  // Jika ketemu header frame baru, reset buffer.
  if (len >= 4 && data[0] == 0x55 && data[1] == 0xAA && data[2] == 0xEB && data[3] == 0x90) {
    frameBuffer.clear();
  }

  frameBuffer.insert(frameBuffer.end(), data, data + len);

  if (frameBuffer.size() > MAX_FRAME_SIZE) {
    Serial.println("Buffer terlalu panjang, reset.");
    frameBuffer.clear();
    return;
  }

  if (frameBuffer.size() >= JK_FRAME_SIZE) {
    parseJk02CellInfo(frameBuffer);
    frameBuffer.clear();
  }
}

void notifyCallback(
  NimBLERemoteCharacteristic* ch,
  uint8_t* data,
  size_t len,
  bool isNotify
) {
  Serial.print("Notify len=");
  Serial.print(len);
  Serial.print(" data=");
  printHex(data, min((size_t)16, len));

  assembleFrame(data, len);
}

// ================= BLE CONNECT =================
bool connectToBMS() {
  Serial.println("Scanning JK-BMS...");

  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->setActiveScan(true);
  scan->setInterval(100);
  scan->setWindow(99);

  NimBLEScanResults results = scan->getResults(15000);

  const NimBLEAdvertisedDevice* target = nullptr;

  for (int i = 0; i < results.getCount(); i++) {
    const NimBLEAdvertisedDevice* dev = results.getDevice(i);
    String addr = dev->getAddress().toString().c_str();

    Serial.print("Found BLE: ");
    Serial.println(addr);

    if (addr.equalsIgnoreCase(TARGET_ADDR)) {
      target = dev;
      Serial.println("Target JK-BMS ditemukan.");
      break;
    }
  }

  if (target == nullptr) {
    Serial.println("Target JK-BMS tidak ditemukan.");
    scan->clearResults();
    return false;
  }

  if (client != nullptr) {
    NimBLEDevice::deleteClient(client);
    client = nullptr;
  }

  client = NimBLEDevice::createClient();

  Serial.println("Connecting to JK-BMS...");
  if (!client->connect(target)) {
    Serial.println("Connect gagal.");
    scan->clearResults();
    return false;
  }

  Serial.println("Connected.");

  NimBLERemoteService* service = client->getService(JK_SERVICE_UUID);
  if (!service) {
    Serial.println("Service FFE0 tidak ditemukan.");
    client->disconnect();
    scan->clearResults();
    return false;
  }

  dataChar = service->getCharacteristic(JK_CHAR_UUID);
  if (!dataChar) {
    Serial.println("Characteristic FFE1 tidak ditemukan.");
    client->disconnect();
    scan->clearResults();
    return false;
  }

  if (!dataChar->canNotify()) {
    Serial.println("Warning: FFE1 tidak advertise notify, tapi tetap coba subscribe.");
  }

  if (!dataChar->subscribe(true, notifyCallback)) {
    Serial.println("Subscribe notify gagal.");
    client->disconnect();
    scan->clearResults();
    return false;
  }

  Serial.println("Subscribe notify berhasil.");

  scan->clearResults();
  return true;
}

// ================= REQUEST =================
void sendFrame(uint8_t* frame, size_t len, const char* label) {
  if (!client || !client->isConnected() || !dataChar) {
    Serial.print(label);
    Serial.println(" tidak dikirim: BLE belum connect.");
    return;
  }

  bool ok = dataChar->writeValue(frame, len, true);  // true = response
  Serial.print(label);
  Serial.print(" sent: ");
  Serial.println(ok ? "OK" : "FAILED");
}

void sendInitialSequence() {
  // Urutan mengikuti kode BMS_Relay_Encoder yang sudah berhasil.
  sendFrame(requestCellInfo, sizeof(requestCellInfo), "Request 0x96 CellInfo");
  delay(700);
  sendFrame(requestDeviceInfo, sizeof(requestDeviceInfo), "Request 0x97 DeviceInfo");
}

void requestBmsPeriodically() {
  if (!client || !client->isConnected()) return;

  unsigned long now = millis();
  if (now - lastRequestMs >= REQUEST_INTERVAL_MS) {
    lastRequestMs = now;
    sendFrame(requestCellInfo, sizeof(requestCellInfo), "Request 0x96 CellInfo");
  }
}

// ================= SETUP LOOP =================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("JK-BMS BLE Modified Start");

  NimBLEDevice::init("ESP32_JK_BMS_Dimas");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  if (connectToBMS()) {
    delay(500);
    sendInitialSequence();
  }
}

void loop() {
  if (!client || !client->isConnected()) {
    Serial.println("BLE disconnected. Reconnect...");
    validFrameReceived = false;
    connectToBMS();
    delay(1000);
    if (client && client->isConnected()) {
      sendInitialSequence();
    }
  }

  requestBmsPeriodically();

  if (lastDataMs > 0 && millis() - lastDataMs > BMS_TIMEOUT_MS) {
    Serial.println("Warning: tidak ada data valid dari BMS > 10 detik.");
    lastDataMs = millis(); // biar tidak spam serial terus
  }

  delay(100);
}
