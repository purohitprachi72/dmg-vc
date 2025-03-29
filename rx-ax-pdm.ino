#include "BLEDevice.h"

// BLE service and characteristic UUIDs (16-bit UUIDs for custom service/char)
static BLEUUID serviceUUID((uint16_t)0xFF00);
static BLEUUID charUUID((uint16_t)0xFF01);

// Target device name to search for
const char* targetDeviceName = "XiaoBLE-AccelStream";

// Global objects and flags for connection logic
static BLEAdvertisedDevice* advDevice = nullptr;
static BLERemoteCharacteristic* remoteChar = nullptr;
static bool doConnect = false;
static bool connected = false;
static bool doScan = false;

// For monitoring notification rate
unsigned long lastPrintTime = 0;
unsigned long notifyCount = 0;

// Notify callback: Processes aggregated packets of 240 bytes (or multiples of 8)
static void notifyCallback(BLERemoteCharacteristic* pRC, uint8_t* pData, size_t length, bool isNotify) {
  notifyCount++;
  
  // Check that the received length is a multiple of 8
  if (length % 8 != 0) {
    Serial.print("Unexpected notify length: ");
    Serial.println(length);
    return;
  }
  
  // Number of sample pairs in this packet
  uint16_t numPairs = length / 8;
  Serial.print("Received packet with ");
  Serial.print(numPairs);
  Serial.println(" sample pairs:");
  
  // For each sample pair, extract and print values
  for (uint16_t i = 0; i < numPairs; i++) {
    uint16_t base = i * 8;
    // Accelerometer values (16-bit little-endian)
    int16_t ax = (int16_t)((pData[base+1] << 8) | pData[base]);
    int16_t ay = (int16_t)((pData[base+3] << 8) | pData[base+2]);
    int16_t az = (int16_t)((pData[base+5] << 8) | pData[base+4]);
    // PDM sample (16-bit PCM)
    int16_t pdm = (int16_t)((pData[base+7] << 8) | pData[base+6]);
    
    // Print the values in a comma-separated format
    Serial.print("Pair ");
    Serial.print(i);
    Serial.print(" - Accel: ");
    Serial.print(ax);
    Serial.print(", ");
    Serial.print(ay);
    Serial.print(", ");
    Serial.print(az);
    Serial.print(" | PDM: ");
    Serial.println(pdm);
  }
  
  // Optionally print notifications per second
  if (millis() - lastPrintTime > 1000) {
    Serial.print("Notifications per second: ");
    Serial.println(notifyCount);
    notifyCount = 0;
    lastPrintTime = millis();
  }
}

// Client callbacks to handle connection events
class MyClientCallbacks : public BLEClientCallbacks {
  void onConnect(BLEClient* pClient) override {
    Serial.println("** Connected to peripheral **");
  }

  void onDisconnect(BLEClient* pClient) override {
    Serial.println("** Disconnected from peripheral **");
    connected = false;
    doScan = true;  // restart scanning
  }
};

// Advertised device callback: Called when devices are discovered
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) override {
    if (advertisedDevice.haveName() && advertisedDevice.getName() == targetDeviceName) {
      Serial.print("Found target device: ");
      Serial.println(advertisedDevice.getName().c_str());
      BLEDevice::getScan()->stop();  // Stop scanning when found
      advDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      doScan = false;
      Serial.println("Scanning stopped. Connecting to the device...");
    }
  }
};

// Connect to the discovered BLE peripheral and subscribe to notifications
bool connectToServer() {
  if (advDevice == nullptr) return false;
  Serial.print("Connecting to ");
  Serial.println(advDevice->getAddress().toString().c_str());
  
  BLEClient* pClient = BLEDevice::createClient();
  pClient->setClientCallbacks(new MyClientCallbacks());
  
  if (!pClient->connect(advDevice)) {
    Serial.println("Failed to connect. Retrying...");
    return false;
  }
  
  Serial.println("Connected to server, discovering services...");
  BLERemoteService* remoteService = pClient->getService(serviceUUID);
  if (remoteService == nullptr) {
    Serial.println("Failed to find service UUID 0xFF00 on peripheral.");
    pClient->disconnect();
    return false;
  }
  
  Serial.println("Found service 0xFF00");
  remoteChar = remoteService->getCharacteristic(charUUID);
  if (remoteChar == nullptr) {
    Serial.println("Failed to find characteristic UUID 0xFF01.");
    pClient->disconnect();
    return false;
  }
  
  Serial.println("Found characteristic 0xFF01");
  if (remoteChar->canNotify()) {
    remoteChar->registerForNotify(notifyCallback);
    Serial.println("Subscribed to notifications.");
  }
  
  connected = true;
  return true;
}

void setup() {
  Serial.begin(115200);
  BLEDevice::init("ESP32AccelClient");
  BLEDevice::setPower(ESP_PWR_LVL_P9);  // Maximum power for reliable connection
  
  // Set up scanning parameters
  BLEScan* pScan = BLEDevice::getScan();
  pScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pScan->setInterval(1349);  // in 0.625ms units
  pScan->setWindow(449);     // in 0.625ms units
  pScan->setActiveScan(true);
  
  Serial.println("Starting scan for device...");
  pScan->start(0, false);  // Scan indefinitely until stopped
}

void loop() {
  if (doConnect) {
    doConnect = false;
    if (connectToServer()) {
      Serial.println(">> Successfully connected and subscribed!");
    } else {
      Serial.println("Connection failed, restarting scan...");
      BLEDevice::getScan()->start(0, false);
    }
  }
  
  if (!connected && doScan) {
    doScan = false;
    Serial.println("Re-starting scan for device...");
    BLEDevice::getScan()->start(0, false);
  }
  
  delay(10);
}
