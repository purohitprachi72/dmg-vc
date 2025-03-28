#include "BLEDevice.h"

// BLE service and characteristic UUIDs (16-bit UUIDs for custom service/char)
static BLEUUID serviceUUID((uint16_t)0xFF00);
static BLEUUID charUUID((uint16_t)0xFF01);

// Target device name to search for
const char* targetDeviceName = "XiaoBLE-AccelStream";

// Flags and global objects for connection logic
static BLEAdvertisedDevice*  advDevice   = nullptr;
static BLERemoteCharacteristic* remoteChar = nullptr;
static bool doConnect = false;
static bool connected = false;
static bool doScan = false;

// Notification callback: called whenever a notification is received
static void notifyCallback(BLERemoteCharacteristic* pRC, uint8_t* pData, size_t length, bool isNotify) {
  // Expect 6 bytes (X, Y, Z as int16_t each in little-endian format)
  if (length == 6) {
    // Parse little-endian 16-bit values
    int16_t ax = (int16_t)((pData[1] << 8) | pData[0]);
    int16_t ay = (int16_t)((pData[3] << 8) | pData[2]);
    int16_t az = (int16_t)((pData[5] << 8) | pData[4]);
    // Print as comma-separated values
    Serial.print(ax);
    Serial.print(',');
    Serial.print(ay);
    Serial.print(',');
    Serial.println(az);
    // (For a raw hex format instead, you could use Serial.printf with %04X for each.)
  } else {
    Serial.print("Unexpected notify length: ");
    Serial.println(length);
  }
}

// Client callback class to handle connect/disconnect events
class MyClientCallbacks : public BLEClientCallbacks {
  void onConnect(BLEClient* pClient) override {
    // Connection established
    Serial.println("** Connected to peripheral **");
  }

  void onDisconnect(BLEClient* pClient) override {
    // Lost connection -> restart scan & reconnect
    Serial.println("** Disconnected from peripheral **");
    connected = false;
    doScan = true;  // flag to restart scanning in loop
  }
};

// Advertised Device callback: called for each device found during scanning
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) override {
    // Check if the found device is our target peripheral
    String name = advertisedDevice.getName();
    if (advertisedDevice.haveName() && name == targetDeviceName) {
      Serial.print("Found target device: ");
      Serial.println(name.c_str());
      // Stop scanning, we found the device we want
      BLEDevice::getScan()->stop();       // stop further scanning&#8203;:contentReference[oaicite:6]{index=6}
      advDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;                   // flag to establish connection in loop
      doScan = false;
      Serial.println("Scanning stopped. Connecting to the device...");
    }
  }
};

// Function to connect to the discovered BLE server (peripheral)
bool connectToServer() {
  if (advDevice == nullptr) return false;
  Serial.print("Connecting to ");
  Serial.println(advDevice->getAddress().toString().c_str());

  // Create a new client and set callback for connect/disconnect
  BLEClient* pClient = BLEDevice::createClient();
  pClient->setClientCallbacks(new MyClientCallbacks());

  // Connect to the remote BLE server
  bool success = pClient->connect(advDevice);  // Connect using the BLEAdvertisedDevice
  if (!success) {
    Serial.println("Failed to connect. Retrying...");
    return false;
  }
  Serial.println("Connected to server, discovering services...");

  // Get the custom service by UUID
  BLERemoteService* remoteService = pClient->getService(serviceUUID);
  if (remoteService == nullptr) {
    Serial.println("Failed to find service UUID 0xFF00 on peripheral.");
    pClient->disconnect();
    return false;
  }
  Serial.println("Found service 0xFF00");

  // Get the characteristic in the service by UUID
  remoteChar = remoteService->getCharacteristic(charUUID);
  if (remoteChar == nullptr) {
    Serial.println("Failed to find char UUID 0xFF01.");
    pClient->disconnect();
    return false;
  }
  Serial.println("Found characteristic 0xFF01");

  // If this characteristic supports notifications, register the callback
  if (remoteChar->canNotify()) {
    remoteChar->registerForNotify(notifyCallback);  // subscribe to notifications&#8203;:contentReference[oaicite:7]{index=7}
    Serial.println("Subscribed to notifications.");
  }

  connected = true;
  return true;
}

void setup() {
  Serial.begin(115200);
  // Tip: Increase baud rate if needed for high throughput logging (e.g., 1000000).

  // Initialize BLE and start scanning
  Serial.println("Initializing BLE client...");
  BLEDevice::init("ESP32AccelClient");  // Initialize BLE with a device name for our central
  BLEDevice::setPower(ESP_PWR_LVL_P9);  // Set radio power to maximum (for reliability)

  // Configure scanning parameters
  BLEScan* pScan = BLEDevice::getScan();
  pScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pScan->setInterval(1349);  // scan interval in 0.625ms units
  pScan->setWindow(449);     // scan window in 0.625ms units
  pScan->setActiveScan(true);// active scanning to get scan response (for name)
  
  Serial.println("Starting scan for device...");
  pScan->start(0, false);    // start scanning indefinitely (until we manually stop)
}

void loop() {
  // If we found the device and need to connect
  if (doConnect) {
    doConnect = false;
    // Attempt to connect to the peripheral
    if (connectToServer()) {
      Serial.println(">> Successfully connected and subscribed!");
    } else {
      Serial.println("Connection failed, resetting scan...");
      // If connection failed, restart scanning
      BLEDevice::getScan()->start(0, false);
    }
  }

  // If connected, nothing else to do in loop (notifications handled in callback)
  // If not connected and scanning is needed (e.g., after a disconnect), restart scan
  if (!connected && doScan) {
    doScan = false;
    Serial.println("Re-starting scan for device...");
    BLEDevice::getScan()->start(0, false);  // resume scanning to find the device&#8203;:contentReference[oaicite:8]{index=8}
  }

  // Small delay to avoid tight loop spinning (give time to BLE background tasks)
  delay(10);
}
