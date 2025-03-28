// // Set DEBUG_MODE to 1 to use dummy data (and enable Serial prints for debugging)
// // Set DEBUG_MODE to 0 to use real sensor data and disable Serial debug messages.
// #define DEBUG_MODE 1

// #include <SPI.h>
// #include <SparkFun_KX13X.h>     // Library defines rawOutputData (used for data format)
// #include <bluefruit.h>          // Adafruit Bluefruit BLE library
// #include <Adafruit_TinyUSB.h>   // USB CDC for Serial
// #include <Adafruit_SPIFlash.h>  // SPI Flash support
// #include <PDM.h>                // Arduino PDM library for onboard microphone

// // Flash transport for internal flash operations
// Adafruit_FlashTransport_QSPI flashTransport;

// ////////////////////
// // Pin Definitions:
// ////////////////////
// #define KX134_CS_PIN 1   // Used only when real sensor is attached (DEBUG_MODE == 0)
// #define KX134_INT_PIN 2  // Used only when real sensor is attached (DEBUG_MODE == 0)

// ////////////////////
// // BLE Service/Characteristic UUIDs:
// ////////////////////
// #define ACCEL_SERVICE_UUID 0xFF00
// #define ACCEL_CHAR_UUID    0xFF01

// ////////////////////
// // Global Objects:
// ////////////////////
// SparkFun_KX134_SPI accel;         // KX134 accelerometer object
// BLEService accelService(ACCEL_SERVICE_UUID);
// BLECharacteristic accelChar(ACCEL_CHAR_UUID);

// // Flag for sensor interrupt trigger
// volatile bool fifoDataReady = false;

// // Decimation counters (assume sensor/PDM sampling at ~16 kHz; we use every 2nd sample for ~8 kHz)
// static uint32_t accelDecimCount = 0;
// static uint32_t pdmDecimCount   = 0;

// // Aggregation settings:
// // We aggregate 30 sample pairs per BLE notification.
// // Each sample pair is 8 bytes: 6 bytes (accel) + 2 bytes (PDM).
// const uint8_t SAMPLE_PAIRS_PER_PKT = 30;
// const uint16_t PACKET_SIZE = SAMPLE_PAIRS_PER_PKT * 8; // 240 bytes

// // Buffer for aggregated BLE notification
// uint8_t aggPacket[PACKET_SIZE];

// // For simulation timing (if no real sensor interrupt)
// static unsigned long lastSimTime = 0;

// // Dummy sample counter for dummy accelerometer data (used in DEBUG_MODE==1)
// static uint32_t dummySampleCounter = 0;

// ////////////////////
// // Simulated Accelerometer ISR (for dummy data)
// // In a real system, the sensor's interrupt triggers this.
// void onAccelDataReady() {
//   fifoDataReady = true;
// }

// ////////////////////
// // Helper Function: Generate one dummy accelerometer sample (for DEBUG_MODE==1)
// // Format: X = dummySampleCounter, Y = dummySampleCounter+1, Z = dummySampleCounter+2.
// bool getDummyAccelSample(rawOutputData &sample) {
//   sample.xData = (int16_t)(dummySampleCounter & 0xFFFF);
//   sample.yData = (int16_t)((dummySampleCounter + 1) & 0xFFFF);
//   sample.zData = (int16_t)((dummySampleCounter + 2) & 0xFFFF);
//   dummySampleCounter++;
  
//   accelDecimCount++;
//   // Use only every 2nd sample
//   if ((accelDecimCount % 2) == 0) {
//     return true;
//   }
//   return false;
// }

// ////////////////////
// // Helper Function: Get one decimated accelerometer sample from sensor (DEBUG_MODE==0)
// bool getDecimatedAccelSample(rawOutputData &sample) {
//   if (accel.getRawAccelBufferData(&sample, 1)) {
//     accelDecimCount++;
//     if ((accelDecimCount % 2) == 0) {
//       return true;
//     }
//   }
//   return false;
// }

// ////////////////////
// // Helper Function: Get one decimated PDM sample (16-bit)
// bool getDecimatedPDMSample(int16_t &pdmSample) {
//   int bytesAvailable = PDM.available();
//   if (bytesAvailable >= 2) {
//     uint8_t tempBuffer[256];
//     int bytesRead = PDM.read(tempBuffer, bytesAvailable);
//     pdmDecimCount++;
//     int16_t sample = (int16_t)((tempBuffer[1] << 8) | tempBuffer[0]);
//     if ((pdmDecimCount % 2) == 0) {
//       pdmSample = sample;
//       return true;
//     }
//   }
//   return false;
// }

// ////////////////////
// // BLE Connection Callbacks
// ////////////////////
// void onBLEConnect(uint16_t conn_hdl) {
//   BLEConnection* conn = Bluefruit.Connection(conn_hdl);
//   conn->requestPHY();
//   conn->requestMtuExchange(247);
//   conn->requestConnectionParameter(6, 8);  // 7.5ms interval
// }

// void onBLEDisconnect(uint16_t conn_hdl, uint8_t reason) {
//   (void)conn_hdl;
//   (void)reason;
// #if DEBUG_MODE == 1
//   Serial.println("BLE disconnected!");
// #endif
// }

// ////////////////////
// // Setup Function
// ////////////////////
// void setup() {
// #if DEBUG_MODE == 1
//   Serial.begin(115200);
//   while (!Serial) delay(10);
// #else
//   // When using real sensor data, you might disable Serial prints.
//   // Serial.begin(115200);
// #endif

//   // Enable DC-DC converter
//   NRF_POWER->DCDCEN = 1;

//   // Flash power-down mode
//   flashTransport.begin();
//   flashTransport.runCommand(0xB9);
//   delayMicroseconds(5);
//   flashTransport.end();

//   /////////////////
//   // Sensor Initialization
//   /////////////////
// #if DEBUG_MODE == 0
//   // If DEBUG_MODE is 0, initialize the real sensor.
//   pinMode(KX134_CS_PIN, OUTPUT);
//   digitalWrite(KX134_CS_PIN, HIGH);
//   SPISettings settings(8000000, MSBFIRST, SPI_MODE0);
//   SPI.begin();
//   if (!accel.begin(SPI, settings, KX134_CS_PIN)) {
//     Serial.println("KX134 not detected. Check wiring!");
//     while (1) delay(10);
//   }
//   Serial.println("KX134 initialized.");
//   accel.softwareReset();
//   delay(5);
//   accel.enableAccel(false);
//   accel.setRange(SFE_KX134_RANGE64G);
//   accel.enableDataEngine();
//   accel.setOutputDataRate(14);  // 12.8 kHz raw; decimated to ~8 kHz
//   accel.setBufferResolution();
//   accel.enableSampleBuffer();
//   accel.setBufferOperationMode(0x00);
//   const uint8_t fifoThreshold = 20;
//   accel.setBufferThreshold(fifoThreshold);
//   accel.enableBufferInt();
//   accel.enablePhysInterrupt(true, 1);
//   accel.routeHardwareInterrupt(0x40, 1);
//   accel.clearInterrupt();
//   accel.enableAccel(true);

//   pinMode(KX134_INT_PIN, INPUT_PULLUP);
//   attachInterrupt(digitalPinToInterrupt(KX134_INT_PIN), onAccelDataReady, FALLING);
// #else
//   // In DEBUG_MODE==1, we use dummy data. You can optionally start Serial prints.
//   // Also, you can simulate the sensor interrupt in the loop using a timer.
// #endif

//   /////////////////
//   // BLE Peripheral Setup
//   /////////////////
//   Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
//   Bluefruit.begin();
//   Bluefruit.setTxPower(4);
//   Bluefruit.setName("XiaoBLE-AccelStream");
//   Bluefruit.Periph.setConnectCallback(onBLEConnect);
//   Bluefruit.Periph.setDisconnectCallback(onBLEDisconnect);
//   Bluefruit.Periph.setConnInterval(6, 12);

//   accelService.begin();
//   accelChar.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ);
//   accelChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
//   accelChar.setMaxLen(244);
//   accelChar.setFixedLen(0);
//   accelChar.begin();

//   Bluefruit.Advertising.clearData();
//   Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
//   Bluefruit.Advertising.addService(accelService);
//   Bluefruit.ScanResponse.addName();
//   Bluefruit.Advertising.setInterval(32, 244);
//   Bluefruit.Advertising.setFastTimeout(30);
//   Bluefruit.Advertising.start(0);
// #if DEBUG_MODE == 1
//   Serial.println("BLE advertising started.");
// #endif

//   /////////////////
//   // PDM Microphone Setup
//   /////////////////
//   if (!PDM.begin(1, 16000)) {  // 1 channel at 16 kHz
//   #if DEBUG_MODE == 1
//     Serial.println("Failed to start PDM!");
//   #endif
//   } else {
//   #if DEBUG_MODE == 1
//     Serial.println("PDM started.");
//   #endif
//   }
//   PDM.setBufferSize(256);
//   // Use a simple onReceive callback; actual processing is done in loop.
//   PDM.onReceive([](){
//     // No action needed here.
//   });
// }

// ////////////////////
// // Loop Function: Aggregate 30 sample pairs and send one BLE notification.
// ////////////////////
// void loop() {
//   // For simulation: if no real sensor interrupt, simulate trigger every ~125 µs to approximate 8 kHz.
//   unsigned long now = micros();
// #if DEBUG_MODE == 1
//   if (now - lastSimTime >= 125) {
//     fifoDataReady = true;
//     lastSimTime = now;
//   }
// #endif

//   static uint8_t samplePairCount = 0;  // Count of sample pairs accumulated
//   uint16_t offset = samplePairCount * 8; // Each sample pair is 8 bytes

//   rawOutputData sensorSample;
// #if DEBUG_MODE == 1
//   // Use dummy data when in debug mode.
//   bool gotAccel = getDummyAccelSample(sensorSample);
// #else
//   // Use real sensor data when DEBUG_MODE is 0.
//   bool gotAccel = getDecimatedAccelSample(sensorSample);
// #endif

//   // Get one decimated PDM sample (common for both modes)
//   int16_t pdmSample;
//   bool gotPDM = getDecimatedPDMSample(pdmSample);

//   // Only if both samples are available, pack them as an 8-byte pair.
//   if (gotAccel && gotPDM) {
//     // Pack accelerometer sample (6 bytes, little-endian)
//     aggPacket[offset++] = (uint8_t)(sensorSample.xData & 0xFF);
//     aggPacket[offset++] = (uint8_t)((sensorSample.xData >> 8) & 0xFF);
//     aggPacket[offset++] = (uint8_t)(sensorSample.yData & 0xFF);
//     aggPacket[offset++] = (uint8_t)((sensorSample.yData >> 8) & 0xFF);
//     aggPacket[offset++] = (uint8_t)(sensorSample.zData & 0xFF);
//     aggPacket[offset++] = (uint8_t)((sensorSample.zData >> 8) & 0xFF);
//     // Pack PDM sample (2 bytes, little-endian)
//     aggPacket[offset++] = (uint8_t)(pdmSample & 0xFF);
//     aggPacket[offset++] = (uint8_t)((pdmSample >> 8) & 0xFF);

//     samplePairCount++;

//     // When we've accumulated 30 sample pairs, send the aggregated packet via BLE.
//     if (samplePairCount >= SAMPLE_PAIRS_PER_PKT) {
//       bool sent = accelChar.notify(aggPacket, PACKET_SIZE);
// #if DEBUG_MODE == 1
//       if (!sent) {
//         Serial.println("BLE notify buffer full! Data dropped.");
//       } else {
//         Serial.print("Sent packet with ");
//         Serial.print(SAMPLE_PAIRS_PER_PKT);
//         Serial.println(" sample pairs.");
//       }
// #endif
//       samplePairCount = 0;  // Reset for the next packet.
//     }
//   }

// #if DEBUG_MODE == 1
//   // In debug mode, print extra messages if needed.
//   // (Optional: add debug prints here)
// #endif

//   // Clear sensor interrupt latch (if applicable)
//   accel.clearInterrupt();

//   // Small delay to yield CPU time.
//   delay(1);
// }


// Set DEBUG_MODE to 1 for dummy data (accelerometer and PDM)
// Set DEBUG_MODE to 0 to use real sensor data (if available)
#define DEBUG_MODE 1

#include <SPI.h>
#include <SparkFun_KX13X.h>     // Library defines rawOutputData (used for data format)
#include <bluefruit.h>          // Adafruit Bluefruit BLE library
#include <Adafruit_TinyUSB.h>   // USB CDC for Serial
#include <Adafruit_SPIFlash.h>  // SPI Flash support
#include <PDM.h>                // Arduino PDM library (not used in dummy mode)

Adafruit_FlashTransport_QSPI flashTransport;

////////////////////
// Pin Definitions:
////////////////////
#define KX134_CS_PIN 1   // Used when real sensor is attached (DEBUG_MODE==0)
#define KX134_INT_PIN 2  // Used when real sensor is attached (DEBUG_MODE==0)

////////////////////
// BLE Service/Characteristic UUIDs:
////////////////////
#define ACCEL_SERVICE_UUID 0xFF00
#define ACCEL_CHAR_UUID    0xFF01

////////////////////
// Global Objects:
////////////////////
SparkFun_KX134_SPI accel;         // Accelerometer object
BLEService accelService(ACCEL_SERVICE_UUID);
BLECharacteristic accelChar(ACCEL_CHAR_UUID);

// Flag for sensor interrupt trigger (dummy simulation)
volatile bool fifoDataReady = false;

// Decimation counters – assume raw sampling at ~16 kHz; we use every 2nd sample for ~8 kHz.
static uint32_t accelDecimCount = 0;
static uint32_t pdmDecimCount   = 0;

// Aggregation settings:
// We aggregate 30 sample pairs per BLE notification.
// Each sample pair is 8 bytes: 6 bytes (accelerometer) + 2 bytes (PDM).
const uint8_t SAMPLE_PAIRS_PER_PKT = 30;
const uint16_t PACKET_SIZE = SAMPLE_PAIRS_PER_PKT * 8; // 240 bytes

// Buffer for aggregated BLE notification
uint8_t aggPacket[PACKET_SIZE];

// Timing for simulation trigger
static unsigned long lastSimTime = 0;

// Dummy sample counters for simulation
static uint32_t dummyAccelCounter = 0;
static uint32_t dummyPdmCounter   = 0;

////////////////////
// Simulated Accelerometer ISR (dummy)
////////////////////
void onAccelDataReady() {
  fifoDataReady = true;
}

////////////////////
// Helper Function: Generate one dummy accelerometer sample (DEBUG_MODE==1)
// Format: X = dummyAccelCounter, Y = dummyAccelCounter+1, Z = dummyAccelCounter+2
bool getDummyAccelSample(rawOutputData &sample) {
  sample.xData = (int16_t)(dummyAccelCounter & 0xFFFF);
  sample.yData = (int16_t)((dummyAccelCounter + 1) & 0xFFFF);
  sample.zData = (int16_t)((dummyAccelCounter + 2) & 0xFFFF);
  dummyAccelCounter++;
  
  accelDecimCount++;
  if ((accelDecimCount % 2) == 0) {
    return true;
  }
  return false;
}

////////////////////
// Helper Function: Get one dummy PDM sample (DEBUG_MODE==1)
// We'll simulate a 16-bit PDM sample using a counter.
bool getDummyPDMSample(int16_t &pdmSample) {
  dummyPdmCounter++;
  pdmDecimCount++;
  if ((pdmDecimCount % 2) == 0) {
    pdmSample = (int16_t)(dummyPdmCounter & 0xFFFF);
    return true;
  }
  return false;
}

////////////////////
// Helper Function: Get one decimated accelerometer sample from sensor (DEBUG_MODE==0)
bool getDecimatedAccelSample(rawOutputData &sample) {
  if (accel.getRawAccelBufferData(&sample, 1)) {
    accelDecimCount++;
    if ((accelDecimCount % 2) == 0) {
      return true;
    }
  }
  return false;
}

////////////////////
// Helper Function: Get one decimated PDM sample from hardware (DEBUG_MODE==0)
// This function reads real PDM data.
bool getDecimatedPDMSample(int16_t &pdmSample) {
  int bytesAvailable = PDM.available();
  if (bytesAvailable >= 2) {
    uint8_t tempBuffer[256];
    int bytesRead = PDM.read(tempBuffer, bytesAvailable);
    pdmDecimCount++;
    int16_t sample = (int16_t)((tempBuffer[1] << 8) | tempBuffer[0]);
    if ((pdmDecimCount % 2) == 0) {
      pdmSample = sample;
      return true;
    }
  }
  return false;
}

////////////////////
// BLE Connection Callbacks
////////////////////
void onBLEConnect(uint16_t conn_hdl) {
  BLEConnection* conn = Bluefruit.Connection(conn_hdl);
  conn->requestPHY();
  conn->requestMtuExchange(247);
  conn->requestConnectionParameter(6, 8);  // 7.5ms interval
}

void onBLEDisconnect(uint16_t conn_hdl, uint8_t reason) {
  (void)conn_hdl;
  (void)reason;
  Serial.println("BLE disconnected!");
}

////////////////////
// Setup Function
////////////////////
void setup() {
#if DEBUG_MODE == 1
  Serial.begin(115200);
  while (!Serial) delay(10);
#endif

  // Enable DC-DC converter for power efficiency.
  NRF_POWER->DCDCEN = 1;

  // Flash power-down mode.
  flashTransport.begin();
  flashTransport.runCommand(0xB9);
  delayMicroseconds(5);
  flashTransport.end();

  /////////////////
  // Sensor Initialization
  /////////////////
#if DEBUG_MODE == 0
  // Real sensor initialization (if available)
  pinMode(KX134_CS_PIN, OUTPUT);
  digitalWrite(KX134_CS_PIN, HIGH);
  SPISettings settings(8000000, MSBFIRST, SPI_MODE0);
  SPI.begin();
  if (!accel.begin(SPI, settings, KX134_CS_PIN)) {
    Serial.println("KX134 not detected. Check wiring!");
    while (1) delay(10);
  }
  Serial.println("KX134 initialized.");
  accel.softwareReset();
  delay(5);
  accel.enableAccel(false);
  accel.setRange(SFE_KX134_RANGE64G);
  accel.enableDataEngine();
  accel.setOutputDataRate(14);  // ~12.8 kHz raw, decimated to ~8 kHz
  accel.setBufferResolution();
  accel.enableSampleBuffer();
  accel.setBufferOperationMode(0x00);
  const uint8_t fifoThreshold = 20;
  accel.setBufferThreshold(fifoThreshold);
  accel.enableBufferInt();
  accel.enablePhysInterrupt(true, 1);
  accel.routeHardwareInterrupt(0x40, 1);
  accel.clearInterrupt();
  accel.enableAccel(true);
  
  pinMode(KX134_INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(KX134_INT_PIN), onAccelDataReady, FALLING);
#else
  // For dummy data, we skip real sensor init.
  // We simulate sensor interrupts in loop.
#endif

  /////////////////
  // BLE Peripheral Setup
  /////////////////
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  Bluefruit.begin();
  Bluefruit.setTxPower(4);
  Bluefruit.setName("XiaoBLE-AccelStream");
  Bluefruit.Periph.setConnectCallback(onBLEConnect);
  Bluefruit.Periph.setDisconnectCallback(onBLEDisconnect);
  Bluefruit.Periph.setConnInterval(6, 12);

  accelService.begin();
  accelChar.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ);
  accelChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  accelChar.setMaxLen(244);  // Maximum payload (MTU negotiated)
  accelChar.setFixedLen(0);
  accelChar.begin();

  Bluefruit.Advertising.clearData();
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addService(accelService);
  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.setInterval(32, 244);
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);
#if DEBUG_MODE == 1
  Serial.println("BLE advertising started.");
#endif

  /////////////////
  // PDM Microphone Setup
  /////////////////
#if DEBUG_MODE == 0
  // For real PDM sensor data:
  if (!PDM.begin(1, 16000)) {  // 1 channel at 16 kHz
    Serial.println("Failed to start PDM!");
  } else {
    Serial.println("PDM started.");
  }
  PDM.setBufferSize(256);
  PDM.onReceive([](){
    // No action needed here; processing is in loop.
  });
#else
  // For dummy PDM data, skip hardware init.
  // Optionally, you can initialize PDM if you want to test real PDM even in dummy mode.
  Serial.println("Dummy PDM mode: using simulated data.");
#endif
}

////////////////////
// Loop Function: Aggregate 30 sample pairs and send one BLE notification.
////////////////////
void loop() {
  // For simulation: If no real sensor interrupt, simulate a trigger every ~125 µs (approx. 8 kHz).
#if DEBUG_MODE == 1
  unsigned long now = micros();
  if (now - lastSimTime >= 125) {
    fifoDataReady = true;
    lastSimTime = now;
  }
#endif

  static uint8_t samplePairCount = 0;  // Count of sample pairs aggregated
  uint16_t offset = samplePairCount * 8; // Each sample pair is 8 bytes

  rawOutputData accelSample;
  bool gotAccel = false;
#if DEBUG_MODE == 1
  gotAccel = getDummyAccelSample(accelSample);
#else
  gotAccel = getDecimatedAccelSample(accelSample);
#endif

  int16_t pdmSample;
  bool gotPDM = false;
#if DEBUG_MODE == 1
  // Generate dummy PDM sample using a counter.
  static uint32_t dummyPdmCounter = 0;
  dummyPdmCounter++;
  pdmDecimCount++;
  if ((pdmDecimCount % 2) == 0) {
    pdmSample = (int16_t)(dummyPdmCounter & 0xFFFF);
    gotPDM = true;
  }
#else
  gotPDM = getDecimatedPDMSample(pdmSample);
#endif

  // Only if both dummy samples are available, pack them as an 8-byte pair.
  if (gotAccel && gotPDM) {
    // Pack accelerometer sample (6 bytes, little-endian)
    aggPacket[offset++] = (uint8_t)(accelSample.xData & 0xFF);
    aggPacket[offset++] = (uint8_t)((accelSample.xData >> 8) & 0xFF);
    aggPacket[offset++] = (uint8_t)(accelSample.yData & 0xFF);
    aggPacket[offset++] = (uint8_t)((accelSample.yData >> 8) & 0xFF);
    aggPacket[offset++] = (uint8_t)(accelSample.zData & 0xFF);
    aggPacket[offset++] = (uint8_t)((accelSample.zData >> 8) & 0xFF);
    // Pack PDM sample (2 bytes, little-endian)
    aggPacket[offset++] = (uint8_t)(pdmSample & 0xFF);
    aggPacket[offset++] = (uint8_t)((pdmSample >> 8) & 0xFF);
    
    samplePairCount++;
    
    // When we've accumulated 30 sample pairs, send the aggregated packet.
    if (samplePairCount >= SAMPLE_PAIRS_PER_PKT) {
      bool sent = accelChar.notify(aggPacket, PACKET_SIZE);
#if DEBUG_MODE == 1
      if (!sent) {
        Serial.println("BLE notify buffer full! Data dropped.");
      } else {
        Serial.print("Sent packet with ");
        Serial.print(SAMPLE_PAIRS_PER_PKT);
        Serial.println(" sample pairs (dummy data).");
      }
#endif
      samplePairCount = 0;  // Reset for next packet.
    }
  }

#if DEBUG_MODE == 1
  // For debugging, you might print occasional status messages.
#endif
  
  // Clear sensor interrupt latch (if applicable)
  accel.clearInterrupt();
  
  // Short delay to yield CPU time.
  delay(1);
}


// #include <SPI.h>
// #include <SparkFun_KX13X.h>     // Defines rawOutputData (used for data format)
// #include <bluefruit.h>          // Adafruit Bluefruit BLE library
// #include <Adafruit_TinyUSB.h>   // USB CDC for Serial
// #include <Adafruit_SPIFlash.h>  // SPI Flash support
// #include <PDM.h>                // Arduino PDM library (not used in dummy mode)

// // Flash transport (for internal flash operations)
// Adafruit_FlashTransport_QSPI flashTransport;

// ////////////////////
// // Pin Definitions:
// ////////////////////
// #define KX134_CS_PIN 1   // Not used in dummy mode
// #define KX134_INT_PIN 2  // Not used in dummy mode

// ////////////////////
// // BLE Service/Characteristic UUIDs:
// ////////////////////
// #define ACCEL_SERVICE_UUID 0xFF00
// #define ACCEL_CHAR_UUID    0xFF01

// ////////////////////
// // Global Objects:
// ////////////////////
// BLEService accelService(ACCEL_SERVICE_UUID);
// BLECharacteristic accelChar(ACCEL_CHAR_UUID);

// // Flag for dummy sensor data trigger
// volatile bool fifoDataReady = false;

// // Decimation counters – assume raw rate is 16 kHz; we use every 2nd sample for an effective 8 kHz.
// static uint32_t accelDecimCount = 0;
// static uint32_t pdmDecimCount   = 0;

// // Aggregation settings:
// // Aggregate 30 sample pairs per BLE notification.
// // Each pair is 8 bytes: 6 bytes for dummy accelerometer + 2 bytes for dummy PDM.
// const uint8_t SAMPLE_PAIRS_PER_PKT = 30;
// const uint16_t PACKET_SIZE = SAMPLE_PAIRS_PER_PKT * 8; // 240 bytes

// // Buffer for aggregated BLE notification
// uint8_t aggPacket[PACKET_SIZE];

// // Timing for simulation trigger
// static unsigned long lastSimTime = 0;

// // Dummy counters for simulated data
// static uint32_t dummyAccelCounter = 0;
// static uint32_t dummyPdmCounter   = 0;

// ////////////////////
// // Simulated Sensor ISR (dummy trigger)
// ////////////////////
// void onAccelDataReady() {
//   fifoDataReady = true;
// }

// ////////////////////
// // Helper Function: Generate one dummy accelerometer sample
// // Format: X = dummyAccelCounter, Y = dummyAccelCounter+1, Z = dummyAccelCounter+2.
// bool getDummyAccelSample(rawOutputData &sample) {
//   sample.xData = (int16_t)(dummyAccelCounter & 0xFFFF);
//   sample.yData = (int16_t)((dummyAccelCounter + 1) & 0xFFFF);
//   sample.zData = (int16_t)((dummyAccelCounter + 2) & 0xFFFF);
//   dummyAccelCounter++;
  
//   accelDecimCount++;
//   if ((accelDecimCount % 2) == 0) {
//     return true;
//   }
//   return false;
// }

// ////////////////////
// // Helper Function: Generate one dummy PDM sample (16-bit)
// bool getDummyPDMSample(int16_t &pdmSample) {
//   dummyPdmCounter++;
//   pdmDecimCount++;
//   if ((pdmDecimCount % 2) == 0) {
//     pdmSample = (int16_t)(dummyPdmCounter & 0xFFFF);
//     return true;
//   }
//   return false;
// }

// ////////////////////
// // BLE Connection Callbacks
// ////////////////////
// void onBLEConnect(uint16_t conn_hdl) {
//   BLEConnection* conn = Bluefruit.Connection(conn_hdl);
//   conn->requestPHY();
//   conn->requestMtuExchange(247);
//   conn->requestConnectionParameter(6, 8);  // 7.5ms interval
// }

// void onBLEDisconnect(uint16_t conn_hdl, uint8_t reason) {
//   (void)conn_hdl;
//   (void)reason;
// }

// ////////////////////
// // Setup Function
// ////////////////////
// void setup() {
//   Serial.begin(115200);
//   while (!Serial) delay(10);

//   NRF_POWER->DCDCEN = 1;
//   flashTransport.begin();
//   flashTransport.runCommand(0xB9);
//   delayMicroseconds(5);
//   flashTransport.end();

//   // Skip real sensor initialization; we are using dummy data.

//   // BLE Peripheral Setup
//   Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
//   Bluefruit.begin();
//   Bluefruit.setTxPower(4);
//   Bluefruit.setName("XiaoBLE-AccelStream");
//   Bluefruit.Periph.setConnectCallback(onBLEConnect);
//   Bluefruit.Periph.setDisconnectCallback(onBLEDisconnect);
//   Bluefruit.Periph.setConnInterval(6, 12);

//   accelService.begin();
//   accelChar.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ);
//   accelChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
//   accelChar.setMaxLen(244);
//   accelChar.setFixedLen(0);
//   accelChar.begin();

//   Bluefruit.Advertising.clearData();
//   Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
//   Bluefruit.Advertising.addService(accelService);
//   Bluefruit.ScanResponse.addName();
//   Bluefruit.Advertising.setInterval(32, 244);
//   Bluefruit.Advertising.setFastTimeout(30);
//   Bluefruit.Advertising.start(0);
//   Serial.println("BLE advertising started.");

//   // Since no real sensor is present, we don't initialize PDM hardware.
//   Serial.println("Dummy mode: using simulated PDM data.");
// }

// ////////////////////
// // Loop Function: Aggregate 30 sample pairs and send one BLE notification.
// ////////////////////
// void loop() {
//   // Simulate sensor interrupt trigger if not set by hardware.
//   unsigned long now = micros();
//   if (now - lastSimTime >= 125) {  // Approximately 125 µs trigger for 8 kHz rate
//     fifoDataReady = true;
//     lastSimTime = now;
//   }
  
//   static uint8_t samplePairCount = 0;  // Number of sample pairs aggregated so far
//   uint16_t offset = samplePairCount * 8; // Each pair is 8 bytes

//   rawOutputData dummyAccel;
//   bool gotAccel = getDummyAccelSample(dummyAccel);
  
//   int16_t dummyPdm;
//   bool gotPDM = getDummyPDMSample(dummyPdm);
  
//   if (gotAccel && gotPDM) {
//     // Pack dummy accelerometer sample (6 bytes)
//     aggPacket[offset++] = (uint8_t)(dummyAccel.xData & 0xFF);
//     aggPacket[offset++] = (uint8_t)((dummyAccel.xData >> 8) & 0xFF);
//     aggPacket[offset++] = (uint8_t)(dummyAccel.yData & 0xFF);
//     aggPacket[offset++] = (uint8_t)((dummyAccel.yData >> 8) & 0xFF);
//     aggPacket[offset++] = (uint8_t)(dummyAccel.zData & 0xFF);
//     aggPacket[offset++] = (uint8_t)((dummyAccel.zData >> 8) & 0xFF);
//     // Pack dummy PDM sample (2 bytes)
//     aggPacket[offset++] = (uint8_t)(dummyPdm & 0xFF);
//     aggPacket[offset++] = (uint8_t)((dummyPdm >> 8) & 0xFF);
    
//     samplePairCount++;
    
//     // When we have 30 sample pairs, send the aggregated packet via BLE.
//     if (samplePairCount >= SAMPLE_PAIRS_PER_PKT) {
//       bool sent = accelChar.notify(aggPacket, PACKET_SIZE);
//       if (!sent) {
//         Serial.println("BLE notify buffer full! Data dropped.");
//       } else {
//         Serial.print("Sent packet with ");
//         Serial.print(SAMPLE_PAIRS_PER_PKT);
//         Serial.println(" sample pairs.");
//       }
//       samplePairCount = 0;  // Reset for next packet.
//     }
//   }
  
//   // No need to clear interrupt latch since we're simulating.
//   delay(1);
// }
