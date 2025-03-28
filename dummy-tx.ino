#include <SPI.h>
#include <SparkFun_KX13X.h>     // SparkFun KX132/KX134 accelerometer library (rawOutputData is defined here)
#include <bluefruit.h>          // Adafruit Bluefruit BLE library
#include <Adafruit_TinyUSB.h>   // TinyUSB for USB CDC (if needed for Serial)
#include <Adafruit_SPIFlash.h>  // SPI Flash (required by Adafruit BSP)

Adafruit_FlashTransport_QSPI flashTransport;

////////////////////
// Pin Definitions:
////////////////////
#define KX134_CS_PIN 1   // Chip select for KX134 (not used in simulation)
#define KX134_INT_PIN 2  // Interrupt pin from KX134 (not used in simulation)

////////////////////
// BLE Service/Characteristic UUIDs (16-bit custom UUIDs)
////////////////////
#define ACCEL_SERVICE_UUID 0xFF00
#define ACCEL_CHAR_UUID    0xFF01

////////////////////
// Global Objects:
////////////////////
SparkFun_KX134_SPI accel;  // KX134 accelerometer object (not used in simulation)
BLEService accelService(ACCEL_SERVICE_UUID);
BLECharacteristic accelChar(ACCEL_CHAR_UUID);

// Flag to simulate the sensor FIFO ready interrupt
volatile bool fifoDataReady = false;

////////////////////
// Global counter for simulated data
////////////////////
volatile uint32_t sampleCounter = 0;

////////////////////
// Interrupt Service Routine (not used in simulation)
// (In simulation we set fifoDataReady in code based on time)
void onAccelDataReady() {
  fifoDataReady = true;
}

////////////////////
// BLE Connect/Disconnect Callbacks
////////////////////
void onBLEConnect(uint16_t conn_hdl) {
  BLEConnection* conn = Bluefruit.Connection(conn_hdl);
  conn->requestPHY();
  conn->requestMtuExchange(247);
  conn->requestConnectionParameter(6, 8);  // 6*1.25ms = 7.5ms interval
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
  Serial.begin(115200);
  while (!Serial) delay(10);

  // Enable DC-DC converter
  NRF_POWER->DCDCEN = 1;  // Enable DC/DC converter for REG1 stage

  // Flash power-down mode
  flashTransport.begin();
  flashTransport.runCommand(0xB9);  // Enter deep power-down mode
  delayMicroseconds(5);
  flashTransport.end();

  ////////////
  // (Simulation) Skip real sensor initialization.
  // Comment out the sensor initialization block.
  /*
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
  accel.setOutputDataRate(14);
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
  */

  ////////////
  // BLE Peripheral Setup
  ////////////
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  Bluefruit.begin();
  Bluefruit.setTxPower(4);
  Bluefruit.setName("XiaoBLE-AccelStream");
  Bluefruit.Periph.setConnectCallback(onBLEConnect);
  Bluefruit.Periph.setDisconnectCallback(onBLEDisconnect);
  Bluefruit.Periph.setConnInterval(6, 12);

  // Define custom service and characteristic for accelerometer data
  accelService.begin();
  accelChar.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ);
  accelChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  accelChar.setMaxLen(244);
  accelChar.setFixedLen(0);
  accelChar.begin();

  // Set up advertising
  Bluefruit.Advertising.clearData();
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addService(accelService);
  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.setInterval(32, 244);
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);
  Serial.println("BLE advertising started.");
}

////////////////////
// Loop Function (Simulation version)
////////////////////
void loop() {
  // Simulation: Use a timer to simulate FIFO ready every ~2.5ms.
  static unsigned long lastSimTime = 0;
  unsigned long now = micros();
  if (now - lastSimTime >= 2500) {  // 2.5ms interval ~ 20 samples at 8kHz
    fifoDataReady = true;
    lastSimTime = now;
  }

  // Process simulated FIFO data when ready
  if (fifoDataReady) {
    fifoDataReady = false;  // reset flag

    uint8_t packetBuf[244];  // buffer for one BLE packet (up to 244 bytes)
    uint16_t bufLen = 0;
    static uint8_t skipCount = 0;  // decimation counter for ~8kHz output

    // Simulate a batch of 20 samples
    int samplesAvailable = 20;
    for (int i = 0; i < samplesAvailable; i++) {
      rawOutputData raw;
      // Simulate counter values: X = counter, Y = counter+1, Z = counter+2
      raw.xData = (int16_t)(sampleCounter & 0xFFFF);
      raw.yData = (int16_t)((sampleCounter + 1) & 0xFFFF);
      raw.zData = (int16_t)((sampleCounter + 2) & 0xFFFF);
      sampleCounter++;  // Increment for next sample

      // Decimation: skip 1 out of every 3 samples
      if (skipCount == 2) {
        skipCount = 0;
        continue;  // do not pack this sample
      } else {
        int16_t x = raw.xData;
        int16_t y = raw.yData;
        int16_t z = raw.zData;
        packetBuf[bufLen++] = (uint8_t)(x & 0xFF);
        packetBuf[bufLen++] = (uint8_t)((x >> 8) & 0xFF);
        packetBuf[bufLen++] = (uint8_t)(y & 0xFF);
        packetBuf[bufLen++] = (uint8_t)((y >> 8) & 0xFF);
        packetBuf[bufLen++] = (uint8_t)(z & 0xFF);
        packetBuf[bufLen++] = (uint8_t)((z >> 8) & 0xFF);
      }
      skipCount++;

      // If packet buffer is nearly full, send a BLE notification
      if (bufLen >= 240) {
        bool sent = accelChar.notify(packetBuf, bufLen);
        if (!sent) {
          Serial.println("BLE notify buffer full! Data dropped.");
        }
        bufLen = 0;
      }
    }

    // Send any remaining data in packetBuf
    if (bufLen > 0) {
      bool sent = accelChar.notify(packetBuf, bufLen);
      if (!sent) {
        Serial.println("BLE notify buffer full! Data dropped.");
      }
      bufLen = 0;
    }
    
    // In simulation, no sensor interrupt latch to clear.
  }

  // Small delay for BLE background processing
  delay(1);
}
