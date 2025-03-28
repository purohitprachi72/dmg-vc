#include <SPI.h>
#include <SparkFun_KX13X.h>     // SparkFun KX132/KX134 accelerometer library
#include <bluefruit.h>          // Adafruit Bluefruit nRF52 BLE library
#include <Adafruit_TinyUSB.h>   // TinyUSB for USB CDC (if needed for Serial)
#include <Adafruit_SPIFlash.h>  // SPI Flash (required by Adafruit BSP)


Adafruit_FlashTransport_QSPI flashTransport;

////////////////////
// Pin Definitions:
////////////////////
#define KX134_CS_PIN 1   // Chip select for KX134 (choose an unused digital pin)
#define KX134_INT_PIN 2  // Interrupt pin from KX134 (INT1 routed here)

// BLE Service/Characteristic UUIDs (16-bit custom UUIDs for simplicity)
#define ACCEL_SERVICE_UUID 0xFF00
#define ACCEL_CHAR_UUID 0xFF01

////////////////////
// Global Objects:
////////////////////
SparkFun_KX134_SPI accel;  // KX134 accelerometer object (±64g capable)
BLEService accelService(ACCEL_SERVICE_UUID);
BLECharacteristic accelChar(ACCEL_CHAR_UUID);

// Flag set by ISR when FIFO threshold interrupt fires
volatile bool fifoDataReady = false;

////////////////////
// Interrupt Service Routine for accelerometer FIFO
////////////////////
void onAccelDataReady() {
  fifoDataReady = true;
}

////////////////////
// Optional: BLE connect/disconnect callbacks for configuring connection
////////////////////
void onBLEConnect(uint16_t conn_hdl) {
  // Get the connection reference
  BLEConnection* conn = Bluefruit.Connection(conn_hdl);
  // Request 2M PHY for high throughput (BLE 5 feature)&#8203;:contentReference[oaicite:5]{index=5}
  conn->requestPHY();
  // Request MTU exchange to 247 bytes (maximum ATT payload 244 bytes)&#8203;:contentReference[oaicite:6]{index=6}
  conn->requestMtuExchange(247);
  // Request update to the fastest connection parameters (7.5ms interval, 0 latency)
  conn->requestConnectionParameter(6, 8);  // 6*1.25ms = 7.5ms, timeout 4s
}

void onBLEDisconnect(uint16_t conn_hdl, uint8_t reason) {
  // (Optional) Reset any state or notify user
  (void)conn_hdl;
  (void)reason;
  Serial.println("BLE disconnected!");
}

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

  /////////////////
  // KX134 Setup
  /////////////////
  pinMode(KX134_CS_PIN, OUTPUT);
  digitalWrite(KX134_CS_PIN, HIGH);
  SPISettings settings(8000000, MSBFIRST, SPI_MODE0);

  SPI.begin();  // Initialize SPI bus
  if (!accel.begin(SPI, settings, KX134_CS_PIN)) {  // start SPI at 10 MHz
    Serial.println("KX134 not detected. Check wiring!");
    while (1) delay(10);
  }
  Serial.println("KX134 initialized.");

  // Perform a software reset to ensure default state
  accel.softwareReset();
  delay(5);  // wait for reset to complete (datasheet: needs ~2ms)

  // Many configuration registers require accelerometer to be in standby
  accel.enableAccel(false);

  // Set full-scale range to ±64g (max for KX134)&#8203;:contentReference[oaicite:7]{index=7}&#8203;:contentReference[oaicite:8]{index=8}
  accel.setRange(SFE_KX134_RANGE64G);
  // Enable the Data Engine's availability flag (data-ready)&#8203;:contentReference[oaicite:9]{index=9}
  accel.enableDataEngine();  // sets bit that indicates data is ready in status

  // Set Output Data Rate (ODR) to high value for oversampling.
  // The KX134 supports ODR up to 25.6 kHz&#8203;:contentReference[oaicite:10]{index=10}. We use code 14 (~12.8 kHz)
  // to approximate 16kHz sampling (the sensor does not have an exact 8kHz or 16kHz setting).
  // This oversampling helps capture high-frequency content and will be decimated to ~8kHz.
  accel.setOutputDataRate(14);  // ODR code 14 ≈ 12.8 kHz (KX134 datasheet)

  // Configure FIFO (sample buffer) in FIFO mode with 16-bit resolution&#8203;:contentReference[oaicite:11]{index=11}:
  accel.setBufferResolution();         // Use 16-bit (full) resolution in buffer
  accel.enableSampleBuffer();          // Enable the sample buffer (FIFO mode by default)
  accel.setBufferOperationMode(0x00);  // FIFO mode (accumulate samples, stop on full)

  // Set FIFO interrupt (watermark) threshold.
  // We'll get an interrupt when this many samples are in FIFO.
  const uint8_t fifoThreshold = 20;  // ~20 samples ≈ 2.5ms of data at 8kHz output
  accel.setBufferThreshold(fifoThreshold);

  // Enable interrupts: route FIFO threshold event to physical interrupt pin INT1&#8203;:contentReference[oaicite:12]{index=12}
  accel.enableBufferInt();                // enable buffer (watermark/full) interrupt generation
  accel.enablePhysInterrupt(true, 1);     // enable physical interrupt pin1 (INT1)&#8203;:contentReference[oaicite:13]{index=13}
  accel.routeHardwareInterrupt(0x40, 1);  // route buffer-full/threshold interrupt to INT1&#8203;:contentReference[oaicite:14]{index=14}

  // Clear any stale interrupts
  accel.clearInterrupt();

  // Activate the accelerometer with the new settings
  accel.enableAccel(true);

  // Configure the microcontroller interrupt pin for the accelerometer INT1
  pinMode(KX134_INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(KX134_INT_PIN), onAccelDataReady, FALLING);
  // (KX134 INT is active low by default. If open-drain, use INPUT_PULLUP, else use INPUT if push-pull)

  /////////////////
  // BLE Peripheral Setup
  /////////////////
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);  // maximize BLE bandwidth allocation&#8203;:contentReference[oaicite:15]{index=15}
  Bluefruit.begin();
  Bluefruit.setTxPower(4);                   // Increase TX power (4 dBm) for stability
  Bluefruit.setName("XiaoBLE-AccelStream");  // Set BLE device name (optional)
  Bluefruit.Periph.setConnectCallback(onBLEConnect);
  Bluefruit.Periph.setDisconnectCallback(onBLEDisconnect);
  Bluefruit.Periph.setConnInterval(6, 12);  // Preferred connection interval: 7.5ms to 15ms&#8203;:contentReference[oaicite:16]{index=16}

  // Define a custom service and characteristic for accelerometer data
  accelService.begin();
  accelChar.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ);
  accelChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);  // not readable or writable by central, open for notify
  accelChar.setMaxLen(244);                                  // allow up to 244 bytes in one notification (att MTU 247)&#8203;:contentReference[oaicite:17]{index=17}
  accelChar.setFixedLen(0);                                  // variable length packets (we will send multiples of 6 bytes)
  accelChar.begin();
  // Note: CCCD (Client Characteristic Config Descriptor) is auto-created for notify property.

  // Set up advertising
  Bluefruit.Advertising.clearData();
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addService(accelService);  // advertise our service UUID
  Bluefruit.ScanResponse.addName();                // send device name in scan response
  Bluefruit.Advertising.setInterval(32, 244);      // advertising interval (20 ms to 152.5 ms)
  Bluefruit.Advertising.setFastTimeout(30);        // fast advertise for 30 seconds
  Bluefruit.Advertising.start(0);                  // start advertising indefinitely
  Serial.println("BLE advertising started.");
}

void loop() {
  // Check if the accelerometer FIFO has signaled new data ready
  if (fifoDataReady) {
    fifoDataReady = false;  // reset flag

    // Read all samples currently in the FIFO buffer
    uint8_t packetBuf[244];  // buffer for one BLE packet of data (up to 244 bytes)
    uint16_t bufLen = 0;
    static uint8_t skipCount = 0;  // decimation counter for ~8kHz output

    // Query how many samples are in the FIFO
    int samplesAvailable = accel.getSampleLevel();  // number of samples in buffer
    while (samplesAvailable > 0) {
      // Read one raw sample from FIFO (16-bit each axis)
      rawOutputData raw;
      bool gotData = accel.getRawAccelBufferData(&raw, 1);  // pop one sample (16-bit x,y,z)
      if (!gotData) break;                                  // if read failed (shouldn't normally happen)

      // Decimate by skipping 1 out of 3 samples to reduce to ~8kHz from ~12.8kHz
      if (skipCount == 2) {
        // Skip this sample (do not send)
        skipCount = 0;
      } else {
        // Keep this sample - copy X, Y, Z raw readings (16-bit each) into packet buffer
        int16_t x = raw.xData;
        int16_t y = raw.yData;
        int16_t z = raw.zData;
        // Little-endian byte order for each 16-bit value
        packetBuf[bufLen++] = (uint8_t)(x & 0xFF);
        packetBuf[bufLen++] = (uint8_t)((x >> 8) & 0xFF);
        packetBuf[bufLen++] = (uint8_t)(y & 0xFF);
        packetBuf[bufLen++] = (uint8_t)((y >> 8) & 0xFF);
        packetBuf[bufLen++] = (uint8_t)(z & 0xFF);
        packetBuf[bufLen++] = (uint8_t)((z >> 8) & 0xFF);
      }
      skipCount++;

      // If packet buffer is full, send notification
      if (bufLen >= 240) {  // 240 bytes is 40 samples; leave a few bytes for safety
        bool sent = accelChar.notify(packetBuf, bufLen);
        if (!sent) {
          Serial.println("BLE notify buffer full! Data dropped.");
        }
        bufLen = 0;
      }

      // Update sample count from FIFO for loop continuation
      samplesAvailable = accel.getSampleLevel();
    }

    // After reading all samples, send any remaining data in packetBuf
    if (bufLen > 0) {
      bool sent = accelChar.notify(packetBuf, bufLen);
      if (!sent) {
        Serial.println("BLE notify buffer full! Data dropped.");
      }
      bufLen = 0;
    }

    // Clear the accelerometer interrupt latch to allow next interrupt&#8203;:contentReference[oaicite:18]{index=18}&#8203;:contentReference[oaicite:19]{index=19}
    accel.clearInterrupt();
  }

  // (Optional) other application tasks could run here

  // Small delay or yield to allow BLE events and lower CPU usage
  delay(1);
}
