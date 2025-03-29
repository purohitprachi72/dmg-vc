#include <SPI.h>
#include <SparkFun_KX13X.h>     // SparkFun KX132/KX134 accelerometer library (defines rawOutputData)
#include <bluefruit.h>          // Adafruit Bluefruit BLE library
#include <Adafruit_TinyUSB.h>   // TinyUSB for USB CDC (if needed for Serial)
#include <Adafruit_SPIFlash.h>  // SPI Flash (required by Adafruit BSP)
#include <PDM.h>                // Arduino PDM library for the onboard microphone

// Flash transport (for internal flash usage, not modified here)
Adafruit_FlashTransport_QSPI flashTransport;

////////////////////
// Pin Definitions:
////////////////////
#define KX134_CS_PIN 1   // Chip select for KX134 (using SPI)
#define KX134_INT_PIN 2  // Interrupt pin from KX134

////////////////////
// BLE Service/Characteristic UUIDs:
////////////////////
#define ACCEL_SERVICE_UUID 0xFF00
#define ACCEL_CHAR_UUID    0xFF01

////////////////////
// Global Objects:
////////////////////
SparkFun_KX134_SPI accel;         // KX134 accelerometer object
BLEService accelService(ACCEL_SERVICE_UUID);
BLECharacteristic accelChar(ACCEL_CHAR_UUID);

// Flag for sensor interrupt (used to trigger sample aggregation)
volatile bool fifoDataReady = false;

// Global counters for decimation:
static uint32_t accelDecimCount = 0; // For accelerometer decimation
static uint32_t pdmDecimCount   = 0; // For PDM decimation

// Aggregation settings:
// We'll aggregate 30 sample pairs per BLE notification.
// Each sample pair is 8 bytes: 6 bytes from accel and 2 bytes from PDM.
const uint8_t SAMPLE_PAIRS_PER_PKT = 30;
const uint16_t PACKET_SIZE = SAMPLE_PAIRS_PER_PKT * 8; // 240 bytes

// Buffer for aggregated notification packet
uint8_t aggPacket[PACKET_SIZE];

// Dummy simulation trigger timing (for accelerometer):
// In a real system, the accelerometer ISR (onAccelDataReady) would set fifoDataReady.
// For simulation we can also use a timer.
static unsigned long lastSimTime = 0;

////////////////////
// Accelerometer FIFO "ISR" (simulation)
// In real hardware, this is triggered by the sensor interrupt.
////////////////////
void onAccelDataReady() {
  fifoDataReady = true;
}

////////////////////
// BLE Connection Callbacks
////////////////////
void onBLEConnect(uint16_t conn_hdl) {
  BLEConnection* conn = Bluefruit.Connection(conn_hdl);
  conn->requestPHY();
  conn->requestMtuExchange(247);
  conn->requestConnectionParameter(6, 8);  // 6 * 1.25ms = 7.5ms interval
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

  // Enable DC-DC converter for power efficiency
  NRF_POWER->DCDCEN = 1;

  // Flash power-down mode
  flashTransport.begin();
  flashTransport.runCommand(0xB9);
  delayMicroseconds(5);
  flashTransport.end();

  /////////////////
  // KX134 Sensor Setup
  /////////////////
  pinMode(KX134_CS_PIN, OUTPUT);
  digitalWrite(KX134_CS_PIN, HIGH);
  SPISettings settings(8000000, MSBFIRST, SPI_MODE0);
  SPI.begin();
  if (!accel.begin(SPI, settings, KX134_CS_PIN)) {
    Serial.println("KX134 not detected. Check wiring!");
    while (1) delay(10);
  }
  Serial.println("KX134 initialized.");

  // Reset and configure sensor
  accel.softwareReset();
  delay(5);
  accel.enableAccel(false);  // Enter standby for configuration
  accel.setRange(SFE_KX134_RANGE64G);
  accel.enableDataEngine();
  accel.setOutputDataRate(14);  // ODR code 14 ≈ 12.8 kHz (oversampled)
  accel.setBufferResolution();  // 16-bit resolution in FIFO
  accel.enableSampleBuffer();   // Enable FIFO mode
  accel.setBufferOperationMode(0x00);
  const uint8_t fifoThreshold = 20;  // e.g. threshold for triggering interrupt
  accel.setBufferThreshold(fifoThreshold);
  accel.enableBufferInt();
  accel.enablePhysInterrupt(true, 1);
  accel.routeHardwareInterrupt(0x40, 1);
  accel.clearInterrupt();
  accel.enableAccel(true);

  // Set up interrupt pin for accelerometer
  pinMode(KX134_INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(KX134_INT_PIN), onAccelDataReady, FALLING);

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
  Serial.println("BLE advertising started.");

  /////////////////
  // PDM Microphone Setup
  /////////////////
  if (!PDM.begin(1, 16000)) {  // 1 channel at 16 kHz
    Serial.println("Failed to start PDM!");
  } else {
    Serial.println("PDM started.");
  }
  PDM.setBufferSize(256);
  // We use a simple onReceive callback that does not process data immediately.
  // We'll read and decimate PDM data in loop.
  PDM.onReceive([](){
    // Just flag that new PDM data is available.
    // (We won't use a separate flag here since we'll decimate on-demand.)
  });
}

////////////////////
// Helper Functions for Decimation
////////////////////

// Function to get one decimated accelerometer sample (one out of 2 samples)
bool getDecimatedAccelSample(rawOutputData &sample) {
  // Check if there are samples in FIFO
  int samplesAvailable = accel.getSampleLevel();
  if (samplesAvailable > 0) {
    // Read one sample
    if (accel.getRawAccelBufferData(&sample, 1)) {
      accelDecimCount++;
      // Use the sample only if we are on an even count (decimation factor = 2)
      if ((accelDecimCount % 2) == 0) {
        return true;
      }
    }
  }
  return false;
}

// Function to get one decimated PDM sample (16-bit) from the PDM buffer
// Returns true if a sample was obtained, false otherwise.
bool getDecimatedPDMSample(int16_t &pdmSample) {
  // Check if at least 2 bytes (one 16-bit sample) are available
  int bytesAvailable = PDM.available();
  if (bytesAvailable >= 2) {
    // Allocate a temporary buffer to read available data
    uint8_t tempBuffer[256];
    int bytesRead = PDM.read(tempBuffer, bytesAvailable);
    // Process the buffer: take one 16-bit sample every 2 samples.
    // For simplicity, we take the first sample and then discard one.
    // (A more robust implementation would maintain state across calls.)
    pdmDecimCount++;
    // Convert first two bytes to a 16-bit value (little-endian)
    int16_t sample = (int16_t)((tempBuffer[1] << 8) | tempBuffer[0]);
    // Only use this sample if decimation condition is met
    if ((pdmDecimCount % 2) == 0) {
      pdmSample = sample;
      return true;
    }
  }
  return false;
}

////////////////////
// Loop Function: Aggregates 30 decimated sample pairs and sends them via BLE notification.
////////////////////
void loop() {
  // For simulation purposes, if fifoDataReady is not set by real interrupt,
  // we can set it periodically. Uncomment the following block to simulate:
  /*
  unsigned long now = micros();
  if (now - lastSimTime >= 125) {  // 125 microseconds for 8 kHz (approx.)
    fifoDataReady = true;
    lastSimTime = now;
  }
  */

  static uint8_t samplePairCount = 0;  // Count of sample pairs accumulated in aggPacket
  uint16_t offset = samplePairCount * 8; // Each sample pair is 8 bytes

  // Try to get a decimated accelerometer sample:
  rawOutputData accelSample;
  bool gotAccel = getDecimatedAccelSample(accelSample);

  // Try to get a decimated PDM sample:
  int16_t pdmSample;
  bool gotPDM = getDecimatedPDMSample(pdmSample);

  // Only if both samples are available, pack them into the aggregated packet.
  if (gotAccel && gotPDM) {
    // Pack accelerometer sample (6 bytes)
    aggPacket[offset++] = (uint8_t)(accelSample.xData & 0xFF);
    aggPacket[offset++] = (uint8_t)((accelSample.xData >> 8) & 0xFF);
    aggPacket[offset++] = (uint8_t)(accelSample.yData & 0xFF);
    aggPacket[offset++] = (uint8_t)((accelSample.yData >> 8) & 0xFF);
    aggPacket[offset++] = (uint8_t)(accelSample.zData & 0xFF);
    aggPacket[offset++] = (uint8_t)((accelSample.zData >> 8) & 0xFF);

    // Pack PDM sample (2 bytes)
    aggPacket[offset++] = (uint8_t)(pdmSample & 0xFF);
    aggPacket[offset++] = (uint8_t)((pdmSample >> 8) & 0xFF);

    samplePairCount++;

    // If we have accumulated the desired number of sample pairs, send the notification.
    if (samplePairCount >= SAMPLE_PAIRS_PER_PKT) {
      bool sent = accelChar.notify(aggPacket, PACKET_SIZE);
      if (!sent) {
        Serial.println("BLE notify buffer full! Data dropped.");
      } else {
        Serial.print("Sent packet with ");
        Serial.print(SAMPLE_PAIRS_PER_PKT);
        Serial.println(" sample pairs.");
      }
      // Reset for next packet
      samplePairCount = 0;
    }
  }

  // Clear the accelerometer interrupt latch (if needed)
  accel.clearInterrupt();

  // Small delay to yield CPU time (adjust as needed)
  delay(1);
}
