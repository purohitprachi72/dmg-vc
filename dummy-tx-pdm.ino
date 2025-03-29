#include <SPI.h>
#include <SparkFun_KX13X.h>     // Defines rawOutputData structure (for dummy data format)
#include <bluefruit.h>          // Adafruit Bluefruit BLE library
#include <Adafruit_TinyUSB.h>   // USB CDC for Serial
#include <Adafruit_SPIFlash.h>  // SPI Flash support
#include <PDM.h>                // Arduino PDM library for onboard microphone

// Flash transport for internal flash operations
Adafruit_FlashTransport_QSPI flashTransport;

////////////////////
// Pin Definitions:
////////////////////
#define KX134_CS_PIN 1   // Not used because we're simulating accelerometer data
#define KX134_INT_PIN 2  // Not used in simulation

////////////////////
// BLE Service/Characteristic UUIDs:
////////////////////
#define ACCEL_SERVICE_UUID 0xFF00
#define ACCEL_CHAR_UUID    0xFF01

////////////////////
// Global Objects:
////////////////////
SparkFun_KX134_SPI accel;         // Accelerometer object (dummy data used)
BLEService accelService(ACCEL_SERVICE_UUID);
BLECharacteristic accelChar(ACCEL_CHAR_UUID);

// Flag for triggering dummy sensor sampling (simulate FIFO interrupt)
volatile bool fifoDataReady = false;

// Decimation counters – assume raw sampling at ~16 kHz and we use every 2nd sample.
static uint32_t accelDecimCount = 0;
static uint32_t pdmDecimCount   = 0;

// Aggregation settings:
// Aggregate 30 sample pairs per BLE notification.
// Each sample pair is 8 bytes: 6 bytes (dummy accel) + 2 bytes (PDM sample).
const uint8_t SAMPLE_PAIRS_PER_PKT = 30;
const uint16_t PACKET_SIZE = SAMPLE_PAIRS_PER_PKT * 8;  // 240 bytes

// Buffer for aggregated BLE notification packet
uint8_t aggPacket[PACKET_SIZE];

// For simulation timing (if no real sensor interrupt)
static unsigned long lastSimTime = 0;

// Dummy sample counter for accelerometer simulation
static uint32_t sampleCounter = 0;

////////////////////
// Simulated Accelerometer ISR
////////////////////
void onAccelDataReady() {
  fifoDataReady = true;
}

////////////////////
// Helper Function: Generate one dummy accelerometer sample (decimated)
// Format: X = sampleCounter, Y = sampleCounter+1, Z = sampleCounter+2.
bool getDummyAccelSample(rawOutputData &sample) {
  sample.xData = (int16_t)(sampleCounter & 0xFFFF);
  sample.yData = (int16_t)((sampleCounter + 1) & 0xFFFF);
  sample.zData = (int16_t)((sampleCounter + 2) & 0xFFFF);
  sampleCounter++;  // Increment counter for next sample

  accelDecimCount++;
  // Use the sample only if on an even call (decimation factor = 2)
  if ((accelDecimCount % 2) == 0) {
    return true;
  }
  return false;
}

////////////////////
// Helper Function: Get one decimated PDM sample (16-bit)
// Reads available PDM data and returns one sample (decimated by factor 2)
bool getDecimatedPDMSample(int16_t &pdmSample) {
  int bytesAvailable = PDM.available();
  if (bytesAvailable >= 2) {
    uint8_t tempBuffer[256];
    int bytesRead = PDM.read(tempBuffer, bytesAvailable);
    pdmDecimCount++;
    // For simplicity, take the first 16-bit sample (little-endian) from the buffer.
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
  Serial.begin(115200);
  while (!Serial) delay(10);

  // Enable DC-DC converter for efficiency
  NRF_POWER->DCDCEN = 1;

  // Flash power-down mode
  flashTransport.begin();
  flashTransport.runCommand(0xB9);
  delayMicroseconds(5);
  flashTransport.end();

  /////////////////
  // Sensor Initialization (Dummy Data)
  /////////////////
  // We skip actual sensor initialization since we’re simulating sensor data.
  // Uncomment the block below if you attach a real sensor.
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
  accel.setOutputDataRate(14);  // 12.8 kHz raw; we'll decimate to 8 kHz
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
  
  // For simulation, we trigger fifoDataReady in the loop via timer.

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
  accelChar.setMaxLen(244);  // Maximum payload per BLE notification
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
  // Use a simple onReceive callback; actual processing is done in loop.
  PDM.onReceive([](){
    // No immediate processing; we'll read data in loop.
  });
}

////////////////////
// Loop Function: Aggregate 30 sample pairs and send one BLE notification.
////////////////////
void loop() {
  // For simulation: if fifoDataReady is not set by an actual sensor, simulate it.
  unsigned long now = micros();
  if (now - lastSimTime >= 125) {  // 125 µs approximates an 8 kHz trigger
    fifoDataReady = true;
    lastSimTime = now;
  }
  
  static uint8_t samplePairCount = 0;  // Count of sample pairs aggregated
  uint16_t offset = samplePairCount * 8; // Each pair is 8 bytes
  
  // Try to get one decimated dummy accelerometer sample.
  rawOutputData dummyAccel;
  bool gotAccel = getDummyAccelSample(dummyAccel);
  
  // Try to get one decimated PDM sample (real microphone data).
  int16_t pdmSample;
  bool gotPDM = getDecimatedPDMSample(pdmSample);
  
  // Only if both samples are available, pack them as an 8-byte pair.
  if (gotAccel && gotPDM) {
    // Pack dummy accelerometer sample (6 bytes)
    aggPacket[offset++] = (uint8_t)(dummyAccel.xData & 0xFF);
    aggPacket[offset++] = (uint8_t)((dummyAccel.xData >> 8) & 0xFF);
    aggPacket[offset++] = (uint8_t)(dummyAccel.yData & 0xFF);
    aggPacket[offset++] = (uint8_t)((dummyAccel.yData >> 8) & 0xFF);
    aggPacket[offset++] = (uint8_t)(dummyAccel.zData & 0xFF);
    aggPacket[offset++] = (uint8_t)((dummyAccel.zData >> 8) & 0xFF);
    // Pack decimated PDM sample (2 bytes)
    aggPacket[offset++] = (uint8_t)(pdmSample & 0xFF);
    aggPacket[offset++] = (uint8_t)((pdmSample >> 8) & 0xFF);
    
    samplePairCount++;
    
    // When we have 30 sample pairs, send the aggregated packet.
    if (samplePairCount >= SAMPLE_PAIRS_PER_PKT) {
      bool sent = accelChar.notify(aggPacket, PACKET_SIZE);
      if (!sent) {
        Serial.println("BLE notify buffer full! Data dropped.");
      } else {
        Serial.print("Sent packet with ");
        Serial.print(SAMPLE_PAIRS_PER_PKT);
        Serial.println(" sample pairs.");
      }
      samplePairCount = 0;  // Reset for next aggregation.
    }
  }
  
  // Clear sensor interrupt latch (if applicable; here it's simulated)
  accel.clearInterrupt();
  
  // Short delay to yield CPU time.
  delay(1);
}
