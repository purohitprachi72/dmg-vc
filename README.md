Per Notify Packet: 30 sample pairs (240 bytes).

Notification Rate: ~267 notifications per second (if effective rate is 8 kHz).

Overall Throughput: ~64 kB/s.

// Uncomment the next line to enable averaging mode (optional)
// #define USE_AVERAGING 1

// Increase FIFO threshold to experiment with larger transfers.
const uint8_t fifoThreshold = 40;  // experiment with a higher threshold
// In setup(), replace your original FIFO threshold setting:
accel.setBufferThreshold(fifoThreshold);

// --- Double Buffering Setup for BLE Notifications ---
#define BLE_PACKET_SIZE 244

uint8_t bleBufferA[BLE_PACKET_SIZE];
uint8_t bleBufferB[BLE_PACKET_SIZE];
// Pointer to the current buffer used for filling sample data.
uint8_t* currentBleBuffer = bleBufferA;
uint16_t currentBleBufferIndex = 0;

// Optionally, you can also maintain a flag to track if a BLE transfer is in progress
// (depending on how non-blocking your notify call is).

// --- In your loop() function ---
void loop() {
  if (fifoDataReady) {
    fifoDataReady = false;  // reset flag

    int samplesAvailable = accel.getSampleLevel();
    static uint8_t skipCount = 0;  // used for decimation

    // Optional accumulators for averaging mode
#ifdef USE_AVERAGING
    static int32_t accX = 0, accY = 0, accZ = 0;
    static uint8_t avgCount = 0;
#endif

    while (samplesAvailable > 0) {
      rawOutputData raw;
      if (!accel.getRawAccelBufferData(&raw, 1)) break;

#ifdef USE_AVERAGING
      // Accumulate samples for averaging.
      accX += raw.xData;
      accY += raw.yData;
      accZ += raw.zData;
      avgCount++;
      // When enough samples are accumulated, compute the average.
      // Here, we average every 3 samples while keeping ~8kHz output.
      if (avgCount == 3) {
        int16_t avgX = accX / 3;
        int16_t avgY = accY / 3;
        int16_t avgZ = accZ / 3;
        // Reset accumulators
        accX = accY = accZ = 0;
        avgCount = 0;

        // Append averaged sample (6 bytes) to current BLE buffer.
        if (currentBleBufferIndex + 6 > BLE_PACKET_SIZE) {
          // Send full packet
          bool sent = accelChar.notify(currentBleBuffer, currentBleBufferIndex);
          if (!sent) {
            Serial.println("BLE notify buffer full! Data dropped.");
          }
          // Swap buffers
          currentBleBuffer = (currentBleBuffer == bleBufferA) ? bleBufferB : bleBufferA;
          currentBleBufferIndex = 0;
        }
        currentBleBuffer[currentBleBufferIndex++] = (uint8_t)(avgX & 0xFF);
        currentBleBuffer[currentBleBufferIndex++] = (uint8_t)((avgX >> 8) & 0xFF);
        currentBleBuffer[currentBleBufferIndex++] = (uint8_t)(avgY & 0xFF);
        currentBleBuffer[currentBleBufferIndex++] = (uint8_t)((avgY >> 8) & 0xFF);
        currentBleBuffer[currentBleBufferIndex++] = (uint8_t)(avgZ & 0xFF);
        currentBleBuffer[currentBleBufferIndex++] = (uint8_t)((avgZ >> 8) & 0xFF);
      }
#else
      // --- Decimation Mode (Default) ---
      // Skip 1 out of every 3 samples to reduce from ~12.8kHz to ~8kHz
      if (skipCount == 2) {
        skipCount = 0; // Skip this sample.
      } else {
        int16_t x = raw.xData;
        int16_t y = raw.yData;
        int16_t z = raw.zData;
        // Check if there's room for 6 more bytes in the current BLE buffer.
        if (currentBleBufferIndex + 6 > BLE_PACKET_SIZE) {
          // Send the current buffer via BLE.
          bool sent = accelChar.notify(currentBleBuffer, currentBleBufferIndex);
          if (!sent) {
            Serial.println("BLE notify buffer full! Data dropped.");
          }
          // Swap buffers.
          currentBleBuffer = (currentBleBuffer == bleBufferA) ? bleBufferB : bleBufferA;
          currentBleBufferIndex = 0;
        }
        // Append X, Y, Z in little-endian format.
        currentBleBuffer[currentBleBufferIndex++] = (uint8_t)(x & 0xFF);
        currentBleBuffer[currentBleBufferIndex++] = (uint8_t)((x >> 8) & 0xFF);
        currentBleBuffer[currentBleBufferIndex++] = (uint8_t)(y & 0xFF);
        currentBleBuffer[currentBleBufferIndex++] = (uint8_t)((y >> 8) & 0xFF);
        currentBleBuffer[currentBleBufferIndex++] = (uint8_t)(z & 0xFF);
        currentBleBuffer[currentBleBufferIndex++] = (uint8_t)((z >> 8) & 0xFF);
      }
      skipCount++;
#endif

      // Update available sample count.
      samplesAvailable = accel.getSampleLevel();
    }

    // If any data remains in the current buffer, send it.
    if (currentBleBufferIndex > 0) {
      bool sent = accelChar.notify(currentBleBuffer, currentBleBufferIndex);
      if (!sent) {
        Serial.println("BLE notify buffer full! Data dropped.");
      }
      currentBleBufferIndex = 0;
    }

    // Clear the interrupt latch.
    accel.clearInterrupt();
  }

  // Replace the fixed delay with yield() if available to allow BLE stack processing.
  delay(1);
}
