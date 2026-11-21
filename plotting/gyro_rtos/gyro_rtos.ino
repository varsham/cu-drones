// Read the gyro at 1 kHz in a FreeRTOS task, print it from another task.
//
// Three jobs, one function each:
//   readTick()   grab one tick's raw bytes from the IMU into a buffer
//   parseTick()  turn that buffer into x, y, z in deg/s
//   printGyro()  write x, y, z to serial
//
// Two tasks use them:
//   gyroTask  (core 1, high priority): readTick() every 1 ms, hands every 100th buffer to printTask
//   printTask (core 0, low priority):  parseTick() + printGyro()
// The fast task only moves bytes. All the slow work (math, Serial) happens on the other core.

#include <SPI.h>

const int CS = 10;
const float LSB_PER_DPS = 16.4f;  // sensitivity at the default ±2000 dps range
const int TICK_BYTES = 6;         // gyro x, y, z: 2 bytes each

SPISettings imuSPI(1000000, MSBFIRST, SPI_MODE3);

QueueHandle_t q;                  // FreeRTOS queue: a safe mailbox for passing data between tasks

// ---------- SPI: talk to the IMU ----------

void writeReg(uint8_t reg, uint8_t val) {
  SPI.beginTransaction(imuSPI);
  digitalWrite(CS, LOW);
  SPI.transfer(reg);              // top bit 0 = write
  SPI.transfer(val);
  digitalWrite(CS, HIGH);
  SPI.endTransaction();
}

void readRegs(uint8_t reg, uint8_t *buf, size_t n) {
  SPI.beginTransaction(imuSPI);
  digitalWrite(CS, LOW);
  SPI.transfer(reg | 0x80);       // top bit 1 = read
  for (size_t i = 0; i < n; i++) buf[i] = SPI.transfer(0x00);
  digitalWrite(CS, HIGH);
  SPI.endTransaction();
}

// ---------- the three jobs ----------

// Read: everything the IMU gives us this tick, as raw bytes.
void readTick(uint8_t *buf) {
  readRegs(0x25, buf, TICK_BYTES);  // GYRO_DATA_X1 .. GYRO_DATA_Z0, high byte first
}

// Parse: raw bytes -> deg/s. Each axis is a signed 16-bit number split over two bytes.
void parseTick(const uint8_t *buf, float &x, float &y, float &z) {
  x = (int16_t)((buf[0] << 8) | buf[1]) / LSB_PER_DPS;
  y = (int16_t)((buf[2] << 8) | buf[3]) / LSB_PER_DPS;
  z = (int16_t)((buf[4] << 8) | buf[5]) / LSB_PER_DPS;
}

// Print: one line per reading.
void printGyro(float x, float y, float z) {
  Serial.printf("gx:%.2f gy:%.2f gz:%.2f\n", x, y, z);
}

// ---------- tasks (a task is just a function that loops forever) ----------

void gyroTask(void *param) {
  TickType_t wake = xTaskGetTickCount();
  uint8_t buf[TICK_BYTES];
  int count = 0;

  while (true) {
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(1));  // sleep until 1 ms after the last wake-up (no drift, unlike delay)
    readTick(buf);

    count++;
    if (count == 100) {           // every 100th tick = 10 prints per second
      xQueueOverwrite(q, buf);    // copy the buffer into the mailbox (replaces any unread one, never waits)
      count = 0;
    }
  }
}

void printTask(void *param) {
  uint8_t buf[TICK_BYTES];
  float x, y, z;

  while (true) {
    xQueueReceive(q, buf, portMAX_DELAY);  // sleep until a buffer arrives
    parseTick(buf, x, y, z);
    printGyro(x, y, z);
  }
}

// ---------- startup ----------

void setup() {
  Serial.begin(115200); delay(1000);
  SPI.begin(12, 13, 11, CS);      // SCK, MISO, MOSI, CS
  pinMode(CS, OUTPUT); digitalWrite(CS, HIGH);

  uint8_t id;
  readRegs(0x75, &id, 1);         // WHO_AM_I, expect 0x47
  Serial.printf("WHO_AM_I = 0x%02X\n", id);

  writeReg(0x4E, 0x0C);           // PWR_MGMT0: gyro on, low-noise mode (defaults: ±2000 dps, 1 kHz)
  delay(100);                     // gyro needs time to start up

  q = xQueueCreate(1, TICK_BYTES);  // holds one tick's buffer

  // (function, name, stack bytes, param, priority, handle, core). Higher priority runs first.
  xTaskCreatePinnedToCore(printTask, "print", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(gyroTask,  "gyro",  4096, NULL, 4, NULL, 1);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));  // loop() is a task too; nothing to do, so sleep
}
