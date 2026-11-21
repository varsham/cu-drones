// Reading the gyro 1000 times a second using FreeRTOS, and printing it from the other core.
//
// the esp32-s3 has 2 cores. freertos lets us run "tasks" (functions that loop forever) on each one.
// gyroTask runs on core 1: wakes up every 1 ms, reads the raw bytes, and every 100th read hands them over.
// printTask runs on core 0: waits for bytes, turns them into deg/s, prints them.
// printing is slow, so keeping it on the other core means it can never make a gyro read late.

#include <SPI.h>
const int CS = 10;
const float LSB_PER_DPS = 16.4; // at the default +-2000 deg/s range, 16.4 raw counts = 1 deg/s
QueueHandle_t q; // a freertos queue. it's a mailbox: one task drops bytes in, the other picks them up. safe across cores

void gyroTask(void *param) {
  uint8_t b[6]; // raw bytes from one read: x high, x low, y high, y low, z high, z low
  int count = 0;
  TickType_t wake = xTaskGetTickCount(); // freertos keeps a tick counter. 1 tick = 1 ms on the esp32

  while (true) {
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(1)); // sleep until exactly 1 ms after the last wake up. unlike delay(1), time spent reading doesn't add up and drift

    // read: grab the 6 gyro bytes
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3)); // 1mhz clock, msb first, mode 3
    digitalWrite(CS, LOW); // start convo w imu
    SPI.transfer(0x25 | 0x80); // read (0x80) starting at 0x25 (GYRO_DATA_X1). the imu moves to the next register each byte
    for (int i = 0; i < 6; i++) b[i] = SPI.transfer(0x00); // 6 blank bytes to clock out x, y, z
    digitalWrite(CS, HIGH); // end convo w imu
    SPI.endTransaction();

    count++;
    if (count == 100) { // every 100th read = 10 times a second, so the screen isn't a blur
      xQueueOverwrite(q, b); // copy the bytes into the mailbox. if the old ones weren't picked up yet, replace them. never waits
      count = 0;
    }
  }
}

void printTask(void *param) {
  uint8_t b[6];

  while (true) {
    xQueueReceive(q, b, portMAX_DELAY); // sleep here (using no cpu) until bytes show up in the mailbox

    // parse: glue each high + low byte into one signed number, then convert to deg/s
    float gx = (int16_t)((b[0] << 8) | b[1]) / LSB_PER_DPS;
    float gy = (int16_t)((b[2] << 8) | b[3]) / LSB_PER_DPS;
    float gz = (int16_t)((b[4] << 8) | b[5]) / LSB_PER_DPS;

    // print
    Serial.printf("gx:%.2f gy:%.2f gz:%.2f\n", gx, gy, gz);
  }
}

void setup() {

  Serial.begin(115200);
  delay(1000); // set baud rate, delay to add padding for connection sanity
  SPI.begin(12, 13, 11, CS); // config wire setup (sck, miso, mosi, cs)
  pinMode(CS, OUTPUT);
  digitalWrite(CS, HIGH); // high on cs wire means imu is not listening

  // turn the gyro on. it starts off by default. default rate is already 1000 readings/s
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
  digitalWrite(CS, LOW);
  SPI.transfer(0x4E); // register 0x4E is PWR_MGMT0. top bit 0 = write
  SPI.transfer(0x0C); // 0x0C = gyro on in low noise mode, accel off
  digitalWrite(CS, HIGH);
  SPI.endTransaction();
  delay(100); // gyro needs time to spin up

  q = xQueueCreate(1, 6); // mailbox with room for 1 item of 6 bytes (xQueueOverwrite only works with room for 1)

  // start the tasks: (function, name, stack size in bytes, param, priority, handle, core)
  // higher priority number = runs first. gyroTask gets 4 so nothing on core 1 can delay it
  xTaskCreatePinnedToCore(printTask, "print", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(gyroTask, "gyro", 4096, NULL, 4, NULL, 1);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000)); // loop() is secretly a task too. nothing to do here, so just sleep
}
