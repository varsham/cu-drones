// Turning gyro readings into angles and sending them to the laptop, so cube_viewer.py can spin a cube.
//
// this is gyro_rtos with 3 things added:
//   1) bias calibration in setup (same as bringup/gyro)
//   2) gyroTask now parses the bytes itself and adds up the angles every 1 ms
//   3) printTask sends "roll,pitch,yaw" instead of rates, 100 times a second
//
// adding up gyro rates gives angles, but small errors pile up, so the cube will slowly drift.
// that's expected. fixing it needs the accelerometer (that's the next step).

#include <SPI.h>
const int CS = 10;
const float LSB_PER_DPS = 16.4; // at the default +-2000 deg/s range, 16.4 raw counts = 1 deg/s
const float DT = 0.001; // seconds between reads. gyroTask wakes every 1 ms
float biasX = 0, biasY = 0, biasZ = 0; // what the gyro reads sitting still, measured in setup
QueueHandle_t q; // mailbox between the two tasks. this time it carries 3 floats: roll, pitch, yaw

void gyroTask(void *param) {
  uint8_t b[6]; // raw bytes from one read: x high, x low, y high, y low, z high, z low
  float roll = 0, pitch = 0, yaw = 0; // degrees. 0,0,0 = however the board was sitting when it powered on
  int count = 0;
  TickType_t wake = xTaskGetTickCount(); // 1 tick = 1 ms on the esp32

  while (true) {
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(1)); // sleep until exactly 1 ms after the last wake up

    // read: grab the 6 gyro bytes
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3)); // 1mhz clock, msb first, mode 3
    digitalWrite(CS, LOW); // start convo w imu
    SPI.transfer(0x25 | 0x80); // read (0x80) starting at 0x25 (GYRO_DATA_X1)
    for (int i = 0; i < 6; i++) b[i] = SPI.transfer(0x00); // 6 blank bytes to clock out x, y, z
    digitalWrite(CS, HIGH); // end convo w imu
    SPI.endTransaction();

    // parse: bytes -> deg/s, bias removed. this moved here from printTask because we need the numbers every 1 ms now
    float gx = (int16_t)((b[0] << 8) | b[1]) / LSB_PER_DPS - biasX;
    float gy = (int16_t)((b[2] << 8) | b[3]) / LSB_PER_DPS - biasY;
    float gz = (int16_t)((b[4] << 8) | b[5]) / LSB_PER_DPS - biasZ;

    // integrate: rate (deg/s) * time (s) = how many degrees we turned since the last read. add it on
    roll += gx * DT;  // rotation around x
    pitch += gy * DT; // rotation around y
    yaw += gz * DT;   // rotation around z
    // note: treating the 3 axes separately like this is only accurate for small or one-axis-at-a-time rotations.
    // tilt a lot on two axes at once and the cube will start to disagree with the board. fine for a first look

    count++;
    if (count == 10) { // every 10th read = 100 times a second, smooth enough for the cube
      float angles[3] = {roll, pitch, yaw};
      xQueueOverwrite(q, angles); // drop the latest angles in the mailbox, replacing any unread ones
      count = 0;
    }
  }
}

void printTask(void *param) {
  float angles[3];

  while (true) {
    xQueueReceive(q, angles, portMAX_DELAY); // sleep until angles show up

    // print: one line per update, comma separated, so the laptop script can split it easily
    Serial.printf("%.2f,%.2f,%.2f\n", angles[0], angles[1], angles[2]);
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

  // measure the bias: average 500 readings while the board is held still (about 1 second)
  // without this, the cube spins on its own even when the board is sitting on the table
  Serial.println("calibrating, keep the board still...");
  float sumX = 0, sumY = 0, sumZ = 0;
  uint8_t b[6];
  for (int i = 0; i < 500; i++) {
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
    digitalWrite(CS, LOW);
    SPI.transfer(0x25 | 0x80); // same read as in gyroTask
    for (int j = 0; j < 6; j++) b[j] = SPI.transfer(0x00);
    digitalWrite(CS, HIGH);
    SPI.endTransaction();
    sumX += (int16_t)((b[0] << 8) | b[1]) / LSB_PER_DPS;
    sumY += (int16_t)((b[2] << 8) | b[3]) / LSB_PER_DPS;
    sumZ += (int16_t)((b[4] << 8) | b[5]) / LSB_PER_DPS;
    delay(2);
  }
  biasX = sumX / 500;
  biasY = sumY / 500;
  biasZ = sumZ / 500;
  Serial.printf("bias: %.3f %.3f %.3f deg/s\n", biasX, biasY, biasZ); // the viewer skips this line since it's not 3 numbers

  q = xQueueCreate(1, sizeof(float) * 3); // mailbox with room for 1 set of 3 floats

  // start the tasks: (function, name, stack size in bytes, param, priority, handle, core)
  xTaskCreatePinnedToCore(printTask, "print", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(gyroTask, "gyro", 4096, NULL, 4, NULL, 1);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000)); // loop() is a task too. nothing to do here, so sleep
}
