// Reading gyro rates (deg/s) from the IMU, with the bias removed.
// datasheet (ICM-42688-P, DS-000347): https://invensense.tdk.com/en-us/download-resource/ds-000347-icm-42688-p-datasheet
// register addresses and the 16.4 sensitivity below all come from it

#include <SPI.h>
const int CS = 10;
const float LSB_PER_DPS = 16.4; // the gyro gives raw counts. at the default +-2000 deg/s range, 16.4 counts = 1 deg/s
float biasX = 0, biasY = 0, biasZ = 0; // what the gyro reads when sitting still. we measure it in setup and subtract it later
uint8_t b[6]; // 6 bytes from the imu: x high, x low, y high, y low, z high, z low

void setup() {

  Serial.begin(115200);
  delay(1000); // set baud rate, delay to add padding for connection sanity
  SPI.begin(12, 13, 11, CS); // config wire setup (sck, miso, mosi, cs)
  pinMode(CS, OUTPUT);
  digitalWrite(CS, HIGH); // high on cs wire means imu is not listening

  // turn the gyro on. it starts off by default
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3)); // same rules as whoami (1mhz clock, msb first, mode 3)
  digitalWrite(CS, LOW); // start convo w imu
  SPI.transfer(0x4E); // register 0x4E is PWR_MGMT0. top bit is 0, so this is a write
  SPI.transfer(0x0C); // 0x0C = gyro on in low noise mode, accel off
  digitalWrite(CS, HIGH); // end convo w imu
  SPI.endTransaction();
  delay(100); // gyro needs time to spin up before readings are good

  // measure the bias: average 500 readings while the board is held still (about 1 second)
  Serial.println("calibrating, keep the board still...");
  float sumX = 0, sumY = 0, sumZ = 0;
  for (int i = 0; i < 500; i++) {
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
    digitalWrite(CS, LOW);
    SPI.transfer(0x25 | 0x80); // read starting at 0x25 (GYRO_DATA_X1). the imu keeps going to the next register for every extra byte we clock
    for (int j = 0; j < 6; j++) b[j] = SPI.transfer(0x00); // 6 blank bytes to clock out x, y, z (2 bytes each)
    digitalWrite(CS, HIGH);
    SPI.endTransaction();
    sumX += (int16_t)((b[0] << 8) | b[1]) / LSB_PER_DPS; // glue high + low byte into one signed number, then convert to deg/s
    sumY += (int16_t)((b[2] << 8) | b[3]) / LSB_PER_DPS;
    sumZ += (int16_t)((b[4] << 8) | b[5]) / LSB_PER_DPS;
    delay(2);
  }
  biasX = sumX / 500; // average = the reading when nothing is moving
  biasY = sumY / 500;
  biasZ = sumZ / 500;
  Serial.printf("bias: %.3f %.3f %.3f deg/s\n", biasX, biasY, biasZ);
}

void loop() {
  // same read as in calibration
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
  digitalWrite(CS, LOW);
  SPI.transfer(0x25 | 0x80); // read starting at GYRO_DATA_X1
  for (int j = 0; j < 6; j++) b[j] = SPI.transfer(0x00);
  digitalWrite(CS, HIGH);
  SPI.endTransaction();

  float gx = (int16_t)((b[0] << 8) | b[1]) / LSB_PER_DPS - biasX; // deg/s around x, bias removed so still = ~0
  float gy = (int16_t)((b[2] << 8) | b[3]) / LSB_PER_DPS - biasY;
  float gz = (int16_t)((b[4] << 8) | b[5]) / LSB_PER_DPS - biasZ;

  Serial.printf("gx:%.2f gy:%.2f gz:%.2f\n", gx, gy, gz);
  delay(10); // about 100 readings a second, slow enough to print every one
}
