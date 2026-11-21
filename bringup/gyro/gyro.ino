 // Read gyro rates (deg/s) from the ICM-42688-P over SPI.

  #include <SPI.h>

  const int CS = 10;
  const float LSB_PER_DPS = 16.4f;  // sensitivity at the default ±2000 dps range

  SPISettings imuSPI(1000000, MSBFIRST, SPI_MODE3);

  float biasX = 0, biasY = 0, biasZ = 0;  // deg/s, measured at startup

  void writeReg(uint8_t reg, uint8_t val) {
    SPI.beginTransaction(imuSPI);
    digitalWrite(CS, LOW);
    SPI.transfer(reg);          // top bit 0 = write
    SPI.transfer(val);
    digitalWrite(CS, HIGH);
    SPI.endTransaction();
  }

  void readRegs(uint8_t reg, uint8_t *buf, size_t n) {
    SPI.beginTransaction(imuSPI);
    digitalWrite(CS, LOW);
    SPI.transfer(reg | 0x80);   // top bit 1 = read
    for (size_t i = 0; i < n; i++) buf[i] = SPI.transfer(0x00);
    digitalWrite(CS, HIGH);
    SPI.endTransaction();
  }

  // Gyro rates in deg/s, before bias removal.
  void readGyro(float &x, float &y, float &z) {
    uint8_t b[6];
    readRegs(0x25, b, 6);       // GYRO_DATA_X1 .. GYRO_DATA_Z0, high byte first
    x = (int16_t)((b[0] << 8) | b[1]) / LSB_PER_DPS;
    y = (int16_t)((b[2] << 8) | b[3]) / LSB_PER_DPS;
    z = (int16_t)((b[4] << 8) | b[5]) / LSB_PER_DPS;
  }

  void setup() {
    Serial.begin(115200); delay(1000);
    SPI.begin(12, 13, 11, CS);  // SCK, MISO, MOSI, CS
    pinMode(CS, OUTPUT); digitalWrite(CS, HIGH);

    uint8_t id;
    readRegs(0x75, &id, 1);     // WHO_AM_I, expect 0x47
    Serial.printf("WHO_AM_I = 0x%02X\n", id);

    writeReg(0x4E, 0x0C);       // PWR_MGMT0: gyro on, low-noise mode
    delay(100);                 // gyro needs time to start up

    // Bias: average 500 samples while the board is held still (about 1 s).
    Serial.println("Calibrating, keep the board still...");
    const int N = 500;
    float sx = 0, sy = 0, sz = 0;
    for (int i = 0; i < N; i++) {
      float x, y, z;
      readGyro(x, y, z);
      sx += x; sy += y; sz += z;
      delay(2);
    }
    biasX = sx / N; biasY = sy / N; biasZ = sz / N;
    Serial.printf("bias: %.3f %.3f %.3f deg/s\n", biasX, biasY, biasZ);
  }

  void loop() {
    static uint32_t last = micros();
    uint32_t now = micros();
    float dt = (now - last) * 1e-6f;  // seconds since the previous read
    last = now;

    float gx, gy, gz;
    readGyro(gx, gy, gz);
    gx -= biasX; gy -= biasY; gz -= biasZ;

    Serial.printf("gx:%.2f gy:%.2f gz:%.2f dt:%.4f\n", gx, gy, gz, dt);
    delay(10);                  // about 100 Hz, slow enough to print every sample
  }