#include <Arduino.h>
#include <SPI.h>
#include <esp_timer.h>
#include <math.h>

// =====================================================
// WHAT THIS SKETCH DOES (NO-FILTER VERSION)
// =====================================================
//
// Reads the ICM-42688-P gyro AND accelerometer 1000 times a second and
// prints TWO separate orientation estimates side by side, with NO
// filtering or blending at all:
//
//   1) ACCEL angles: roll and pitch worked out from which way gravity
//      points. They never drift, but they are noisy, jump around when
//      the motors vibrate, and are wrong while the drone accelerates.
//
//   2) GYRO angles: roll, pitch and yaw made by adding up the gyro's
//      rotation rate every millisecond (angle += rate * dt). They are
//      smooth and react instantly, but they slowly DRIFT away from the
//      truth because small errors pile up.
//
//   roll  = tilt left/right   (rotation around the X axis)
//   pitch = tilt nose up/down (rotation around the Y axis)
//   yaw   = heading           (rotation around the Z axis)
//
// Watching both lets you see each sensor's weakness on its own. The
// complementary filter version (freertos_attitude.ino) combines them
// so each covers the other's weakness.
//
// The accelerometer cannot measure yaw (gravity points down whichever
// way the nose faces), so yaw only comes from the gyro.
//
// The one correction still applied is the gyro bias calibration at
// startup. It is not a filter: it just subtracts the gyro's constant
// offset. Without it the gyro angles drift within seconds. To see
// that, set GYRO_CAL_SAMPLES to 0.
//
// The timing statistics from the original sketch are all still here.

// =====================================================
// CONFIGURATION
// =====================================================

// SPI pins
#define CS       10
#define SCK_PIN  12
#define MISO_PIN 13
#define MOSI_PIN 11

#define SPI_FREQUENCY 10000000

// Sampling configuration
#define SAMPLE_RATE_HZ 1000
#define SAMPLE_PERIOD_US 1000

// FreeRTOS configuration
#define IMU_CORE 1
#define REPORT_CORE 0

#define IMU_PRIORITY 4
#define REPORT_PRIORITY 1

#define IMU_STACK_SIZE 4096
#define REPORT_STACK_SIZE 4096

// Report every 100 samples (about 10 times per second).
// This was 1000 (once per second). 10 Hz makes it much easier to tilt
// the board and watch the angles follow. Set it back to 1000 if you
// prefer the old rate. Timing stats are computed over each interval.
#define REPORT_INTERVAL 100

#if (configTICK_RATE_HZ < 1000)
#error "FreeRTOS tick rate must be at least 1000 Hz for 1 ms scheduling"
#endif

// =====================================================
// CALIBRATION SETTINGS
// =====================================================

// How many samples to average at startup to measure the gyro's bias.
// 1000 samples at 1 ms each = about 1 second. KEEP THE BOARD STILL.
// Set to 0 to skip calibration and see the raw gyro drift.
#define GYRO_CAL_SAMPLES 1000

// Ignore a dt bigger than this (seconds). Protects the gyro angles if
// the task is ever paused for a long time (e.g. a debugger break).
#define MAX_DT_S 0.01f

#define RAD_TO_DEG_F 57.2957795f

// =====================================================
// ICM-42688-P REGISTERS (User Bank 0)
// =====================================================
// Addresses checked against the ICM-42688-P datasheet register map.

#define REG_DEVICE_CONFIG  0x11
#define REG_ACCEL_DATA_X1  0x1F   // first of 12 data bytes: accel X,Y,Z then gyro X,Y,Z
#define REG_GYRO_DATA_X1   0x25
#define REG_PWR_MGMT0      0x4E
#define REG_GYRO_CONFIG0   0x4F
#define REG_ACCEL_CONFIG0  0x50
#define REG_WHO_AM_I       0x75
#define REG_BANK_SEL       0x76

#define EXPECTED_WHO_AM_I  0x47

// Gyro sensitivity for +/-2000 degrees/second: 16.4 counts per dps
#define GYRO_SENSITIVITY 16.4f

// Accel sensitivity for +/-16 g: 2048 counts per g
#define ACCEL_SENSITIVITY 2048.0f

// =====================================================
// SPI CONFIGURATION
// =====================================================

SPISettings imuSPISettings(
    SPI_FREQUENCY,
    MSBFIRST,
    SPI_MODE0
);

// =====================================================
// DATA STRUCTURES
// =====================================================

// One reading from the IMU, already converted to real units
struct ImuData {
    float ax, ay, az;   // acceleration in g (1 g = gravity)
    float gx, gy, gz;   // rotation rate in degrees/second
};

// Orientation in degrees (here: built from the gyro alone)
struct Attitude {
    float roll;
    float pitch;
    float yaw;
};

struct TimingReport {
    uint32_t samples;

    // Tilt from the accelerometer alone (this instant)
    float accelRoll;
    float accelPitch;

    // Orientation from adding up the gyro alone
    float gyroRoll;
    float gyroPitch;
    float gyroYaw;

    // Latest bias-corrected gyro rates (handy for debugging)
    float gyroX;
    float gyroY;
    float gyroZ;

    // Total acceleration (should be close to 1.00 g when still)
    float accelMag;

    float avgPeriod;
    float minPeriod;
    float maxPeriod;

    float maxAbsJitter;
    float avgAbsJitter;
    float periodSD;

    uint32_t missedPeriods;
};

// Queue for sending statistics to reporting task
QueueHandle_t reportQueue = NULL;

// Gyro bias (the reading when the board is perfectly still).
// Measured once in setup(), then subtracted from every reading.
float gyroBiasX = 0.0f;
float gyroBiasY = 0.0f;
float gyroBiasZ = 0.0f;

// Starting orientation, measured in setup() from the accelerometer
Attitude initialAttitude = {0.0f, 0.0f, 0.0f};

// =====================================================
// SPI REGISTER FUNCTIONS
// =====================================================

void writeRegister(uint8_t reg, uint8_t value) {

    SPI.beginTransaction(imuSPISettings);

    digitalWrite(CS, LOW);

    SPI.transfer(reg & 0x7F);
    SPI.transfer(value);

    digitalWrite(CS, HIGH);

    SPI.endTransaction();
}

uint8_t readRegister(uint8_t reg) {

    SPI.beginTransaction(imuSPISettings);

    digitalWrite(CS, LOW);

    SPI.transfer(reg | 0x80);

    uint8_t value = SPI.transfer(0x00);

    digitalWrite(CS, HIGH);

    SPI.endTransaction();

    return value;
}

void readRegisters(
    uint8_t startReg,
    uint8_t *buffer,
    uint8_t length
) {

    SPI.beginTransaction(imuSPISettings);

    digitalWrite(CS, LOW);

    SPI.transfer(startReg | 0x80);

    for (uint8_t i = 0; i < length; i++) {
        buffer[i] = SPI.transfer(0x00);
    }

    digitalWrite(CS, HIGH);

    SPI.endTransaction();
}

// =====================================================
// IMU INITIALIZATION
// =====================================================

bool initializeIMU() {

    // Initialize SPI using specified ESP32-S3 pins
    SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS);

    pinMode(CS, OUTPUT);
    digitalWrite(CS, HIGH);

    delay(10);

    // Select user bank 0
    writeRegister(REG_BANK_SEL, 0x00);

    // Check device identity
    uint8_t whoAmI = readRegister(REG_WHO_AM_I);

    Serial.printf(
        "ICM-42688-P WHO_AM_I: 0x%02X\n",
        whoAmI
    );

    if (whoAmI != EXPECTED_WHO_AM_I) {
        Serial.println("ERROR: IMU not detected!");
        return false;
    }

    // Configure gyro:
    // GYRO_FS_SEL  (bits 7:5) = 000:  +/-2000 dps
    // GYRO_ODR     (bits 3:0) = 0110: 1 kHz
    writeRegister(REG_GYRO_CONFIG0, 0x06);

    // Configure accelerometer:
    // ACCEL_FS_SEL (bits 7:5) = 000:  +/-16 g
    // ACCEL_ODR    (bits 3:0) = 0110: 1 kHz
    writeRegister(REG_ACCEL_CONFIG0, 0x06);

    // Turn both sensors on in Low Noise mode:
    // GYRO_MODE  (bits 3:2) = 11: Low Noise
    // ACCEL_MODE (bits 1:0) = 11: Low Noise
    // (The old value 0x0C left the accelerometer off.)
    writeRegister(REG_PWR_MGMT0, 0x0F);

    // Allow sensor startup time (gyro needs about 45 ms)
    delay(50);

    Serial.println("Gyro + accelerometer configured.");
    Serial.println("ODR: 1000 Hz for both");
    Serial.println("Gyro range: +/-2000 dps, accel range: +/-16 g");

    return true;
}

// =====================================================
// READ ACCELEROMETER + GYROSCOPE
// =====================================================

// Turn a high byte and a low byte into one signed 16-bit number
static inline int16_t combineBytes(uint8_t high, uint8_t low) {
    return (int16_t)(((uint16_t)high << 8) | low);
}

bool readIMU(ImuData &imu) {

    uint8_t data[12];

    // The accel and gyro registers sit next to each other, so one
    // 12-byte read gets everything, all from the same instant:
    //   bytes 0-5:  accel X, Y, Z (high byte, low byte each)
    //   bytes 6-11: gyro  X, Y, Z
    readRegisters(
        REG_ACCEL_DATA_X1,
        data,
        12
    );

    int16_t rawAX = combineBytes(data[0],  data[1]);
    int16_t rawAY = combineBytes(data[2],  data[3]);
    int16_t rawAZ = combineBytes(data[4],  data[5]);

    int16_t rawGX = combineBytes(data[6],  data[7]);
    int16_t rawGY = combineBytes(data[8],  data[9]);
    int16_t rawGZ = combineBytes(data[10], data[11]);

    // While the sensor is still starting up it reports -32768 as
    // "no data yet". Skip those readings.
    if (rawAX == -32768 || rawGX == -32768) {
        return false;
    }

    // Convert raw counts to real units
    imu.ax = rawAX / ACCEL_SENSITIVITY;   // g
    imu.ay = rawAY / ACCEL_SENSITIVITY;
    imu.az = rawAZ / ACCEL_SENSITIVITY;

    imu.gx = rawGX / GYRO_SENSITIVITY;    // degrees/second
    imu.gy = rawGY / GYRO_SENSITIVITY;
    imu.gz = rawGZ / GYRO_SENSITIVITY;

    return true;
}

// =====================================================
// ORIENTATION MATH
// =====================================================

// Tilt angles from the accelerometer alone (valid when the drone is
// not accelerating). With the board flat and still, az reads about
// +1 g and both angles are about 0.
void accelToAngles(const ImuData &imu, float &accelRoll, float &accelPitch) {

    accelRoll  = atan2f(imu.ay, imu.az) * RAD_TO_DEG_F;

    accelPitch = atan2f(
        -imu.ax,
        sqrtf(imu.ay * imu.ay + imu.az * imu.az)
    ) * RAD_TO_DEG_F;
}

// Keep an angle in the range -180..+180 degrees
static inline float wrapAngle(float angle) {
    while (angle > 180.0f)   angle -= 360.0f;
    while (angle <= -180.0f) angle += 360.0f;
    return angle;
}

// Add up the gyro: angle += rate * time.
//   imu: latest reading (gyro already bias-corrected)
//   dt:  time since the previous step, in seconds
// Nothing corrects these angles, so they will slowly drift.
void integrateGyro(Attitude &att, const ImuData &imu, float dt) {
    att.roll  = wrapAngle(att.roll  + imu.gx * dt);
    att.pitch = wrapAngle(att.pitch + imu.gy * dt);
    att.yaw   = wrapAngle(att.yaw   + imu.gz * dt);
}

// =====================================================
// STARTUP CALIBRATION
// =====================================================

// Measures gyro bias and the starting tilt.
// The board MUST be still (ideally flat) while this runs.
bool calibrateIMU() {

    if (GYRO_CAL_SAMPLES == 0) {
        Serial.println("Calibration skipped: gyro bias NOT removed.");
        return true;
    }

    Serial.println("Calibrating... keep the board STILL.");

    // Let the sensor settle and throw away the first readings
    delay(100);

    double sumGX = 0.0, sumGY = 0.0, sumGZ = 0.0;
    double sumAX = 0.0, sumAY = 0.0, sumAZ = 0.0;
    uint32_t good = 0;

    ImuData imu;

    for (uint32_t i = 0; i < GYRO_CAL_SAMPLES; i++) {

        if (readIMU(imu)) {
            sumGX += imu.gx;
            sumGY += imu.gy;
            sumGZ += imu.gz;

            sumAX += imu.ax;
            sumAY += imu.ay;
            sumAZ += imu.az;

            good++;
        }

        delay(1);   // one new sample per millisecond
    }

    if (good < GYRO_CAL_SAMPLES / 2) {
        Serial.println("ERROR: too few valid samples during calibration!");
        return false;
    }

    // Average gyro reading while still = bias
    gyroBiasX = (float)(sumGX / good);
    gyroBiasY = (float)(sumGY / good);
    gyroBiasZ = (float)(sumGZ / good);

    // Average accel reading gives the starting tilt, so the gyro angles
    // start from the board's real tilt instead of from 0.
    ImuData avg;
    avg.ax = (float)(sumAX / good);
    avg.ay = (float)(sumAY / good);
    avg.az = (float)(sumAZ / good);

    accelToAngles(avg, initialAttitude.roll, initialAttitude.pitch);
    initialAttitude.yaw = 0.0f;   // "whatever way we face now" = 0

    float mag = sqrtf(avg.ax * avg.ax + avg.ay * avg.ay + avg.az * avg.az);

    Serial.printf(
        "Gyro bias: X:%.3f Y:%.3f Z:%.3f dps\n",
        gyroBiasX, gyroBiasY, gyroBiasZ
    );
    Serial.printf(
        "Accel at rest: X:%.3f Y:%.3f Z:%.3f g (|a| = %.3f, should be ~1.000)\n",
        avg.ax, avg.ay, avg.az, mag
    );
    Serial.printf(
        "Starting angles: roll:%.2f pitch:%.2f deg\n",
        initialAttitude.roll, initialAttitude.pitch
    );

    return true;
}

// =====================================================
// FREERTOS IMU ACQUISITION TASK
// =====================================================

void imuTask(void *parameter) {

    ImuData imu;

    // Gyro-only angles. This task is the only one that changes them.
    Attitude gyroAngles = initialAttitude;

    // Accel-only angles, recalculated fresh every sample
    float accelRoll = 0.0f;
    float accelPitch = 0.0f;
    float accelMag = 1.0f;

    // Periodic scheduling
    TickType_t lastWakeTime = xTaskGetTickCount();

    // Initial timestamp
    int64_t previousTime = esp_timer_get_time();

    // Timestamp of the last reading added to the gyro angles
    int64_t previousGyroTime = previousTime;

    // Statistics
    uint32_t count = 0;
    uint32_t missedPeriods = 0;

    float minPeriod = 1000000.0f;
    float maxPeriod = 0.0f;

    double sumPeriod = 0.0;
    double sumSquaredPeriod = 0.0;

    float maxAbsJitter = 0.0f;
    double sumAbsJitter = 0.0;

    while (true) {

        // Schedule next 1 ms execution
        vTaskDelayUntil(
            &lastWakeTime,
            pdMS_TO_TICKS(1)
        );

        // Timestamp immediately before IMU read
        int64_t currentTime = esp_timer_get_time();

        // Actual interval between consecutive reads
        float period = (float)(
            currentTime - previousTime
        );

        previousTime = currentTime;

        // Calculate jitter
        float jitter = period - SAMPLE_PERIOD_US;
        float absJitter = fabsf(jitter);

        // Count periods significantly longer than target
        if (period > 1500.0f) {
            missedPeriods++;
        }

        // Read IMU
        if (!readIMU(imu)) {
            continue;
        }

        // Remove gyro bias so a still board reads ~0 dps
        imu.gx -= gyroBiasX;
        imu.gy -= gyroBiasY;
        imu.gz -= gyroBiasZ;

        // dt = real time since the last gyro update, in seconds.
        // Using the measured time (not a fixed 0.001) keeps the angles
        // right even when a period runs a little long or short.
        float dt = (float)(currentTime - previousGyroTime) / 1000000.0f;
        previousGyroTime = currentTime;

        if (dt > 0.0f && dt < MAX_DT_S) {
            integrateGyro(gyroAngles, imu, dt);
        }

        // Tilt straight from the accelerometer, no smoothing
        accelToAngles(imu, accelRoll, accelPitch);
        accelMag = sqrtf(imu.ax * imu.ax + imu.ay * imu.ay + imu.az * imu.az);

        count++;

        // Update timing statistics
        if (period < minPeriod) {
            minPeriod = period;
        }

        if (period > maxPeriod) {
            maxPeriod = period;
        }

        sumPeriod += period;
        sumSquaredPeriod += (double)period * period;

        sumAbsJitter += absJitter;

        if (absJitter > maxAbsJitter) {
            maxAbsJitter = absJitter;
        }

        // Report every REPORT_INTERVAL samples
        if (count >= REPORT_INTERVAL) {

            float avgPeriod =
                (float)(sumPeriod / count);

            float variance =
                (float)(
                    sumSquaredPeriod / count
                    - (double)avgPeriod * avgPeriod
                );

            float stdDev = sqrtf(
                fmaxf(variance, 0.0f)
            );

            TimingReport report;

            report.samples = count;

            report.accelRoll  = accelRoll;
            report.accelPitch = accelPitch;

            report.gyroRoll  = gyroAngles.roll;
            report.gyroPitch = gyroAngles.pitch;
            report.gyroYaw   = gyroAngles.yaw;

            report.gyroX = imu.gx;
            report.gyroY = imu.gy;
            report.gyroZ = imu.gz;

            report.accelMag = accelMag;

            report.avgPeriod = avgPeriod;
            report.minPeriod = minPeriod;
            report.maxPeriod = maxPeriod;

            report.maxAbsJitter = maxAbsJitter;

            report.avgAbsJitter =
                (float)(sumAbsJitter / count);

            report.periodSD = stdDev;
            report.missedPeriods = missedPeriods;

            // Send report without blocking acquisition
            xQueueOverwrite(reportQueue, &report);

            // Reset statistics
            count = 0;
            missedPeriods = 0;

            minPeriod = 1000000.0f;
            maxPeriod = 0.0f;

            sumPeriod = 0.0;
            sumSquaredPeriod = 0.0;

            maxAbsJitter = 0.0f;
            sumAbsJitter = 0.0;
        }
    }
}

// =====================================================
// FREERTOS REPORTING TASK
// =====================================================

void reportingTask(void *parameter) {

    TimingReport report;

    while (true) {

        if (xQueueReceive(
            reportQueue,
            &report,
            portMAX_DELAY
        ) == pdTRUE) {

            Serial.printf(
                "ACCEL Roll:%7.2f Pitch:%7.2f | "
                "GYRO Roll:%7.2f Pitch:%7.2f Yaw:%7.2f deg | "
                "Gyro X:%7.2f Y:%7.2f Z:%7.2f dps | |a|:%.2f g | "
                "N:%lu Period:%.1f us (%.1f-%.1f) "
                "Jitter max:%.1f avg:%.1f SD:%.1f us Missed:%lu\n",

                report.accelRoll,
                report.accelPitch,

                report.gyroRoll,
                report.gyroPitch,
                report.gyroYaw,

                report.gyroX,
                report.gyroY,
                report.gyroZ,

                report.accelMag,

                (unsigned long)report.samples,

                report.avgPeriod,
                report.minPeriod,
                report.maxPeriod,

                report.maxAbsJitter,
                report.avgAbsJitter,

                report.periodSD,

                (unsigned long)report.missedPeriods
            );
        }
    }
}


// =====================================================
// ARDUINO SETUP
// =====================================================

void setup() {

    Serial.begin(115200);

    delay(1000);

    Serial.println("\nStarting ESP32-S3 ICM-42688-P no-filter attitude test");

    // Initialize sensor before starting tasks
    if (!initializeIMU()) {

        Serial.println("IMU initialization failed!");

        while (true) {
            delay(1000);
        }
    }

    // Measure gyro bias and starting tilt (board must be still).
    // This runs before the tasks start, so nothing else uses SPI yet.
    if (!calibrateIMU()) {

        Serial.println("IMU calibration failed!");

        while (true) {
            delay(1000);
        }
    }

    // Create one-element queue for statistics
    reportQueue = xQueueCreate(
        1,
        sizeof(TimingReport)
    );

    if (reportQueue == NULL) {
        Serial.println("Queue creation failed!");
        while (true) {
            delay(1000);
        }
    }

    // Create reporting task on Core 0
    BaseType_t reportResult = xTaskCreatePinnedToCore(
        reportingTask,
        "ReportingTask",
        REPORT_STACK_SIZE,
        NULL,
        REPORT_PRIORITY,
        NULL,
        REPORT_CORE
    );

    if (reportResult != pdPASS) {
        Serial.println("Reporting task creation failed!");
        while (true) {
            delay(1000);
        }
    }

    // Create dedicated IMU acquisition task on Core 1
    BaseType_t imuResult = xTaskCreatePinnedToCore(
        imuTask,
        "ImuTask",
        IMU_STACK_SIZE,
        NULL,
        IMU_PRIORITY,
        NULL,
        IMU_CORE
    );

    if (imuResult != pdPASS) {
        Serial.println("IMU task creation failed!");
        while (true) {
            delay(1000);
        }
    }

    Serial.println("FreeRTOS tasks started.");
}

// =====================================================
// ARDUINO LOOP
// =====================================================

void loop() {

    // No acquisition or reporting in Arduino loop
    vTaskDelay(pdMS_TO_TICKS(1000));
}
