
#include <Arduino.h>
#include <SPI.h>
#include <esp_timer.h>
#include <math.h>

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
#define GYRO_CORE 1
#define REPORT_CORE 0

#define GYRO_PRIORITY 4
#define REPORT_PRIORITY 1

#define GYRO_STACK_SIZE 4096
#define REPORT_STACK_SIZE 4096

// Report every 1000 samples (approximately 1 second)
#define REPORT_INTERVAL 1000

#if (configTICK_RATE_HZ < 1000)
#error "FreeRTOS tick rate must be at least 1000 Hz for 1 ms scheduling"
#endif

// =====================================================
// ICM-42688-P REGISTERS
// =====================================================

#define REG_DEVICE_CONFIG  0x11
#define REG_PWR_MGMT0      0x4E
#define REG_GYRO_CONFIG0   0x4F

#define REG_GYRO_DATA_X1   0x25
#define REG_WHO_AM_I       0x75
#define REG_BANK_SEL       0x76

#define EXPECTED_WHO_AM_I  0x47

// Gyro sensitivity for +/-2000 degrees/second
#define GYRO_SENSITIVITY 16.4f

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

struct GyroData {
    float x;
    float y;
    float z;
};

struct TimingReport {
    uint32_t samples;

    float gyroX;
    float gyroY;
    float gyroZ;

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
    // FS_SEL = 000: +/-2000 dps
    // GYRO_ODR = 0110: 1 kHz
    writeRegister(REG_GYRO_CONFIG0, 0x06);

    // Enable gyro in Low Noise mode
    // GYRO_MODE = 11
    // ACCEL_MODE = 00 (disabled)
    writeRegister(REG_PWR_MGMT0, 0x0C);

    // Allow gyro startup time
    delay(50);

    Serial.println("Gyroscope configured.");
    Serial.println("Gyro ODR: 1000 Hz");
    Serial.println("Gyro range: +/-2000 dps");

    return true;
}

// =====================================================
// READ GYROSCOPE
// =====================================================

bool readGyro(GyroData &gyro) {

    uint8_t data[6];

    // Read all three axes in one SPI transaction
    readRegisters(
        REG_GYRO_DATA_X1,
        data,
        6
    );

    // Combine high and low bytes
    int16_t rawX = (int16_t)(
        ((uint16_t)data[0] << 8) | data[1]
    );

    int16_t rawY = (int16_t)(
        ((uint16_t)data[2] << 8) | data[3]
    );

    int16_t rawZ = (int16_t)(
        ((uint16_t)data[4] << 8) | data[5]
    );

    // Convert raw values to degrees/second
    gyro.x = rawX / GYRO_SENSITIVITY;
    gyro.y = rawY / GYRO_SENSITIVITY;
    gyro.z = rawZ / GYRO_SENSITIVITY;

    return true;
}

// =====================================================
// FREERTOS GYRO ACQUISITION TASK
// =====================================================

void gyroTask(void *parameter) {

    GyroData gyro;

    // Periodic scheduling
    TickType_t lastWakeTime = xTaskGetTickCount();

    // Initial timestamp
    int64_t previousTime = esp_timer_get_time();

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

        // Timestamp immediately before gyro read
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
        if (!readGyro(gyro)) {
            continue;
        }

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

        // Report every 1000 samples
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

            report.gyroX = gyro.x;
            report.gyroY = gyro.y;
            report.gyroZ = gyro.z;

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
                "N:%lu | X:%.2f Y:%.2f Z:%.2f dps | "
                "Period:%.2f us (%.2f-%.2f) | "
                "Jitter max:%.2f us avg:%.2f us | "
                "SD:%.2f us | Missed:%lu\n",

                (unsigned long)report.samples,

                report.gyroX,
                report.gyroY,
                report.gyroZ,

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

    Serial.println("\nStarting ESP32-S3 ICM-42688-P test");

    // Initialize sensor before starting tasks
    if (!initializeIMU()) {

        Serial.println("IMU initialization failed!");

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

    // Create dedicated gyro acquisition task on Core 1
    BaseType_t gyroResult = xTaskCreatePinnedToCore(
        gyroTask,
        "GyroTask",
        GYRO_STACK_SIZE,
        NULL,
        GYRO_PRIORITY,
        NULL,
        GYRO_CORE
    );

    if (gyroResult != pdPASS) {
        Serial.println("Gyro task creation failed!");
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

