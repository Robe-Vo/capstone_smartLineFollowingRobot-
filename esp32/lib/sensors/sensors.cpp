#include "sensors.hpp"
#include <math.h>

// General function
void sensors_setup()
{
    line_setup();
    ultra_setup();
}


/* ======================= LINE SENSORS ======================= */

// Stored line signal
static uint8_t line_signals[5];

// Threshold for analog line sensor (ESP32 ADC: 0–4095)
static const int LINE_THRESHOLD = 2000;   // adjust in practice

void line_setup()
{
    pinMode(LINE_SENSOR_IDX_1_PIN, INPUT);
    pinMode(LINE_SENSOR_IDX_2_PIN, INPUT);
    pinMode(LINE_SENSOR_IDX_3_PIN, INPUT);
    pinMode(LINE_SENSOR_IDX_4_PIN, INPUT);
    pinMode(LINE_SENSOR_IDX_5_PIN, INPUT);
}

/**
 * Read 5 sensors and return bitmask:
 */
uint8_t* line_readSignals()
{
    int v1 = analogRead(LINE_SENSOR_IDX_1_PIN);
    int v2 = analogRead(LINE_SENSOR_IDX_2_PIN);
    int v3 = analogRead(LINE_SENSOR_IDX_3_PIN);
    int v4 = analogRead(LINE_SENSOR_IDX_4_PIN);
    int v5 = analogRead(LINE_SENSOR_IDX_5_PIN);

    line_signals[0] = v1 >> 4;
    line_signals[1] = v2 >> 4;
    line_signals[2] = v3 >> 4;
    line_signals[3] = v4 >> 4;
    line_signals[4] = v5 >> 4;

    return line_signals;
}

/* ======================= ULTRASONIC (HC-SR04 STYLE) ======================= */

// Ultra-sonics buffer
static uint16_t distance = 0; // (cm)
static volatile bool waiting_echo = true;
uint32_t ultra_timer = 0; // timer

// Setup ultra-sonics
void ultra_setup()
{
    pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
    digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

    // Ultra-sonics ISR
    pinMode(ULTRASONIC_ECHO_PIN, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(ULTRASONIC_ECHO_PIN), hanlder_ultra_echo, CHANGE);
}

void ultra_kick()
{
    // If there is no response from last kick
    if (TS_KICK_ULTRA)
    {
        noInterrupts();
        distance  = 0;
        waiting_echo = false;
        interrupts();
    }

    // Gửi xung trigger 10µs
    digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

    // Chờ echo
    waiting_echo = true;
}


// Ultra-sonic's interrupt: Interrupt every rising edge/falling edge 
void IRAM_ATTR hanlder_ultra_echo()
{
    int level      = digitalRead(ULTRASONIC_ECHO_PIN);
    uint32_t nowUs = micros();

    if (level == HIGH)
    {
        // Rising edge: bắt đầu đo
        ultra_timer = nowUs;
    }
    else
    {
        // Falling edge: kết thúc đo
        uint32_t width_us = 0;
        if (nowUs >= ultra_timer)
        {
            width_us = nowUs - ultra_timer;
        }

        // HC-SR04: distance (cm) ≈ time_us / 58
        distance  = (uint16_t)(width_us / 58U);
        waiting_echo = false;
    }
}


/**
 * Return distance in cm (0 if no new measurement)
 */
uint16_t ultra_getSignal()
{
    noInterrupts();
    uint16_t d = distance;
    interrupts();
    return d;
}

// Help functions
void ultra_setHigh()
{
    digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
}

void ultra_setLow()
{
    digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
}

// Disable ultra's ISR 
void ultra_disable_isr()
{
    detachInterrupt(digitalPinToInterrupt(ULTRASONIC_ECHO_PIN));
}

// Enable ultra's ISR
void ultra_enable_isr()
{
    attachInterrupt(digitalPinToInterrupt(ULTRASONIC_ECHO_PIN), hanlder_ultra_echo, CHANGE);
}

/* ======================= MPU6050 ======================= */

// // I2C address
// static const uint8_t MPU_ADDR = 0x68;

// // Stored I2C pins (for ESP32)
// static uint8_t mpu_sda_pin;
// static uint8_t mpu_scl_pin;

// // RAW buffer: [Ax, Ay, Az, Gx, Gy, Gz] (each cast to uint16_t)
// static uint16_t mpu_raw_buf[6] = {0};

// // PROCESSED buffer: [0] = angle (deg * 100), [1] = displacement (mm * 10)
// static uint16_t mpu_proc_buf[2] = {0, 0};

// // State for integration
// static float mpu_velocity_mps   = 0.0f;
// static float mpu_displacement_m = 0.0f;

// void mpu_setup(uint8_t MPU_SDA_PIN, uint8_t MPU_SCL_PIN)
// {
//     mpu_sda_pin = MPU_SDA_PIN;
//     mpu_scl_pin = MPU_SCL_PIN;

//     // ESP32: Wire.begin(SDA, SCL)
//     Wire.begin(mpu_sda_pin, mpu_scl_pin);

//     // Wake up MPU6050
//     Wire.beginTransmission(MPU_ADDR);
//     Wire.write(0x6B);   // PWR_MGMT_1
//     Wire.write(0x00);   // clear sleep bit
//     Wire.endTransmission(true);

//     // Accel: ±2g
//     Wire.beginTransmission(MPU_ADDR);
//     Wire.write(0x1C);   // ACCEL_CONFIG
//     Wire.write(0x00);
//     Wire.endTransmission(true);

//     // Gyro: ±250 °/s
//     Wire.beginTransmission(MPU_ADDR);
//     Wire.write(0x1B);   // GYRO_CONFIG
//     Wire.write(0x00);
//     Wire.endTransmission(true);
// }

// /** Internal helper: read raw accel + gyro */
// static bool mpu_readRaw(int16_t &AcX, int16_t &AcY, int16_t &AcZ,
//                         int16_t &GyX, int16_t &GyY, int16_t &GyZ)
// {
//     // Start from ACCEL_XOUT_H
//     Wire.beginTransmission(MPU_ADDR);
//     Wire.write(0x3B);
//     Wire.endTransmission(false);
//     Wire.requestFrom(MPU_ADDR, (uint8_t)14, (uint8_t)true);

//     if (Wire.available() < 14)
//     {
//         return false;
//     }

//     AcX = (Wire.read() << 8) | Wire.read();
//     AcY = (Wire.read() << 8) | Wire.read();
//     AcZ = (Wire.read() << 8) | Wire.read();

//     // Skip temperature
//     Wire.read();
//     Wire.read();

//     GyX = (Wire.read() << 8) | Wire.read();
//     GyY = (Wire.read() << 8) | Wire.read();
//     GyZ = (Wire.read() << 8) | Wire.read();

//     return true;
// }

// /** Internal helper: compute angle + forward acceleration from accel */
// static void mpu_computeAngleAccel(float &angle_deg, float &acc_forward_mps2)
// {
//     int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;
//     if (!mpu_readRaw(AcX, AcY, AcZ, GyX, GyY, GyZ))
//     {
//         angle_deg        = 0.0f;
//         acc_forward_mps2 = 0.0f;
//         return;
//     }

//     float ax = (float)AcX;
//     float ay = (float)AcY;
//     float az = (float)AcZ;

//     // Simple pitch from accelerometer
//     float pitch_rad = atan2f(-ax, sqrtf(ay * ay + az * az));
//     angle_deg       = pitch_rad * 180.0f / PI;

//     // Acc scale ±2g → 16384 LSB/g
//     const float ACC_SCALE = 16384.0f;
//     float ax_g = ax / ACC_SCALE;
//     acc_forward_mps2 = ax_g * 9.80665f;
// }

// /**
//  * Return RAW signals buffer:
//  *  [0] = Ax (raw int16 cast to uint16)
//  *  [1] = Ay
//  *  [2] = Az
//  *  [3] = Gx
//  *  [4] = Gy
//  *  [5] = Gz
//  */
// uint16_t* mpu_getSignals()
// {
//     int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;
//     if (!mpu_readRaw(AcX, AcY, AcZ, GyX, GyY, GyZ))
//     {
//         // If read fails, leave previous values
//         return mpu_raw_buf;
//     }

//     mpu_raw_buf[0] = (uint16_t)AcX;
//     mpu_raw_buf[1] = (uint16_t)AcY;
//     mpu_raw_buf[2] = (uint16_t)AcZ;
//     mpu_raw_buf[3] = (uint16_t)GyX;
//     mpu_raw_buf[4] = (uint16_t)GyY;
//     mpu_raw_buf[5] = (uint16_t)GyZ;

//     return mpu_raw_buf;
// }

// /**
//  * Return PROCESSED signals buffer:
//  *  [0] = angle (deg * 100, e.g. 1234 = 12.34°)
//  *  [1] = displacement (mm * 10, i.e. 1 unit = 0.1 mm)
//  *
//  * delta_t is in milliseconds (typical: your scheduler period)
//  */
// uint16_t* mpu_getSignals(size_t delta_t)
// {
//     float angle_deg, acc_forward;
//     mpu_computeAngleAccel(angle_deg, acc_forward);

//     // Integrate acceleration → velocity → displacement
//     float dt = (float)delta_t / 1000.0f;  // ms → s

//     mpu_velocity_mps   += acc_forward * dt;
//     mpu_displacement_m += mpu_velocity_mps * dt;

//     // Angle scaled: deg * 100
//     int16_t angle_scaled = (int16_t)(angle_deg * 100.0f);
//     mpu_proc_buf[0]      = (uint16_t)angle_scaled;

//     // Displacement: meters → mm*10
//     float   disp_mm  = mpu_displacement_m * 1000.0f;
//     int16_t disp10   = (int16_t)(disp_mm * 10.0f);   // 0.1 mm units
//     mpu_proc_buf[1]  = (uint16_t)disp10;

//     return mpu_proc_buf;
// }
