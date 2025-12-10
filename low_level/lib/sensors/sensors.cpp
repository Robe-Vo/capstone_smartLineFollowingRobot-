#include "sensors.hpp"
#include <math.h>

/* ======================= LINE SENSORS ======================= */

// Stored pin numbers
static uint8_t line_pin_1;
static uint8_t line_pin_2;
static uint8_t line_pin_3;
static uint8_t line_pin_4;
static uint8_t line_pin_5;
static uint8_t line_signals[5];

// Threshold for analog line sensor (ESP32 ADC: 0–4095)
static const int LINE_THRESHOLD = 2000;   // adjust in practice

void line_setup(uint8_t LINE_SENSOR_PIN_1,
                uint8_t LINE_SENSOR_PIN_2,
                uint8_t LINE_SENSOR_PIN_3,
                uint8_t LINE_SENSOR_PIN_4,
                uint8_t LINE_SENSOR_PIN_5)
{
    line_pin_1 = LINE_SENSOR_PIN_1;
    line_pin_2 = LINE_SENSOR_PIN_2;
    line_pin_3 = LINE_SENSOR_PIN_3;
    line_pin_4 = LINE_SENSOR_PIN_4;
    line_pin_5 = LINE_SENSOR_PIN_5;

    pinMode(line_pin_1, INPUT);
    pinMode(line_pin_2, INPUT);
    pinMode(line_pin_3, INPUT);
    pinMode(line_pin_4, INPUT);
    pinMode(line_pin_5, INPUT);
}

/**
 * Read 5 sensors and return bitmask:
 */
uint8_t* line_readSignals()
{
    int v1 = analogRead(line_pin_1);
    int v2 = analogRead(line_pin_2);
    int v3 = analogRead(line_pin_3);
    int v4 = analogRead(line_pin_4);
    int v5 = analogRead(line_pin_5);

    line_signals[0] = v1 >> 4;
    line_signals[1] = v2 >> 4;
    line_signals[2] = v3 >> 4;
    line_signals[3] = v4 >> 4;
    line_signals[4] = v5 >> 4;

    return line_signals;

}

/* ======================= ULTRASONIC (HC-SR04 STYLE) ======================= */

// Stored pins
static uint8_t ultra_trig_pin = 0;
static uint8_t ultra_echo_pin = 0;

// Shared state between ISR and main
static volatile uint32_t echo_start_us = 0;
static volatile uint16_t distance_cm   = 0;  // last measured distance
static volatile bool     waiting_echo  = false; // true after kick until falling edge

// Obstacle threshold (cm)
static const uint16_t OBSTACLE_MAX_DIST_CM = 300; // chỉnh theo ý bạn

void ultra_setup(uint8_t ULTRA_TRIG_PIN, uint8_t ULTRA_ECHO_PIN)
{
    ultra_trig_pin = ULTRA_TRIG_PIN;
    ultra_echo_pin = ULTRA_ECHO_PIN;

    pinMode(ultra_trig_pin, OUTPUT);
    pinMode(ultra_echo_pin, INPUT);

    digitalWrite(ultra_trig_pin, LOW);

    // NOTE: KHÔNG gọi attachInterrupt ở đây nữa.
    // ISR hanlder_ultra_echo() vẫn nằm trong file này,
    // nhưng attachInterrupt sẽ được gọi ở main.
}

bool ultra_available()
{
    noInterrupts();
    uint16_t d = distance_cm;
    interrupts();

    return (d > 0 && d <= OBSTACLE_MAX_DIST_CM);
}

void ultra_kick()
{
    // Nếu lần trước vẫn chưa có falling edge -> không có phản hồi
    if (waiting_echo)
    {
        noInterrupts();
        distance_cm  = 0;
        waiting_echo = false;
        interrupts();
    }

    // Gửi xung trigger 10µs
    digitalWrite(ultra_trig_pin, LOW);
    delayMicroseconds(2);
    digitalWrite(ultra_trig_pin, HIGH);
    delayMicroseconds(10);
    digitalWrite(ultra_trig_pin, LOW);

    // Chờ echo
    waiting_echo = true;
}


/**
 * Send 10 µs trigger pulse
 */
void IRAM_ATTR hanlder_ultra_echo()
{
    int level      = digitalRead(ultra_echo_pin);
    uint32_t nowUs = micros();

    if (level == HIGH)
    {
        // Rising edge: bắt đầu đo
        echo_start_us = nowUs;
    }
    else
    {
        // Falling edge: kết thúc đo
        uint32_t width_us = 0;
        if (nowUs >= echo_start_us)
        {
            width_us = nowUs - echo_start_us;
        }

        // HC-SR04: distance (cm) ≈ time_us / 58
        distance_cm  = (uint16_t)(width_us / 58U);
        waiting_echo = false;
    }
}



/**
 * Return distance in cm (0 if no new measurement)
 */
uint16_t ultra_getSignal()
{
    noInterrupts();
    uint16_t d = distance_cm;
    interrupts();
    return d;
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
