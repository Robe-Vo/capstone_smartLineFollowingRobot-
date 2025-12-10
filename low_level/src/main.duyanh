#include <Arduino.h>
#include <ESP32Servo.h>

#include "bluetooth.hpp"
#include "sensors.hpp"

/* ======================= CONFIG ======================= */

#define TIME_SEND_SIGNAL     50   // ms between frames
#define TIME_KICK_ULTRA      60   // ms

#define LINE_SENSOR_IDX_5_PIN 32
#define LINE_SENSOR_IDX_4_PIN 35
#define LINE_SENSOR_IDX_3_PIN 34
#define LINE_SENSOR_IDX_2_PIN 39
#define LINE_SENSOR_IDX_1_PIN 36

#define ULTRASONIC_TRIG_PIN 26
#define ULTRASONIC_ECHO_PIN 27

#define MOTOR_PWM_PIN   5
#define MOTOR_OUT_1_PIN 18
#define MOTOR_OUT_2_PIN 19

// SIMPLE ENCODER: use only channel A for speed
#define ENCODER_PIN     33
#define ENCODER_PPR     44  // pulses per revolution

#define SERVO_PIN 23

/* ======================= ENCODER STATE ======================= */

// Total pulses (for debug)
static volatile int32_t encoder_pulse_total = 0;

// Pulses accumulated in current frame
static volatile int32_t encoder_pulse_frame = 0;

// Speed in RPM (computed once per frame)
static volatile float   encoder_speed = 0.0f;

/* ======================= SENSORS ======================= */

// line_readSignals() returns pointer to 5 bytes
uint8_t*  line_signals = nullptr;

// ultrasonic distance (unit depends on your ultra code)
uint16_t ultra_signal = 0;

// MPU processed signals
uint16_t mpu_signals[6] = {0};

/* ======================= ACTUATORS ======================= */
Servo servo;

/* ======================= COMMUNICATION ======================= */
Network server(5946);
static uint8_t  cmd          = 0x00;
static uint8_t* payload      = nullptr;

static uint8_t  speed        = 0;
static uint16_t steer_angle  = 90;

static volatile bool flag_read_line          = false;
static volatile bool flag_read_ultra         = false;
static volatile bool flag_read_mpu           = false;
static volatile bool flag_run_drive_forward  = false; 
static volatile bool flag_run_drive_backward = false;
static volatile bool flag_run_drive_stop     = false;
static volatile bool flag_disable_steer      = false;
static volatile bool flag_turn_steer         = false;

static volatile bool flag_op_transmit_ready  = true;
static volatile bool flag_op_stop_motor      = false;

static volatile unsigned long tick_1ms       = 0;
static volatile unsigned long tick_transmit  = 0;
static volatile unsigned long tick_kickUltra = 0;

/* Robot operation system */
enum mode { IDLE_MODE, OPERATION_MODE };
volatile mode robot_mode = IDLE_MODE;

/* ======================= TIMER ======================= */
hw_timer_t *Timer0_Cfg = NULL;
void IRAM_ATTR Timer0_ISR();

/* ======================= RTOS TASKS ======================= */
TaskHandle_t IDLE_handle      = NULL;
TaskHandle_t OPERATION_handle = NULL;
TaskHandle_t COM_handle       = NULL;

void IDLE_process(void *parameter);
void OPERATION_process(void *parameter);
void com_process(void *parameter);

/* ======================= HELPERS ======================= */
void driveForward(uint8_t speed);
void stopMotor();
void setSteerAngle(uint16_t angle);

/* ======================= ENCODER ISR ======================= */
void IRAM_ATTR ISR_encoder();

/* ======================= SETUP ======================= */

void setup()
{
    Serial.begin(115200);

    /* -------- Sensors setup -------- */
    line_setup(LINE_SENSOR_IDX_1_PIN,
               LINE_SENSOR_IDX_2_PIN,
               LINE_SENSOR_IDX_3_PIN,
               LINE_SENSOR_IDX_4_PIN,
               LINE_SENSOR_IDX_5_PIN);

    ultra_setup(ULTRASONIC_TRIG_PIN, ULTRASONIC_ECHO_PIN);
    // mpu_setup(21, 22);   // SDA, SCL – change if needed

    // Echo ISR
    attachInterrupt(ULTRASONIC_ECHO_PIN, hanlder_ultra_echo, CHANGE);

    /* -------- Actuators setup -------- */
    pinMode(MOTOR_OUT_1_PIN, OUTPUT);
    pinMode(MOTOR_OUT_2_PIN, OUTPUT);
    pinMode(MOTOR_PWM_PIN,   OUTPUT);

    servo.attach(SERVO_PIN);
    servo.write(steer_angle);    // center

    // Make sure motor is stopped at start
    stopMotor();

    /* -------- Network setup -------- */
    Serial.println("Start to configurate network");
    while (!server.begin())
    {
        Serial.println("Configuration failed");
        delay(1000);
    }

    /* -------- Timer interrupt setup: 1 ms tick -------- */
    // 80 MHz / 80 = 1 MHz → 1000 ticks = 1 ms
    Timer0_Cfg = timerBegin(0, 80, true);
    timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);
    timerAlarmWrite(Timer0_Cfg, 1000, true);  // 1 ms period
    timerAlarmEnable(Timer0_Cfg);

    /* -------- Encoder setup (single channel) -------- */
    pinMode(ENCODER_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), ISR_encoder, RISING);

    robot_mode = IDLE_MODE;

    /* -------- RTOS tasks -------- */
    // Communication (core 0)
    xTaskCreatePinnedToCore(
        com_process,
        "com_process",
        10000,
        NULL,
        1,
        &COM_handle,
        0
    );

    // IDLE (core 1)
    xTaskCreatePinnedToCore(
        IDLE_process,
        "IDLE_process",
        10000,
        NULL,
        1,
        &IDLE_handle,
        1
    );

    // OPERATION (core 1)
    xTaskCreatePinnedToCore(
        OPERATION_process,
        "OPERATION_process",
        10000,
        NULL,
        1,
        &OPERATION_handle,
        1
    );
}

void loop()
{
    // Everything is handled by FreeRTOS tasks
}

/* ======================= TIMER ISR ======================= */

void IRAM_ATTR Timer0_ISR()
{
    tick_1ms++;
}

/* ======================= ENCODER ISR ======================= */

void IRAM_ATTR ISR_encoder()
{
    // Count rising edges
    encoder_pulse_total++;
    encoder_pulse_frame++;
}

/* ======================= COMMUNICATION TASK ======================= */

void com_process(void *parameter)
{
    (void)parameter;

    for (;;)
    {
        switch (robot_mode)
        {
            case IDLE_MODE:
            {
                uint8_t c;
                if (server.getUint8(c))
                {
                    bool ok = true;

                    switch (c)
                    {
                        // Change mode -> OPERATION
                        case 0xFF:
                            robot_mode = OPERATION_MODE;
                            break;

                        // Change mode -> IDLE
                        case 0xFE:
                            robot_mode = IDLE_MODE;
                            break;

                        /* -------- Sensor Commands -------- */

                        case 0xEF: // read line
                            flag_read_line = true;
                            break;

                        case 0xEE: // read ultrasonic
                            flag_read_ultra = true;
                            break;

                        case 0xED: // read mpu
                            flag_read_mpu = true;
                            break;

                        /* -------- Drive / Servo Commands -------- */

                        // [0xDF] [speed]
                        case 0xDF:
                        {
                            uint8_t spd;
                            if (server.getUint8(spd))
                            {
                                speed = spd;
                                flag_run_drive_forward = true;
                                Serial.print("Command: ");
                                Serial.print(c);
                                Serial.print(" | ");
                                Serial.println(speed);
                            }
                            else
                            {
                                ok = false;
                            }
                            break;
                        }

                        // [0xDE] [speed]
                        case 0xDE:
                        {
                            uint8_t spd;
                            if (server.getUint8(spd))
                            {
                                speed = spd;
                                flag_run_drive_backward = true;
                            }
                            else
                            {
                                ok = false;
                            }
                            break;
                        }

                        case 0xDD:  // stop motor
                            flag_run_drive_stop = true;
                            break;

                        case 0xDC:  // disable steer
                            flag_disable_steer = true;
                            break;

                        // [0xDB] [angle_hi] [angle_lo]
                        case 0xDB:
                        {
                            uint8_t hi, lo;
                            if (server.getUint8(hi) && server.getUint8(lo))
                            {
                                steer_angle =
                                    (uint16_t(hi) << 8) | uint16_t(lo);
                                flag_turn_steer = true;
                            }
                            else
                            {
                                ok = false;
                            }
                            break;
                        }

                        case 0xA0:
                        {
                            uint32_t t_rx = micros();
                            server.transmitUint8(0xA1);
                            uint32_t t_tx = micros();
                            Serial.print("[PING] dt_us = ");
                            Serial.println(t_tx - t_rx);
                            ok = false;
                            break;
                        }

                        default:
                            break;
                    }

                    if (ok)
                    {
                        server.transmitUint8(0x20); // ACK
                        Serial.print("Data receive in IDLE at: ");
                        Serial.println(tick_1ms);
                    }
                }
                break;
            }

            case OPERATION_MODE:
            {
                // ----- Send frame if ready -----
                if (flag_op_transmit_ready)
                {
                    // 1) Take pulses and reset frame counter
                    int32_t pulses;
                    noInterrupts();
                    pulses = encoder_pulse_frame;
                    encoder_pulse_frame = 0;
                    interrupts();

                    // 2) Compute speed from pulses per TIME_SEND_SIGNAL
                    float pulses_f     = (float)pulses;
                    float pulses_per_s = pulses_f * (1000.0f / (float)TIME_SEND_SIGNAL);
                    float rev_per_s    = pulses_per_s / (float)ENCODER_PPR;
                    float rpm          = rev_per_s * 60.0f;
                    if (rpm < 0.0f) rpm = 0.0f;  // single channel -> no direction

                    encoder_speed = rpm;

                    // Debug to Serial
                    Serial.print("[ENC] pulses_frame=");
                    Serial.print(pulses);
                    Serial.print("  rpm=");
                    Serial.println(encoder_speed);

                    // 3) Build frame (22 bytes):
                    // [0..4]   : 5 bytes line sensors
                    // [5..6]   : 2 bytes ultrasonic (uint16, big-endian)
                    // [7..18]  : reserved (0)
                    // [19]     : encoder counter (uint8, pulses this frame)
                    // [20..21] : encoder_speed[rpm]*100 (uint16, big-endian)
                    const uint8_t FRAME_LEN = 22;
                    uint8_t frame[FRAME_LEN] = {0};

                    // line sensors
                    for (int i = 0; i < 5; ++i)
                    {
                        frame[i] = line_signals ? line_signals[i] : 0;
                    }

                    // ultrasonic
                    frame[5] = (uint8_t)(ultra_signal >> 8);
                    frame[6] = (uint8_t)(ultra_signal & 0xFF);

                    // encoder pulses (clip to 0..255)
                    if (pulses < 0)   pulses = 0;
                    if (pulses > 255) pulses = 255;
                    frame[19] = (uint8_t)pulses;

                    // speed: float rpm -> uint16 (scale 100)
                    float spd_f = encoder_speed;
                    if (spd_f < 0.0f)     spd_f = 0.0f;
                    uint16_t spd_q = (uint16_t)(spd_f);   // store rpm directly

                    frame[20] = (uint8_t)(spd_q >> 8);
                    frame[21] = (uint8_t)(spd_q & 0xFF);

                    server.transmitArrayUint8(frame, FRAME_LEN);
                    flag_op_transmit_ready = false;
                }

                // ----- Receive control packet [cmd, speed, angle_hi, angle_lo] -----
                uint8_t ctrl[4];
                if (server.getArrayUint8(ctrl, 4))
                {
                    uint8_t  c     = ctrl[0];
                    uint8_t  spd   = ctrl[1];
                    uint16_t angle = (uint16_t(ctrl[2]) << 8) | uint16_t(ctrl[3]);

                    switch (c)
                    {
                        case 0xF0: // stop
                            speed = 0;
                            flag_op_stop_motor = true;
                            break;

                        case 0xF1: // forward + steering
                            speed       = spd;
                            steer_angle = angle;
                            break;

                        case 0xFE: // back to IDLE
                            robot_mode = IDLE_MODE;
                            server.transmitUint8(0x20);
                            stopMotor();
                            break;

                        default:
                            break;
                    }

                    Serial.print("Data receive in OPERATION at: ");
                    Serial.println(tick_1ms);
                }

                break;
            }

            default:
                break;
        }

        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

/* ======================= IDLE TASK ======================= */

void IDLE_process(void *parameter)
{
    (void)parameter;

    for (;;)
    {
        if (robot_mode == IDLE_MODE)
        {
            /* ---------- Sensor requests ---------- */

            if (flag_read_line)
            {
                flag_read_line = false;

                line_signals = line_readSignals();  // uint8_t[5]
                if (line_signals != nullptr)
                {
                    server.transmitArrayUint8(line_signals, 5);
                }
            }

            if (flag_read_ultra)
            {
                flag_read_ultra = false;

                ultra_signal = ultra_getSignal();   // uint16_t

                uint8_t buf[2];
                buf[0] = (uint8_t)(ultra_signal >> 8);
                buf[1] = (uint8_t)(ultra_signal & 0xFF);
                server.transmitArrayUint8(buf, 2);
            }

            if (flag_read_mpu)
            {
                flag_read_mpu = false;

                uint8_t buf[12];
                for (int i = 0; i < 6; ++i)
                {
                    buf[2 * i]     = (uint8_t)(mpu_signals[i] >> 8);
                    buf[2 * i + 1] = (uint8_t)(mpu_signals[i] & 0xFF);
                }
                server.transmitArrayUint8(buf, 12);
            }

            /* ---------- Motor commands ---------- */

            if (flag_run_drive_forward)
            {
                flag_run_drive_forward = false;
                digitalWrite(MOTOR_OUT_1_PIN, HIGH);
                digitalWrite(MOTOR_OUT_2_PIN, LOW);
                analogWrite(MOTOR_PWM_PIN, speed);
            }

            if (flag_run_drive_backward)
            {
                flag_run_drive_backward = false;
                digitalWrite(MOTOR_OUT_1_PIN, LOW);
                digitalWrite(MOTOR_OUT_2_PIN, HIGH);
                analogWrite(MOTOR_PWM_PIN, speed);
            }

            if (flag_run_drive_stop)
            {
                flag_run_drive_stop = false;
                stopMotor();
            }

            /* ---------- Steering commands ---------- */

            if (flag_disable_steer)
            {
                flag_disable_steer = false;
                if (servo.attached())
                    servo.detach();
            }

            if (flag_turn_steer)
            {
                flag_turn_steer = false;

                if (!servo.attached())
                    servo.attach(SERVO_PIN);

                uint16_t angle = steer_angle;
                if (angle > 180) angle = 180;
                servo.write(angle);
            }
        }

        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

/* ======================= OPERATION TASK ======================= */

void OPERATION_process(void *parameter)
{
    (void)parameter;

    speed       = 0;
    steer_angle = 95;
    if (!servo.attached()) servo.attach(SERVO_PIN);

    for (;;)
    {
        if (robot_mode == OPERATION_MODE)
        {
            // Timing for sending frame
            if ((tick_1ms - tick_transmit) >= TIME_SEND_SIGNAL)
            {
                flag_op_transmit_ready = true;
                tick_transmit          = tick_1ms;
            }

            // Read sensors
            line_signals = line_readSignals();
            ultra_signal = ultra_getSignal();

            // Kick ultrasonic periodically
            if ((tick_1ms - tick_kickUltra) >= TIME_KICK_ULTRA)
            {
                ultra_kick();
                tick_kickUltra = tick_1ms;
            }

            // Motor & steering
            if (flag_op_stop_motor)
            {
                stopMotor();
                flag_op_stop_motor = false;
            }
            else
            {
                digitalWrite(MOTOR_OUT_1_PIN, HIGH);
                digitalWrite(MOTOR_OUT_2_PIN, LOW);
                analogWrite(MOTOR_PWM_PIN, speed);
                servo.write(steer_angle);
            }
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

/* ======================= HELPERS ======================= */

void driveForward(uint8_t spd)
{
    digitalWrite(MOTOR_OUT_1_PIN, HIGH);
    digitalWrite(MOTOR_OUT_2_PIN, LOW);
    analogWrite(MOTOR_PWM_PIN, spd);
}

void stopMotor()
{
    analogWrite(MOTOR_PWM_PIN, 0);
    digitalWrite(MOTOR_OUT_1_PIN, LOW);
    digitalWrite(MOTOR_OUT_2_PIN, LOW);
}

void setSteerAngle(uint16_t angle)
{
    if (angle > 180) angle = 180;
    servo.write(angle);
}
