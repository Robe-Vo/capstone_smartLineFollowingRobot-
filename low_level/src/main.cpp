#include <Arduino.h>
#include <ESP32Servo.h>

#include "bluetooth.hpp"
#include "sensors.hpp"
#include "actuators.hpp"

/* ======================= CONFIG ======================= */

#define TIME_SEND_SIGNAL       50   // ms between frames
#define TIME_KICK_ULTRA        60   // ms

#define LINE_SENSOR_IDX_5_PIN  32
#define LINE_SENSOR_IDX_4_PIN  35
#define LINE_SENSOR_IDX_3_PIN  34
#define LINE_SENSOR_IDX_2_PIN  39
#define LINE_SENSOR_IDX_1_PIN  36

#define ULTRASONIC_TRIG_PIN    26
#define ULTRASONIC_ECHO_PIN    27

#define MOTOR_PWM_PIN          5
#define MOTOR_OUT_1_PIN        18
#define MOTOR_OUT_2_PIN        19

// Encoder channels
#define ENCODER_CHANNEL_A_PIN  33   // Channel A
#define ENCODER_CHANNEL_B_PIN  25   // Channel B
#define ENCODER_PPR            44   // pulses per revolution (per channel A edge)
#define ENCODER_WAIT_RESPONSE_TIME 200  // ms

#define SERVO_PIN              23

#define DEBUG_ENCODER          1    // set 0 to disable Serial debug in ISRs

/* ======================= ENCODER STATE ======================= */

// Total pulses (for distance, signed)
static volatile int32_t encoder_pulse_total = 0;

// Pulses accumulated in current frame (for speed)
static volatile int32_t encoder_pulse_frame = 0;

// Speed in RPM (computed once per frame)
static volatile float   encoder_speed = 0.0f;

/* ======================= SENSORS ======================= */

// line_readSignals() returns pointer to 5 bytes
uint8_t*  line_signals = nullptr;

// ultrasonic distance
uint16_t ultra_signal = 0;

// MPU processed signals
uint16_t mpu_signals[6] = {0};

/* ======================= ACTUATORS ======================= */

static Actuators::Drive drive(
    MOTOR_OUT_1_PIN,
    MOTOR_OUT_2_PIN,
    MOTOR_PWM_PIN,
    ENCODER_CHANNEL_A_PIN,
    ENCODER_CHANNEL_B_PIN,
    ENCODER_PPR,
    ENCODER_WAIT_RESPONSE_TIME);

static Actuators::Steer steer(SERVO_PIN);

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

// safety enable/disable in IDLE
static volatile bool flag_drive_enable       = false;
static volatile bool flag_drive_disable      = false;

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

/* ======================= ENCODER ISR PROTOTYPES ======================= */
void IRAM_ATTR ISR_encoder_A();
void IRAM_ATTR ISR_encoder_B();

/* ======================= HELPERS ======================= */
void driveForward(uint8_t speed);
void stopMotor();
void setSteerAngle(uint16_t angle);

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

    // Echo ISR for ultrasonic
    attachInterrupt(ULTRASONIC_ECHO_PIN, hanlder_ultra_echo, CHANGE);

    /* -------- Encoder setup (A & B) -------- */
    pinMode(ENCODER_CHANNEL_A_PIN, INPUT_PULLUP);
    pinMode(ENCODER_CHANNEL_B_PIN, INPUT_PULLUP);

    // Channel A: use for speed + direction
    attachInterrupt(digitalPinToInterrupt(ENCODER_CHANNEL_A_PIN),
                    ISR_encoder_A,
                    CHANGE);

    // Channel B: debug + better quadrature (trigger on any edge)
    attachInterrupt(digitalPinToInterrupt(ENCODER_CHANNEL_B_PIN),
                    ISR_encoder_B,
                    CHANGE);

    /* -------- Steer initial position -------- */
    steer.enable();
    steer_angle = 90;
    steer.turn((int)steer_angle);

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

/* ======================= ENCODER ISRs ======================= */

// Channel A ISR: main pulse counter for speed & distance
void IRAM_ATTR ISR_encoder_A()
{
    // Signed pulse counting can be extended if you want to use direction
    encoder_pulse_total++;
    encoder_pulse_frame++;

    // Use Drive encoder handler (quadrature + speed estimation)
    drive.Endcoder_channel_A_ISR();

#if DEBUG_ENCODER
    static uint32_t lastPrintA = 0;
    uint32_t now = micros();
    if (now - lastPrintA > 5000)   // print every ~5 ms max (debug only)
    {
        Serial.print("[ENC A] total=");
        Serial.print(encoder_pulse_total);
        Serial.print(" frame=");
        Serial.print(encoder_pulse_frame);
        Serial.print("  driveSpeed=");
        Serial.println(drive.get_respondedSpeed());
        lastPrintA = now;
    }
#endif
}

// Channel B ISR: second channel used for direction + debug
void IRAM_ATTR ISR_encoder_B()
{
    drive.Endcoder_channel_B_ISR();

#if DEBUG_ENCODER
    static uint32_t lastPrintB = 0;
    uint32_t now = micros();
    if (now - lastPrintB > 5000)   // print every ~5 ms max (debug only)
    {
        Serial.print("[ENC B] driveSpeed=");
        Serial.println(drive.get_respondedSpeed());
        lastPrintB = now;
    }
#endif
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

                        /* -------- Drive Safety Commands in IDLE -------- */
                        // [0xE1] : enable drive
                        case 0xE1:
                            flag_drive_enable = true;
                            break;

                        // [0xE2] : disable drive
                        case 0xE2:
                            flag_drive_disable = true;
                            break;

                        /* -------- Drive / Servo Commands -------- */

                        // [0xDF] [speed] : forward in IDLE
                        case 0xDF:
                        {
                            uint8_t spd;
                            if (server.getUint8(spd))
                            {
                                speed = spd;
                                flag_run_drive_forward = true;
                                Serial.print("Command: ");
                                Serial.print(c);
                                Serial.print(" | speed=");
                                Serial.println(speed);
                            }
                            else
                            {
                                ok = false;
                            }
                            break;
                        }

                        // [0xDE] [speed] : backward in IDLE
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

                    // 2) Compute speed from pulses per TIME_SEND_SIGNAL [ms]
                    float pulses_f     = (float)pulses;
                    float pulses_per_s = pulses_f * (1000.0f / (float)TIME_SEND_SIGNAL);
                    float rev_per_s    = pulses_per_s / (float)ENCODER_PPR;
                    float rpm          = rev_per_s * 60.0f;

                    if (rpm < 0.0f) rpm = 0.0f;

                    encoder_speed = rpm;

#if DEBUG_ENCODER
                    Serial.print("[ENC FRAME] pulses_frame=");
                    Serial.print(pulses);
                    Serial.print("  rpm=");
                    Serial.println(encoder_speed);
#endif

                    // 3) Build frame (22 bytes):
                    // [0..4]   : 5 bytes line sensors
                    // [5..6]   : 2 bytes ultrasonic (uint16, big-endian)
                    // [7..18]  : reserved (0)
                    // [19]     : encoder pulses this frame (uint8)
                    // [20..21] : encoder_speed [rpm] (uint16, big-endian)
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

                    // speed: float rpm -> uint16 (integer rpm)
                    float    spd_f = encoder_speed;
                    if (spd_f < 0.0f)    spd_f = 0.0f;
                    if (spd_f > 65535.0f) spd_f = 65535.0f;
                    uint16_t spd_q = (uint16_t)(spd_f);

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
            /* ---------- Drive safety enable/disable ---------- */

            if (flag_drive_enable)
            {
                flag_drive_enable = false;
                drive.enable();
                Serial.println("[DRIVE] enabled");
            }

            if (flag_drive_disable)
            {
                flag_drive_disable = false;
                drive.disable();
                Serial.println("[DRIVE] disabled");
            }

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
                drive.driving(speed, true);   // forward
            }

            if (flag_run_drive_backward)
            {
                flag_run_drive_backward = false;
                drive.driving(speed, false);  // backward
            }

            if (flag_run_drive_stop)
            {
                flag_run_drive_stop = false;
                drive.brake();
            }

            /* ---------- Steering commands ---------- */

            if (flag_disable_steer)
            {
                flag_disable_steer = false;
                steer.disable();
            }

            if (flag_turn_steer)
            {
                flag_turn_steer = false;
                steer.enable();
                uint16_t angle = steer_angle;
                if (angle > 180) angle = 180;
                steer.turn((int)angle);
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
    steer.enable();
    steer.turn((int)steer_angle);

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

            // Motor & steering control
            if (flag_op_stop_motor)
            {
                stopMotor();
                flag_op_stop_motor = false;
            }
            else
            {
                drive.driving(speed, true);  // forward in normal operation
                uint16_t angle = steer_angle;
                if (angle > 180) angle = 180;
                steer.turn((int)angle);
            }
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

/* ======================= HELPERS ======================= */

void driveForward(uint8_t spd)
{
    drive.driving(spd, true);
}

void stopMotor()
{
    drive.brake();
}

void setSteerAngle(uint16_t angle)
{
    if (angle > 180) angle = 180;
    steer.turn((int)angle);
}
