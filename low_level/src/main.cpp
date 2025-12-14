// ======================= main.cpp (QUICK PROTOTYPE, STRUCTURE LIKE OLD) =======================
// Changes implemented per request:
// 1) OPERATION receive frame: 1 byte cmd | 2 byte speed | 2 byte angle |
//    - speed: uint16_t -> PWM 16-bit for DC motor
//    - angle: uint16_t treated as DEG (55..105) for servo.write()
// 2) Motor DC: PWM 16-bit via LEDC (replaces analogWrite)
// 3) Encoder: add hybrid measurement (period + accumulate) with requested architecture
//    - Encoder "resolution = 1": report speeds in Hz (pulses/s) directly
// 4) Keep FreeRTOS tasks, flags, timer tick structure similar to provided code

#include <Arduino.h>
#include <ESP32Servo.h>
#include "bluetooth.hpp"
#include "sensors.hpp"
#include "actuators.hpp"
#include "esp_timer.h"
#include "driver/gpio.h"

/* ======================= CONFIG ======================= */
#define TIME_KICK_ULTRA    60   // ms
#define TS_SPEED_MS        20   // ms
#define TS_POSITION_MS     200  // ms

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

#define ENCODER_CHANNEL_A_PIN 33
#define ENCODER_CHANNEL_B_PIN 25

#define SERVO_PIN 23

// PWM 16-bit
#define PWM_FREQ_HZ 20000
#define PWM_RES_BITS 16
#define PWM_CH_MOTOR 0

// Servo degree limit (per request)
#define SERVO_MIN_DEG 55
#define SERVO_MAX_DEG 105

/* ======================= SENSORS ======================= */
uint8_t*  line_signals  = nullptr;
uint16_t  ultra_signal  = 0;
uint16_t  mpu_signals[6]= {0};

/* ======================= ACTUATORS ======================= */
Servo servo;

/* ======================= COMMUNICATION ======================= */
Network server(5946);

/* OPERATION frame: 1B cmd | 2B speed | 2B angle |  => total 5 bytes */
static inline uint16_t u16_le(const uint8_t* p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

/* ======================= STATE ======================= */
static uint16_t speed_pwm16  = 0;   // 0..65535
static uint16_t steer_angle  = 75;  // degree (55..105)

/* ======================= FLAGS & TIMERS ======================= */
static volatile bool     flag_read_line          = false;
static volatile bool     flag_read_ultra         = false;
static volatile bool     flag_read_mpu           = false;
static volatile bool     flag_run_drive_forward  = false;
static volatile bool     flag_run_drive_backward = false;
static volatile bool     flag_run_drive_stop     = false;
static volatile bool     flag_disable_steer      = false;
static volatile bool     flag_turn_steer         = false;
static volatile bool     flag_op_stop_motor      = false;

static volatile uint32_t tick_1ms       = 0;
static volatile uint32_t tick_kickUltra = 0;

static volatile bool speed_loop_tick    = false;
static volatile bool position_loop_tick = false;

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
static inline void stopMotor() {
    MotorPWM16::stopMotor();
}
static inline void driveForward(uint16_t pwm16) {
    MotorPWM16::driveForward(pwm16);
}
static inline void driveBackward(uint16_t pwm16) {
    MotorPWM16::driveBackward(pwm16);
}
static inline void setSteerAngle_deg(uint16_t deg) {
    if (deg < SERVO_MIN_DEG) deg = SERVO_MIN_DEG;
    if (deg > SERVO_MAX_DEG) deg = SERVO_MAX_DEG;
    if (!servo.attached()) servo.attach(SERVO_PIN);
    servo.write((int)deg);
}

/* ======================= SETUP ======================= */
void setup() {
    Serial.begin(115200);
    delay(300);

    // -------- Sensors --------
    line_setup(LINE_SENSOR_IDX_1_PIN,
               LINE_SENSOR_IDX_2_PIN,
               LINE_SENSOR_IDX_3_PIN,
               LINE_SENSOR_IDX_4_PIN,
               LINE_SENSOR_IDX_5_PIN);

    ultra_setup(ULTRASONIC_TRIG_PIN, ULTRASONIC_ECHO_PIN);
    attachInterrupt(ULTRASONIC_ECHO_PIN, hanlder_ultra_echo, CHANGE);

    // -------- Actuators: Motor PWM 16-bit --------
    MotorPWM16::begin(MOTOR_PWM_PIN, MOTOR_OUT_1_PIN, MOTOR_OUT_2_PIN,
                      PWM_CH_MOTOR, PWM_FREQ_HZ, PWM_RES_BITS);

    // Servo
    servo.attach(SERVO_PIN);
    setSteerAngle_deg(steer_angle);
    stopMotor();

    // -------- Encoder (hybrid) --------
    Encoder::speed_filter_init();

    Encoder::HybridConfig hc;
    hc.Tw_min_s = 0.005f;
    hc.Tw_max_s = 0.007f;
    hc.fLow_hz  = 40.0f;
    hc.fHigh_hz = 80.0f;
    hc.tau_iir_s = 0.050f;
    hc.alpha_min = 0.05f;
    hc.alpha_max = 0.30f;
    hc.a_max_hz_per_s = 500.0f;
    hc.k_zero = 3.0f;
    hc.Tmin_zero_s = 0.035f;
    hc.tau_zero_s  = 0.060f;
    hc.f_dead_hz   = 1.0f;
    hc.Tdead_s     = 0.060f;

    // HARD clamp physical max (Hz). Set placeholder; adjust for your motor+encoder.
    hc.f_hard_max_hz = 300.0f;

    // "resolution = 1" per request (ppr_effective=1)
    Encoder::encoder_begin(ENCODER_CHANNEL_A_PIN, ENCODER_CHANNEL_B_PIN, 1, hc);

    // -------- Network --------
    while (!server.begin()) {
        delay(300);
    }

    // -------- Timer 1 ms --------
    Timer0_Cfg = timerBegin(0, 80, true);      // 80 MHz / 80 = 1 MHz
    timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);
    timerAlarmWrite(Timer0_Cfg, 1000, true);   // 1000 ticks = 1 ms
    timerAlarmEnable(Timer0_Cfg);

    robot_mode = IDLE_MODE;

    // -------- FreeRTOS Tasks --------
    xTaskCreatePinnedToCore(com_process,       "com_process",       4096, NULL, 3, &COM_handle,       0);
    xTaskCreatePinnedToCore(IDLE_process,      "IDLE_process",      4096, NULL, 1, &IDLE_handle,      1);
    xTaskCreatePinnedToCore(OPERATION_process, "OPERATION_process", 6144, NULL, 2, &OPERATION_handle, 1);
}

void loop() {
    // FreeRTOS only
}

/* ======================= TIMER ISR ======================= */
void IRAM_ATTR Timer0_ISR() {
    tick_1ms++;

    static uint16_t cnt_speed = 0;
    static uint16_t cnt_pos   = 0;

    if (++cnt_speed >= TS_SPEED_MS) {
        cnt_speed = 0;
        speed_loop_tick = true;
    }
    if (++cnt_pos >= TS_POSITION_MS) {
        cnt_pos = 0;
        position_loop_tick = true;
    }
}

/* ======================= COMMUNICATION TASK ======================= */
void com_process(void *parameter) {
    (void)parameter;

    static int32_t encoder_total_prev = 0;

    for (;;) {
        if (robot_mode == IDLE_MODE) {
            uint8_t c;
            if (server.getUint8(c)) {
                bool ok = true;

                switch (c) {
                    case 0xFF:
                        robot_mode = OPERATION_MODE;
                        break;
                    case 0xFE:
                        robot_mode = IDLE_MODE;
                        break;

                    case 0xEF: flag_read_line  = true; break;
                    case 0xEE: flag_read_ultra = true; break;
                    case 0xED: flag_read_mpu   = true; break;

                    case 0xDF: {
                        // old: 1 byte speed -> now map into 16-bit (quick compat)
                        uint8_t spd8;
                        if (server.getUint8(spd8)) {
                            speed_pwm16 = (uint16_t)spd8 * 257u; // 0..255 -> 0..65535
                            flag_run_drive_forward = true;
                        } else ok = false;
                    } break;

                    case 0xDE: {
                        uint8_t spd8;
                        if (server.getUint8(spd8)) {
                            speed_pwm16 = (uint16_t)spd8 * 257u;
                            flag_run_drive_backward = true;
                        } else ok = false;
                    } break;

                    case 0xDD:
                        flag_run_drive_stop = true;
                        break;

                    case 0xDC:
                        flag_disable_steer = true;
                        break;

                    case 0xDB: {
                        uint8_t hi, lo;
                        if (server.getUint8(hi) && server.getUint8(lo)) {
                            steer_angle = (uint16_t(hi) << 8) | uint16_t(lo);
                            flag_turn_steer = true;
                        } else ok = false;
                    } break;

                    case 0xEC: {
                        // keep old filter config command; still accepted
                        uint8_t mode;
                        uint8_t a_hi, a_lo, b_hi, b_lo;
                        if (server.getUint8(mode) &&
                            server.getUint8(a_hi) && server.getUint8(a_lo) &&
                            server.getUint8(b_hi) && server.getUint8(b_lo)) {
                            uint16_t a_raw = (uint16_t(a_hi) << 8) | uint16_t(a_lo);
                            uint16_t b_raw = (uint16_t(b_hi) << 8) | uint16_t(b_lo);
                            Encoder::speed_filter_config(mode, a_raw, b_raw);
                        } else ok = false;
                    } break;

                    case 0xA0: {
                        server.transmitUint8(0xA1);
                        ok = false; // no ACK
                    } break;

                    default:
                        break;
                }

                if (ok) server.transmitUint8(0x20);
            }
        }
        else if (robot_mode == OPERATION_MODE) {
            // NEW: Operation frame 5 bytes: cmd | speed(u16) | angle(u16)
            uint8_t ctrl[5];
            if (server.getArrayUint8(ctrl, 5)) {
                uint8_t  c   = ctrl[0];
                uint16_t spd = u16_le(&ctrl[1]);   // PWM 16-bit
                uint16_t ang = u16_le(&ctrl[3]);   // degree 55..105 (expected)

                if (c == 0xF0) {
                    speed_pwm16 = 0;
                    flag_op_stop_motor = true;
                } else if (c == 0xF1) {
                    speed_pwm16 = spd;
                    steer_angle = ang;
                } else if (c == 0xFE) {
                    robot_mode = IDLE_MODE;
                    server.transmitUint8(0x20);
                    stopMotor();
                }

                // Atomic snapshot for pulses/frame (same pattern)
                int32_t total_now = Encoder::encoder_get_total();
                int32_t delta = total_now - encoder_total_prev;
                encoder_total_prev = total_now;

                int32_t pulses_mag = delta;
                if (pulses_mag < 0) pulses_mag = -pulses_mag;
                if (pulses_mag > 255) pulses_mag = 255;

                // Read sensors
                line_signals = line_readSignals();
                ultra_signal = ultra_getSignal();

                // Send speed as uint16 (Hz, from hybrid output magnitude)
                float f_out = Encoder::speed_get_hz_out();
                if (f_out < 0.0f) f_out = -f_out;
                if (f_out > 65535.0f) f_out = 65535.0f;
                uint16_t spd_q = (uint16_t)f_out;

                // Frame 22 bytes (keep same length for MATLAB parser)
                const uint8_t FRAME_LEN = 22;
                uint8_t frame[FRAME_LEN] = {0};

                for (int i = 0; i < 5; ++i) frame[i] = line_signals ? line_signals[i] : 0;

                frame[5] = (uint8_t)(ultra_signal >> 8);
                frame[6] = (uint8_t)(ultra_signal & 0xFF);

                // Keep old fields at end: pulses_mag + speed_u16
                frame[19] = (uint8_t)pulses_mag;
                frame[20] = (uint8_t)(spd_q >> 8);
                frame[21] = (uint8_t)(spd_q & 0xFF);

                server.transmitArrayUint8(frame, FRAME_LEN);
            }
        }

        vTaskDelay(1);
    }
}

/* ======================= IDLE TASK ======================= */
void IDLE_process(void *parameter) {
    (void)parameter;

    for (;;) {
        if (robot_mode == IDLE_MODE) {
            if (flag_read_line) {
                flag_read_line = false;
                line_signals = line_readSignals();
                if (line_signals) server.transmitArrayUint8(line_signals, 5);
            }

            if (flag_read_ultra) {
                flag_read_ultra = false;

                ultra_kick();
                delayMicroseconds(200);

                ultra_signal = ultra_getSignal();
                uint8_t buf[2] = {(uint8_t)(ultra_signal >> 8), (uint8_t)(ultra_signal & 0xFF)};
                server.transmitArrayUint8(buf, 2);
            }

            if (flag_read_mpu) {
                flag_read_mpu = false;
                uint8_t buf[12];
                for (int i = 0; i < 6; ++i) {
                    buf[2*i]   = (uint8_t)(mpu_signals[i] >> 8);
                    buf[2*i+1] = (uint8_t)(mpu_signals[i] & 0xFF);
                }
                server.transmitArrayUint8(buf, 12);
            }

            if (flag_run_drive_forward) {
                flag_run_drive_forward = false;
                driveForward(speed_pwm16);
            }

            if (flag_run_drive_backward) {
                flag_run_drive_backward = false;
                driveBackward(speed_pwm16);
            }

            if (flag_run_drive_stop) {
                flag_run_drive_stop = false;
                stopMotor();
            }

            if (flag_disable_steer) {
                flag_disable_steer = false;
                if (servo.attached()) servo.detach();
            }

            if (flag_turn_steer) {
                flag_turn_steer = false;
                setSteerAngle_deg(steer_angle);
            }
        }

        vTaskDelay(1);
    }
}

/* ======================= OPERATION TASK ======================= */
void OPERATION_process(void *parameter) {
    (void)parameter;

    speed_pwm16  = 0;
    steer_angle  = 95;

    if (!servo.attached()) servo.attach(SERVO_PIN);
    setSteerAngle_deg(steer_angle);

    uint32_t last_speed_ms = tick_1ms;

    for (;;) {
        if (robot_mode == OPERATION_MODE) {
            // Kick ultrasonic
            uint32_t now_ms = tick_1ms;
            if ((now_ms - tick_kickUltra) >= TIME_KICK_ULTRA) {
                ultra_kick();
                tick_kickUltra = now_ms;
            }

            // Hybrid measurement update at TS_SPEED_MS (same tick as old speed loop)
            if (speed_loop_tick) {
                speed_loop_tick = false;

                // dt_s based on scheduler
                uint32_t now2 = tick_1ms;
                float dt_s = (now2 - last_speed_ms) * 1e-3f;
                if (dt_s <= 0.0f) dt_s = TS_SPEED_MS * 1e-3f;
                last_speed_ms = now2;

                Encoder::hybrid_update(dt_s, now2);
            }

            if (flag_op_stop_motor) {
                stopMotor();
                flag_op_stop_motor = false;
            } else {
                driveForward(speed_pwm16);
                setSteerAngle_deg(steer_angle);
            }
        }

        vTaskDelay(1);
    }
}
