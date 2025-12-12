// ======================= main.cpp =======================
#include <Arduino.h>
#include <ESP32Servo.h>
#include "bluetooth.hpp"
#include "sensors.hpp"
#include "actuators.hpp"

/* ======================= CONFIG ======================= */
#define TIME_SEND_SIGNAL   50   // ms, chu kỳ frame PC
#define TIME_KICK_ULTRA    60   // ms

#define TS_SPEED_MS        20   // ms, chu kỳ cập nhật tốc độ (lọc EMA/IIR)
#define TS_POSITION_MS     200  // ms, chu kỳ vòng vị trí (chưa dùng PID)

#define ENCODER_TIMEOUT_MS 200  // ms, quá lâu không có xung -> rpm = 0

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

// Encoder: quadrature, dùng cả A/B, cả cạnh lên/xuống
#define ENCODER_CHANNEL_A_PIN 33
#define ENCODER_CHANNEL_B_PIN 25
#define ENCODER_PPR           44   // pulses / rev đã nhân 4 cạnh

#define SERVO_PIN 23

#define DEBUG_ENCODER_ISR 0

/* ======================= ENCODER GLOBAL STATE ======================= */
// Tổng xung (dùng cho quãng đường + tính pulses/frame)
volatile int32_t encoder_pulse_total  = 0;
static  int32_t  encoder_pulse_total_prev = 0;
static  int32_t  encoder_pulse_frame      = 0;

// Quadrature state
static volatile uint8_t enc_prev_state   = 0;
static volatile int8_t  encoder_direction = +1;

// Period measurement
static volatile uint32_t enc_last_edge_us   = 0;   // thời điểm cạnh hợp lệ trước
static volatile uint32_t enc_period_us      = 0;   // period mới nhất (us)
static volatile uint32_t enc_last_pulse_ms  = 0;   // tick_1ms tại xung hợp lệ mới nhất

/* ======================= SENSORS ======================= */
uint8_t* line_signals = nullptr;
uint16_t ultra_signal = 0;
uint16_t mpu_signals[6] = {0};

/* ======================= ACTUATORS (servo trong main) ======================= */
Servo servo;

/* ======================= COMMUNICATION ======================= */
Network server(5946);

static uint8_t  cmd         = 0x00;
static uint8_t* payload     = nullptr;
static uint8_t  speed       = 0;
static uint16_t steer_angle = 75;

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

static volatile uint32_t tick_1ms                = 0;
static volatile uint32_t tick_kickUltra          = 0;

static volatile bool speed_loop_tick    = false;
static volatile bool position_loop_tick = false;

/* Robot operation system */
enum mode {
    IDLE_MODE,
    OPERATION_MODE
};
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

/* ======================= ENCODER ISRs ======================= */
void IRAM_ATTR ISR_encoder_AB();

/* ======================= HELPERS ======================= */
void driveForward(uint8_t spd);
void stopMotor();
void setSteerAngle(uint16_t angle);

/* ======================= SETUP ======================= */
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println();
    Serial.println("[SETUP] Booting...");

    // -------- Sensors --------
    line_setup(LINE_SENSOR_IDX_1_PIN,
               LINE_SENSOR_IDX_2_PIN,
               LINE_SENSOR_IDX_3_PIN,
               LINE_SENSOR_IDX_4_PIN,
               LINE_SENSOR_IDX_5_PIN);

    ultra_setup(ULTRASONIC_TRIG_PIN, ULTRASONIC_ECHO_PIN);
    attachInterrupt(ULTRASONIC_ECHO_PIN, hanlder_ultra_echo, CHANGE);

    // -------- Actuators --------
    pinMode(MOTOR_OUT_1_PIN, OUTPUT);
    pinMode(MOTOR_OUT_2_PIN, OUTPUT);
    pinMode(MOTOR_PWM_PIN,   OUTPUT);

    servo.attach(SERVO_PIN);
    servo.write(steer_angle);
    stopMotor();

    // -------- Encoder (quadrature, cả A/B, cả cạnh) --------
    pinMode(ENCODER_CHANNEL_A_PIN, INPUT_PULLUP);
    pinMode(ENCODER_CHANNEL_B_PIN, INPUT_PULLUP);

    uint8_t a0 = digitalRead(ENCODER_CHANNEL_A_PIN) ? 1 : 0;
    uint8_t b0 = digitalRead(ENCODER_CHANNEL_B_PIN) ? 1 : 0;
    enc_prev_state = (a0 << 1) | b0;

    attachInterrupt(digitalPinToInterrupt(ENCODER_CHANNEL_A_PIN), ISR_encoder_AB, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_CHANNEL_B_PIN), ISR_encoder_AB, CHANGE);
    Serial.println("[SETUP] Encoder quadrature interrupts attached");

    // -------- Bộ lọc tốc độ (EMA + IIR bậc 2) --------
    Encoder::speed_filter_init();
    // Mặc định: mode = EMA bậc 1, alpha = 0.3, beta = 0.0
    Encoder::speed_filter_config(0, 300, 0);

    // -------- Network --------
    Serial.println("[SETUP] Start to configurate network");
    while (!server.begin()) {
        Serial.println("[SETUP] Configuration failed");
        delay(1000);
    }
    Serial.println("[SETUP] Network configured");

    // -------- Timer 1 ms --------
    Timer0_Cfg = timerBegin(0, 80, true);      // 80 MHz / 80 = 1 MHz
    timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);
    timerAlarmWrite(Timer0_Cfg, 1000, true);   // 1000 ticks = 1 ms
    timerAlarmEnable(Timer0_Cfg);

    robot_mode = IDLE_MODE;
    Serial.println("[SETUP] Robot in IDLE_MODE");

    // -------- FreeRTOS Tasks --------
    xTaskCreatePinnedToCore(com_process,      "com_process",      10000, NULL, 1, &COM_handle,      0);
    xTaskCreatePinnedToCore(IDLE_process,     "IDLE_process",     10000, NULL, 1, &IDLE_handle,     1);
    xTaskCreatePinnedToCore(OPERATION_process,"OPERATION_process",10000, NULL, 1, &OPERATION_handle,1);
}

void loop() {
    // dùng FreeRTOS, không code trong loop
}

/* ======================= TIMER ISR ======================= */
void IRAM_ATTR Timer0_ISR() {
    tick_1ms++;

    static uint16_t cnt_speed = 0;
    static uint16_t cnt_pos   = 0;

    cnt_speed++;
    if (cnt_speed >= TS_SPEED_MS) {
        cnt_speed = 0;
        speed_loop_tick = true;
    }

    cnt_pos++;
    if (cnt_pos >= TS_POSITION_MS) {
        cnt_pos = 0;
        position_loop_tick = true;
    }
}

/* ======================= ENCODER ISR (quadrature + period) ======================= */
void IRAM_ATTR ISR_encoder_AB() {
    uint32_t now_us = micros();

    uint8_t a = gpio_get_level((gpio_num_t)ENCODER_CHANNEL_A_PIN) ? 1 : 0;
    uint8_t b = gpio_get_level((gpio_num_t)ENCODER_CHANNEL_B_PIN) ? 1 : 0;
    uint8_t newState = (a << 1) | b;

    // Bảng chuyển trạng thái quadrature 4x
    // states: 00,01,11,10 = 0,1,3,2 (lưu ý Gray code)
    static const int8_t quad_table[4][4] = {
        // new:  0   1   2   3  (we use 0,1,2,3 indexing but state 2,3 mapping depends on wiring)
        /*old=0*/ {  0, +1, -1,  0 },
        /*old=1*/ { -1,  0,  0, +1 },
        /*old=2*/ { +1,  0,  0, -1 },
        /*old=3*/ {  0, -1, +1,  0 }
    };

    uint8_t oldState = enc_prev_state & 0x03;
    int8_t step = quad_table[oldState][newState & 0x03];

    enc_prev_state = newState;

    if (step != 0) {
        encoder_pulse_total  += step;
        encoder_pulse_frame  += step;
        encoder_direction     = (step > 0) ? +1 : -1;

        uint32_t dt = now_us - enc_last_edge_us;
        enc_last_edge_us = now_us;

        // bỏ glitch quá nhanh (dt quá nhỏ)
        if (dt > 50) {
            enc_period_us     = dt;
            enc_last_pulse_ms = tick_1ms;
        }

#if DEBUG_ENCODER_ISR
        static uint32_t lastPrint = 0;
        if (now_us - lastPrint > 10000) {
            ets_printf("[ENC] tot=%d frame=%d step=%d\n",
                       (int)encoder_pulse_total,
                       (int)encoder_pulse_frame,
                       (int)step);
            lastPrint = now_us;
        }
#endif
    }
}

/* ======================= COMMUNICATION TASK ======================= */
void com_process(void *parameter) {
    (void)parameter;

    for (;;) {
        switch (robot_mode) {
        case IDLE_MODE: {
            uint8_t c;
            if (server.getUint8(c)) {
                bool ok = true;

                switch (c) {
                case 0xFF: // IDLE -> OPERATION
                    robot_mode = OPERATION_MODE;
                    Serial.println("[MODE] Switch to OPERATION_MODE");
                    break;

                case 0xFE: // stay in IDLE
                    robot_mode = IDLE_MODE;
                    Serial.println("[MODE] Stay in IDLE_MODE");
                    break;

                /* -------- Sensor commands -------- */
                case 0xEF:
                    flag_read_line = true;
                    break;

                case 0xEE:
                    flag_read_ultra = true;
                    break;

                case 0xED:
                    flag_read_mpu = true;
                    break;

                /* -------- Drive / Servo commands -------- */
                // [0xDF] [speed]
                case 0xDF: {
                    uint8_t spd;
                    if (server.getUint8(spd)) {
                        speed = spd;
                        flag_run_drive_forward = true;
                        Serial.print("[IDLE CMD] FWD speed=");
                        Serial.println(speed);
                    } else ok = false;
                    break;
                }

                // [0xDE] [speed]
                case 0xDE: {
                    uint8_t spd;
                    if (server.getUint8(spd)) {
                        speed = spd;
                        flag_run_drive_backward = true;
                        Serial.print("[IDLE CMD] REV speed=");
                        Serial.println(speed);
                    } else ok = false;
                    break;
                }

                case 0xDD:
                    flag_run_drive_stop = true;
                    Serial.println("[IDLE CMD] STOP");
                    break;

                case 0xDC:
                    flag_disable_steer = true;
                    break;

                // [0xDB] [angle_hi] [angle_lo]
                case 0xDB: {
                    uint8_t hi, lo;
                    if (server.getUint8(hi) && server.getUint8(lo)) {
                        steer_angle = (uint16_t(hi) << 8) | uint16_t(lo);
                        flag_turn_steer = true;
                        Serial.print("[IDLE CMD] STEER=");
                        Serial.println(steer_angle);
                    } else ok = false;
                    break;
                }

                // [0xEC] [m] [a_hi] [a_lo] [b_hi] [b_lo]
                // m = 0: EMA bậc 1, alpha = a; m = 1: IIR bậc 2 (cascade), alpha=a, beta=b
                // a_raw,b_raw: uint16, scale 1/1000 (0..1000 -> 0..1.000)
                case 0xEC: {
                    uint8_t mode;
                    uint8_t a_hi, a_lo, b_hi, b_lo;
                    if (server.getUint8(mode) &&
                        server.getUint8(a_hi) && server.getUint8(a_lo) &&
                        server.getUint8(b_hi) && server.getUint8(b_lo)) {

                        uint16_t a_raw = (uint16_t(a_hi) << 8) | uint16_t(a_lo);
                        uint16_t b_raw = (uint16_t(b_hi) << 8) | uint16_t(b_lo);
                        Encoder::speed_filter_config(mode, a_raw, b_raw);

                        Serial.print("[IDLE CMD] ENC filter mode=");
                        Serial.print(mode);
                        Serial.print(" a_raw=");
                        Serial.print(a_raw);
                        Serial.print(" b_raw=");
                        Serial.println(b_raw);
                    } else {
                        ok = false;
                    }
                    break;
                }

                // Ping
                case 0xA0: {
                    uint32_t t_rx = micros();
                    server.transmitUint8(0xA1);
                    uint32_t t_tx = micros();
                    Serial.print("[PING] dt_us = ");
                    Serial.println(t_tx - t_rx);
                    ok = false; // không trả ACK 0x20
                    break;
                }

                default:
                    break;
                }

                if (ok) {
                    server.transmitUint8(0x20); // ACK
                }
            }
            break;
        }

        case OPERATION_MODE: {
            // MATLAB gửi 4 byte control [cmd, speed, angle_hi, angle_lo]
            uint8_t ctrl[4];
            if (server.getArrayUint8(ctrl, 4)) {
                uint8_t c    = ctrl[0];
                uint8_t spd  = ctrl[1];
                uint16_t ang = (uint16_t(ctrl[2]) << 8) | uint16_t(ctrl[3]);

                switch (c) {
                case 0xF0: // STOP
                    speed = 0;
                    flag_op_stop_motor = true;
                    Serial.println("[OPE CMD] STOP");
                    break;

                case 0xF1: // forward + steering
                    speed       = spd;
                    steer_angle = ang;
                    Serial.print("[OPE CMD] FWD speed=");
                    Serial.print(speed);
                    Serial.print(" angle=");
                    Serial.println(steer_angle);
                    break;

                case 0xFE: // back to IDLE
                    robot_mode = IDLE_MODE;
                    server.transmitUint8(0x20);
                    stopMotor();
                    Serial.println("[MODE] Back to IDLE_MODE from OPERATION");
                    break;

                default:
                    break;
                }

                // Pulses per frame: dựa trên encoder_pulse_total
                int32_t total_now;
                noInterrupts();
                total_now = encoder_pulse_total;
                interrupts();

                int32_t delta = total_now - encoder_pulse_total_prev;
                encoder_pulse_total_prev = total_now;
                encoder_pulse_frame      = delta;

                int32_t pulses = encoder_pulse_frame;

                // Đọc line + ultrasonic
                line_signals = line_readSignals();
                ultra_signal = ultra_getSignal();

                // Lấy tốc độ đã lọc (EMA/IIR)
                float rpm_filt = Encoder::speed_get_rpm();

                // Build frame 22 byte
                const uint8_t FRAME_LEN = 22;
                uint8_t frame[FRAME_LEN] = {0};

                // line sensors
                for (int i = 0; i < 5; ++i) {
                    frame[i] = line_signals ? line_signals[i] : 0;
                }

                // ultrasonic
                frame[5] = (uint8_t)(ultra_signal >> 8);
                frame[6] = (uint8_t)(ultra_signal & 0xFF);

                // pulses per frame (clamp 0..255 magnitude)
                if (pulses < 0)   pulses = -pulses;   // nếu cần magnitude
                if (pulses > 255) pulses = 255;
                frame[19] = (uint8_t)pulses;

                // speed (rpm) đã lọc, uint16 big-endian
                if (rpm_filt < 0.0f)      rpm_filt = -rpm_filt; // gửi magnitude, nếu cần dấu thì mở rộng frame
                if (rpm_filt > 65535.0f)  rpm_filt = 65535.0f;
                uint16_t spd_q = (uint16_t)rpm_filt;
                frame[20] = (uint8_t)(spd_q >> 8);
                frame[21] = (uint8_t)(spd_q & 0xFF);

                server.transmitArrayUint8(frame, FRAME_LEN);
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
void IDLE_process(void *parameter) {
    (void)parameter;

    for (;;) {
        if (robot_mode == IDLE_MODE) {
            if (flag_read_line) {
                flag_read_line = false;
                line_signals = line_readSignals();
                if (line_signals != nullptr) {
                    server.transmitArrayUint8(line_signals, 5);
                }
            }

            if (flag_read_ultra) {
                flag_read_ultra = false;
                ultra_signal = ultra_getSignal();
                uint8_t buf[2];
                buf[0] = (uint8_t)(ultra_signal >> 8);
                buf[1] = (uint8_t)(ultra_signal & 0xFF);
                server.transmitArrayUint8(buf, 2);
            }

            if (flag_read_mpu) {
                flag_read_mpu = false;
                uint8_t buf[12];
                for (int i = 0; i < 6; ++i) {
                    buf[2 * i]     = (uint8_t)(mpu_signals[i] >> 8);
                    buf[2 * i + 1] = (uint8_t)(mpu_signals[i] & 0xFF);
                }
                server.transmitArrayUint8(buf, 12);
            }

            if (flag_run_drive_forward) {
                flag_run_drive_forward = false;
                driveForward(speed);
            }

            if (flag_run_drive_backward) {
                flag_run_drive_backward = false;
                digitalWrite(MOTOR_OUT_1_PIN, LOW);
                digitalWrite(MOTOR_OUT_2_PIN, HIGH);
                analogWrite(MOTOR_PWM_PIN, speed);
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
                setSteerAngle(steer_angle);
            }
        }

        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

/* ======================= OPERATION TASK ======================= */
void OPERATION_process(void *parameter) {
    (void)parameter;

    speed       = 0;
    steer_angle = 95;
    if (!servo.attached()) servo.attach(SERVO_PIN);
    servo.write(steer_angle);

    for (;;) {
        if (robot_mode == OPERATION_MODE) {
            // Kick ultrasonic
            if ((tick_1ms - tick_kickUltra) >= TIME_KICK_ULTRA) {
                ultra_kick();
                tick_kickUltra = tick_1ms;
            }

            // Cập nhật tốc độ thô và lọc EMA/IIR
            if (speed_loop_tick) {
                speed_loop_tick = false;

                uint32_t now_ms = tick_1ms;
                uint32_t T_us;
                uint32_t last_ms;
                int8_t   dir;

                noInterrupts();
                T_us    = enc_period_us;
                last_ms = enc_last_pulse_ms;
                dir     = encoder_direction;
                interrupts();

                float rpm_raw = 0.0f;

                if (T_us > 0 && (now_ms - last_ms) <= ENCODER_TIMEOUT_MS) {
                    float pulses_per_rev = (float)ENCODER_PPR;
                    float rev_per_sec    = 1.0e6f / (pulses_per_rev * (float)T_us);
                    rpm_raw              = (dir >= 0 ? 1.0f : -1.0f) * 60.0f * rev_per_sec;
                } else {
                    rpm_raw = 0.0f;
                }

                Encoder::speed_filter_update(rpm_raw);
            }

            // Vòng vị trí sẽ thêm sau (position_loop_tick)

            if (flag_op_stop_motor) {
                stopMotor();
                flag_op_stop_motor = false;
            } else {
                digitalWrite(MOTOR_OUT_1_PIN, HIGH);
                digitalWrite(MOTOR_OUT_2_PIN, LOW);
                analogWrite(MOTOR_PWM_PIN, speed);
                setSteerAngle(steer_angle);
            }
        }

        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

/* ======================= HELPERS ======================= */
void driveForward(uint8_t spd) {
    digitalWrite(MOTOR_OUT_1_PIN, HIGH);
    digitalWrite(MOTOR_OUT_2_PIN, LOW);
    analogWrite(MOTOR_PWM_PIN, spd);
}

void stopMotor() {
    analogWrite(MOTOR_PWM_PIN, 0);
    digitalWrite(MOTOR_OUT_1_PIN, LOW);
    digitalWrite(MOTOR_OUT_2_PIN, LOW);
}

void setSteerAngle(uint16_t angle) {
    if (angle > 180) angle = 180;
    if (!servo.attached()) servo.attach(SERVO_PIN);
    servo.write(angle);
}
