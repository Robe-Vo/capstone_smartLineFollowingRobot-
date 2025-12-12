#include <Arduino.h>
#include <ESP32Servo.h>
#include "bluetooth.hpp"
#include "sensors.hpp"
#include "actuators.hpp"

/* ======================= CONFIG ======================= */
#define TIME_SEND_SIGNAL   50   // ms, chu kỳ frame PC
#define TIME_KICK_ULTRA    60   // ms

#define TS_SPEED_MS        50   // ms, speed loop (dùng period measurement)
#define TS_POSITION_MS     500  // ms, position loop (chưa dùng PID)

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
#define ENCODER_PPR           44   // pulses / rev đã nhân 4 cạnh

#define SERVO_PIN 23

#define DEBUG_ENCODER_ISR 0

/* ======================= ENCODER GLOBAL STATE ======================= */
// Tổng xung (dùng cho quãng đường, frame)
volatile int32_t encoder_pulse_total  = 0;
static volatile int32_t encoder_pulse_frame = 0;

// Period measurement
static volatile uint32_t enc_last_edge_us   = 0;   // thời điểm cạnh trước
static volatile uint32_t enc_period_us      = 0;   // period mới nhất
static volatile uint32_t enc_last_pulse_ms  = 0;   // tick_1ms tại cạnh gần nhất
static volatile bool     enc_period_valid   = false;

/* ======================= ENCODER NAMESPACE ======================= */
namespace Encoder
{
    // Cấu hình
    static uint16_t g_encoderPPR   = 44;
    static float    g_alpha        = 0.3f;   // IIR
    static uint32_t g_timeout_ms   = 200;    // nếu quá lâu không có xung -> rpm=0
    static uint8_t  g_filterN      = 1;      // N logic cho alpha

    // Trạng thái
    static float    g_rpm_filt     = 0.0f;

    void speed_init(uint16_t ppr, float alpha, uint32_t timeout_ms)
    {
        if (ppr == 0) ppr = 1;
        g_encoderPPR = ppr;

        if (alpha < 0.0f) alpha = 0.0f;
        if (alpha > 1.0f) alpha = 1.0f;
        g_alpha = alpha;

        if (timeout_ms < 10) timeout_ms = 10;
        g_timeout_ms = timeout_ms;

        g_filterN  = 1;
        g_rpm_filt = 0.0f;
    }

    // N dùng để chỉnh alpha đơn giản: alpha = 1/N
    void speed_setFilterWindow(uint8_t N)
    {
        if (N == 0) N = 1;
        if (N > 32) N = 32;
        g_filterN = N;
        g_alpha   = 1.0f / (float)N;
    }

    uint8_t speed_getFilterWindow()
    {
        return g_filterN;
    }

    // Cập nhật tốc độ theo period; now_ms = tick_1ms hiện tại
    void speed_update_period(uint32_t now_ms)
    {
        uint32_t T_us;
        bool     valid;
        uint32_t lastPulseMs;

        noInterrupts();
        T_us         = enc_period_us;
        valid        = enc_period_valid;
        enc_period_valid = false;        // đã dùng period này
        lastPulseMs  = enc_last_pulse_ms;
        interrupts();

        float rpm_raw = g_rpm_filt;

        if (valid && T_us > 0)
        {
            rpm_raw = (60.0f * 1e6f) / ((float)g_encoderPPR * (float)T_us);
        }
        else
        {
            uint32_t dt_ms = now_ms - lastPulseMs;
            if (dt_ms > g_timeout_ms)
                rpm_raw = 0.0f;          // coi như đã dừng
            // nếu chưa timeout: giữ rpm_raw cũ
        }

        g_rpm_filt = g_alpha * rpm_raw + (1.0f - g_alpha) * g_rpm_filt;
    }

    float speed_get_rpm()
    {
        return g_rpm_filt;
    }
} // namespace Encoder

/* ======================= SENSORS ======================= */
uint8_t* line_signals = nullptr;
uint16_t ultra_signal = 0;
uint16_t mpu_signals[6] = {0};

/* ======================= ACTUATORS ======================= */
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
void IRAM_ATTR ISR_encoder_A();
void IRAM_ATTR ISR_encoder_B();

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

    line_setup(LINE_SENSOR_IDX_1_PIN,
               LINE_SENSOR_IDX_2_PIN,
               LINE_SENSOR_IDX_3_PIN,
               LINE_SENSOR_IDX_4_PIN,
               LINE_SENSOR_IDX_5_PIN);

    ultra_setup(ULTRASONIC_TRIG_PIN, ULTRASONIC_ECHO_PIN);
    attachInterrupt(ULTRASONIC_ECHO_PIN, hanlder_ultra_echo, CHANGE);

    pinMode(MOTOR_OUT_1_PIN, OUTPUT);
    pinMode(MOTOR_OUT_2_PIN, OUTPUT);
    pinMode(MOTOR_PWM_PIN, OUTPUT);

    servo.attach(SERVO_PIN);
    servo.write(steer_angle);
    stopMotor();

    pinMode(ENCODER_CHANNEL_A_PIN, INPUT_PULLUP);
    pinMode(ENCODER_CHANNEL_B_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER_CHANNEL_A_PIN), ISR_encoder_A, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_CHANNEL_B_PIN), ISR_encoder_B, CHANGE);
    Serial.println("[SETUP] Encoder interrupts attached");

    Encoder::speed_init(ENCODER_PPR, 0.3f, 200);   // alpha, timeout
    Encoder::speed_setFilterWindow(3);             // alpha = 1/3 mặc định

    Serial.println("[SETUP] Start to configurate network");
    while (!server.begin()) {
        Serial.println("[SETUP] Configuration failed");
        delay(1000);
    }
    Serial.println("[SETUP] Network configured");

    Timer0_Cfg = timerBegin(0, 80, true); // 1 MHz
    timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);
    timerAlarmWrite(Timer0_Cfg, 1000, true); // 1 ms
    timerAlarmEnable(Timer0_Cfg);

    robot_mode = IDLE_MODE;
    Serial.println("[SETUP] Robot in IDLE_MODE");

    xTaskCreatePinnedToCore(com_process, "com_process", 10000, NULL, 1, &COM_handle, 0);
    xTaskCreatePinnedToCore(IDLE_process,"IDLE_process",10000,NULL,1,&IDLE_handle,1);
    xTaskCreatePinnedToCore(OPERATION_process,"OPERATION_process",10000,NULL,1,&OPERATION_handle,1);
}

void loop() {
    // dùng FreeRTOS
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

/* ======================= ENCODER ISRs ======================= */
void IRAM_ATTR ISR_encoder_A() {
    uint32_t now = micros();
    uint32_t dt  = now - enc_last_edge_us;
    enc_last_edge_us = now;

    encoder_pulse_total++;
    encoder_pulse_frame++;

    if (dt > 50) {                  // bỏ glitch < 50 us
        enc_period_us     = dt;
        enc_period_valid  = true;
        enc_last_pulse_ms = tick_1ms;
    }

#if DEBUG_ENCODER_ISR
    static uint32_t lastPrintA = 0;
    if (now - lastPrintA > 10000) {
        ets_printf("[ENC A] total=%d frame=%d\n",
                   (int)encoder_pulse_total,
                   (int)encoder_pulse_frame);
        lastPrintA = now;
    }
#endif
}

void IRAM_ATTR ISR_encoder_B() {
#if DEBUG_ENCODER_ISR
    static uint32_t lastPrintB = 0;
    uint32_t now = micros();
    if (now - lastPrintB > 20000) {
        int levelA = gpio_get_level((gpio_num_t)ENCODER_CHANNEL_A_PIN);
        int levelB = gpio_get_level((gpio_num_t)ENCODER_CHANNEL_B_PIN);
        ets_printf("[ENC B] A=%d B=%d\n", levelA, levelB);
        lastPrintB = now;
    }
#endif
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
                case 0xFF:
                    robot_mode = OPERATION_MODE;
                    Serial.println("[MODE] Switch to OPERATION_MODE");
                    break;

                case 0xFE:
                    robot_mode = IDLE_MODE;
                    Serial.println("[MODE] Stay in IDLE_MODE");
                    break;

                case 0xEF:
                    flag_read_line = true;
                    break;

                case 0xEE:
                    flag_read_ultra = true;
                    break;

                case 0xED:
                    flag_read_mpu = true;
                    break;

                case 0xDF: {
                    uint8_t spd;
                    if (server.getUint8(spd)) {
                        speed = spd;
                        flag_run_drive_forward = true;
                    } else ok = false;
                    break;
                }

                case 0xDE: {
                    uint8_t spd;
                    if (server.getUint8(spd)) {
                        speed = spd;
                        flag_run_drive_backward = true;
                    } else ok = false;
                    break;
                }

                case 0xDD:
                    flag_run_drive_stop = true;
                    break;

                case 0xDC:
                    flag_disable_steer = true;
                    break;

                case 0xDB: {
                    uint8_t hi, lo;
                    if (server.getUint8(hi) && server.getUint8(lo)) {
                        steer_angle =
                            (uint16_t(hi) << 8) | uint16_t(lo);
                        flag_turn_steer = true;
                    } else ok = false;
                    break;
                }

                // 0xEC: chỉnh N của IIR (alpha=1/N)
                case 0xEC: {
                    uint8_t win;
                    if (server.getUint8(win)) {
                        Encoder::speed_setFilterWindow(win);
                        Serial.print("[IDLE CMD] ENC filter N=");
                        Serial.println(Encoder::speed_getFilterWindow());
                    } else ok = false;
                    break;
                }

                case 0xA0: {
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

                if (ok) {
                    server.transmitUint8(0x20);
                }
            }
            break;
        }

        case OPERATION_MODE: {
            uint8_t ctrl[4];
            if (server.getArrayUint8(ctrl, 4)) {
                uint8_t c    = ctrl[0];
                uint8_t spd  = ctrl[1];
                uint16_t ang = (uint16_t(ctrl[2]) << 8) | uint16_t(ctrl[3]);

                switch (c) {
                case 0xF0:
                    speed = 0;
                    flag_op_stop_motor = true;
                    break;

                case 0xF1:
                    speed       = spd;
                    steer_angle = ang;
                    break;

                case 0xFE:
                    robot_mode = IDLE_MODE;
                    server.transmitUint8(0x20);
                    stopMotor();
                    break;

                default:
                    break;
                }

                int32_t pulses;
                noInterrupts();
                pulses = encoder_pulse_frame;
                encoder_pulse_frame = 0;
                interrupts();

                line_signals = line_readSignals();
                ultra_signal = ultra_getSignal();

                float spd_f = Encoder::speed_get_rpm();

                const uint8_t FRAME_LEN = 22;
                uint8_t frame[FRAME_LEN] = {0};

                for (int i = 0; i < 5; ++i) {
                    frame[i] = line_signals ? line_signals[i] : 0;
                }

                frame[5] = (uint8_t)(ultra_signal >> 8);
                frame[6] = (uint8_t)(ultra_signal & 0xFF);

                if (pulses < 0)   pulses = 0;
                if (pulses > 255) pulses = 255;
                frame[19] = (uint8_t)pulses;

                if (spd_f < 0.0f)      spd_f = 0.0f;
                if (spd_f > 65535.0f)  spd_f = 65535.0f;
                uint16_t spd_q = (uint16_t)spd_f;
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
            if ((tick_1ms - tick_kickUltra) >= TIME_KICK_ULTRA) {
                ultra_kick();
                tick_kickUltra = tick_1ms;
            }

            if (speed_loop_tick) {
                speed_loop_tick = false;
                Encoder::speed_update_period(tick_1ms);
                // sau này thêm PID tốc độ
            }

            if (position_loop_tick) {
                position_loop_tick = false;
                // sau này thêm PID vị trí
            }

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
