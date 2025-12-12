#include <Arduino.h>
#include <ESP32Servo.h>
#include "bluetooth.hpp"
#include "sensors.hpp"
#include "actuators.hpp"

/* ======================= CONFIG ======================= */
// Thời gian gửi / nhận frame với PC (MATLAB) – chỉ dùng cho thiết kế giao thức
#define TIME_SEND_SIGNAL   50   // ms, phía PC nên gửi theo chu kỳ này

// Thời gian kích ultrasonic
#define TIME_KICK_ULTRA    60   // ms

// Chu kỳ vòng tốc độ & vị trí (có thể chỉnh)
#define TS_SPEED_MS        10   // ms, speed loop + encoder accumulation
#define TS_POSITION_MS     500  // ms, position loop (chưa dùng PID nhưng chuẩn bị sẵn)

/* ======================= PIN MAPPING ======================= */

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

// Encoder: channel A dùng đếm xung, channel B debug / direction
#define ENCODER_CHANNEL_A_PIN 33
#define ENCODER_CHANNEL_B_PIN 25
#define ENCODER_PPR           44   // pulses per revolution (edge A)

#define SERVO_PIN 23

#define DEBUG_ENCODER_ISR 0 // 1: hiện debug ISR, 0: tắt

/* ======================= ENCODER GLOBAL STATE ======================= */
// Tổng xung (cho quãng đường + tính tốc độ)
volatile int32_t encoder_pulse_total = 0;
// Xung trong một frame (gửi về MATLAB)
static volatile int32_t encoder_pulse_frame = 0;

/* ======================= ENCODER NAMESPACE ======================= */

namespace Encoder
{
    // Giới hạn cửa sổ theo số lần gọi speed_update (frame)
    static constexpr uint8_t WIN_MIN = 1;
    static constexpr uint8_t WIN_MAX = 32;

    // Cấu hình
    static uint16_t g_encoderPPR      = 1;      // pulses per rev
    static uint16_t g_dt_speed_ms     = 50;     // chu kỳ gọi speed_update (ms)
    static uint16_t g_accum_window_ms = 50;     // thời gian accumulation (ms)
    static uint8_t  g_accum_N         = 1;      // số lần gọi update trong 1 cửa sổ

    // IIR filter hệ số
    static float    g_alpha           = 0.2f;   // 0..1

    // Trạng thái nội bộ
    static int32_t  g_lastCount       = 0;      // count lần trước
    static int32_t  g_acc_delta       = 0;      // tổng delta trong cửa sổ
    static uint8_t  g_acc_count       = 0;      // số mẫu đã tích lũy
    static float    g_rpm_filt        = 0.0f;   // tốc độ đã lọc

    // Khởi tạo: gọi một lần trong setup
    void speed_init(uint16_t encoderPPR,
                    uint16_t dt_speed_ms,
                    uint16_t accum_window_ms,
                    float    alpha)
    {
        if (encoderPPR == 0) encoderPPR = 1;
        g_encoderPPR      = encoderPPR;

        if (dt_speed_ms == 0) dt_speed_ms = 1;
        g_dt_speed_ms     = dt_speed_ms;

        if (accum_window_ms < dt_speed_ms) accum_window_ms = dt_speed_ms;
        g_accum_window_ms = accum_window_ms;

        uint16_t N = g_accum_window_ms / g_dt_speed_ms;
        if (N < WIN_MIN) N = WIN_MIN;
        if (N > WIN_MAX) N = WIN_MAX;
        g_accum_N = (uint8_t)N;

        if (alpha < 0.0f) alpha = 0.0f;
        if (alpha > 1.0f) alpha = 1.0f;
        g_alpha      = alpha;

        g_lastCount  = 0;
        g_acc_delta  = 0;
        g_acc_count  = 0;
        g_rpm_filt   = 0.0f;
    }

    // Đặt lại cửa sổ accumulation (đơn vị: số frame speed_update)
    void speed_setFilterWindow(uint8_t N)
    {
        if (N < WIN_MIN) N = WIN_MIN;
        if (N > WIN_MAX) N = WIN_MAX;

        g_accum_N         = N;
        g_accum_window_ms = g_dt_speed_ms * (uint16_t)g_accum_N;

        g_acc_delta = 0;
        g_acc_count = 0;
        g_rpm_filt  = 0.0f;
    }

    uint8_t speed_getFilterWindow()
    {
        return g_accum_N;
    }

    // Gọi đúng chu kỳ g_dt_speed_ms (ví dụ TS_SPEED_MS)
    void speed_update()
    {
        int32_t total_now;

        // Đọc encoder_pulse_total an toàn
        noInterrupts();
        total_now = encoder_pulse_total;
        interrupts();

        int32_t delta = total_now - g_lastCount;
        g_lastCount   = total_now;

        g_acc_delta += delta;
        g_acc_count++;

        if (g_acc_count >= g_accum_N)
        {
            float window_s = (float)g_accum_window_ms / 1000.0f;
            if (window_s <= 0.0f) window_s = (float)g_dt_speed_ms / 1000.0f;

            float pulses_per_s = (float)g_acc_delta / window_s;
            float rev_per_s    = pulses_per_s / (float)g_encoderPPR;
            float rpm_raw      = rev_per_s * 60.0f;
            if (rpm_raw < 0.0f) rpm_raw = 0.0f;

            // IIR smoothing
            g_rpm_filt = g_alpha * rpm_raw + (1.0f - g_alpha) * g_rpm_filt;

            g_acc_delta = 0;
            g_acc_count = 0;
        }
    }

    float speed_get_rpm()
    {
        return g_rpm_filt;
    }
} // namespace Encoder

/* ======================= SENSORS ======================= */
uint8_t* line_signals = nullptr; // 5 bytes
uint16_t ultra_signal = 0;
uint16_t mpu_signals[6] = {0};

/* ======================= ACTUATORS (đơn giản) ======================= */
Servo servo;

/* ======================= COMMUNICATION ======================= */
Network server(5946);

static uint8_t  cmd         = 0x00;
static uint8_t* payload     = nullptr;
static uint8_t  speed       = 0;
static uint16_t steer_angle = 75;

/* ======================= FLAGS & TIMERS ======================= */

static volatile bool flag_read_line          = false;
static volatile bool flag_read_ultra         = false;
static volatile bool flag_read_mpu           = false;
static volatile bool flag_run_drive_forward  = false;
static volatile bool flag_run_drive_backward = false;
static volatile bool flag_run_drive_stop     = false;
static volatile bool flag_disable_steer      = false;
static volatile bool flag_turn_steer         = false;
static volatile bool flag_op_stop_motor      = false;

// Tick 1 ms chung
static volatile uint32_t tick_1ms            = 0;

// Tick cho ultrasonic
static volatile uint32_t tick_kickUltra      = 0;

// Flag cho speed loop và position loop
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

/* ======================= ENCODER ISR PROTOTYPES ======================= */
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

    /* -------- Sensors setup -------- */
    line_setup(LINE_SENSOR_IDX_1_PIN,
               LINE_SENSOR_IDX_2_PIN,
               LINE_SENSOR_IDX_3_PIN,
               LINE_SENSOR_IDX_4_PIN,
               LINE_SENSOR_IDX_5_PIN);

    ultra_setup(ULTRASONIC_TRIG_PIN, ULTRASONIC_ECHO_PIN);
    attachInterrupt(ULTRASONIC_ECHO_PIN, hanlder_ultra_echo, CHANGE);

    /* -------- Actuators setup -------- */
    pinMode(MOTOR_OUT_1_PIN, OUTPUT);
    pinMode(MOTOR_OUT_2_PIN, OUTPUT);
    pinMode(MOTOR_PWM_PIN, OUTPUT);

    servo.attach(SERVO_PIN);
    servo.write(steer_angle); // center

    stopMotor();

    /* -------- Encoder setup (A & B) -------- */
    pinMode(ENCODER_CHANNEL_A_PIN, INPUT_PULLUP);
    pinMode(ENCODER_CHANNEL_B_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER_CHANNEL_A_PIN), ISR_encoder_A, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_CHANNEL_B_PIN), ISR_encoder_B, CHANGE);
    Serial.println("[SETUP] Encoder interrupts attached");

    // Khởi tạo khối Encoder: accumulation window = TS_SPEED_MS, alpha = 0.2
    Encoder::speed_init(ENCODER_PPR, TS_SPEED_MS, TS_SPEED_MS, 0.2f);
    // Cửa sổ filter = 1 frame (có thể đổi bằng cmd 0xEC)
    Encoder::speed_setFilterWindow(1);

    /* -------- Network setup -------- */
    Serial.println("[SETUP] Start to configurate network");
    while (!server.begin()) {
        Serial.println("[SETUP] Configuration failed");
        delay(1000);
    }
    Serial.println("[SETUP] Network configured");

    /* -------- Timer interrupt setup: 1 ms tick -------- */
    Timer0_Cfg = timerBegin(0, 80, true); // 80 MHz / 80 = 1 MHz
    timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);
    timerAlarmWrite(Timer0_Cfg, 1000, true); // 1000 ticks = 1 ms
    timerAlarmEnable(Timer0_Cfg);

    robot_mode = IDLE_MODE;
    Serial.println("[SETUP] Robot in IDLE_MODE");

    /* -------- RTOS tasks -------- */
    xTaskCreatePinnedToCore(
        com_process,
        "com_process",
        10000,
        NULL,
        1,
        &COM_handle,
        0
    );

    xTaskCreatePinnedToCore(
        IDLE_process,
        "IDLE_process",
        10000,
        NULL,
        1,
        &IDLE_handle,
        1
    );

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

void loop() {
    // FreeRTOS xử lý
}

/* ======================= TIMER ISR ======================= */

void IRAM_ATTR Timer0_ISR() {
    tick_1ms++;

    static uint16_t cnt_speed = 0;
    static uint16_t cnt_pos   = 0;

    // Speed loop tick
    cnt_speed++;
    if (cnt_speed >= TS_SPEED_MS) {
        cnt_speed = 0;
        speed_loop_tick = true;
    }

    // Position loop tick
    cnt_pos++;
    if (cnt_pos >= TS_POSITION_MS) {
        cnt_pos = 0;
        position_loop_tick = true;
    }
}

/* ======================= ENCODER ISRs ======================= */

void IRAM_ATTR ISR_encoder_A() {
    encoder_pulse_total++;
    encoder_pulse_frame++;

#if DEBUG_ENCODER_ISR
    static uint32_t lastPrintA = 0;
    uint32_t now = micros();
    if (now - lastPrintA > 10000) // ~10 ms
    {
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
    if (now - lastPrintB > 20000) // ~20 ms
    {
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
                case 0xFF: // IDLE -> OPERATION
                    robot_mode = OPERATION_MODE;
                    Serial.println("[MODE] Switch to OPERATION_MODE");
                    break;

                case 0xFE: // IDLE stay
                    robot_mode = IDLE_MODE;
                    Serial.println("[MODE] Stay in IDLE_MODE");
                    break;

                /* -------- Sensor Commands -------- */
                case 0xEF:
                    flag_read_line = true;
                    break;

                case 0xEE:
                    flag_read_ultra = true;
                    break;

                case 0xED:
                    flag_read_mpu = true;
                    break;

                /* -------- Drive / Servo Commands -------- */
                // [0xDF] [speed] : forward
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

                // [0xDE] [speed] : backward
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

                case 0xDD: // stop
                    flag_run_drive_stop = true;
                    Serial.println("[IDLE CMD] STOP");
                    break;

                case 0xDC: // disable steer
                    flag_disable_steer = true;
                    break;

                // [0xDB] [angle_hi] [angle_lo]
                case 0xDB: {
                    uint8_t hi, lo;
                    if (server.getUint8(hi) && server.getUint8(lo)) {
                        steer_angle =
                            (uint16_t(hi) << 8) | uint16_t(lo);
                        flag_turn_steer = true;
                        Serial.print("[IDLE CMD] STEER=");
                        Serial.println(steer_angle);
                    } else ok = false;
                    break;
                }

                // [0xEC] [N] : set accumulation window (số frame speed_update)
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
                    server.transmitUint8(0x20); // ACK
                    Serial.print("[IDLE] Data receive at tick=");
                    Serial.println(tick_1ms);
                }
            }
            break;
        }

        case OPERATION_MODE: {
            // Trong OPERATION, PC gửi 4 byte control -> ESP trả lại 22 byte
            uint8_t ctrl[4];
            if (server.getArrayUint8(ctrl, 4)) {
                uint8_t c    = ctrl[0];
                uint8_t spd  = ctrl[1];
                uint16_t ang = (uint16_t(ctrl[2]) << 8) | uint16_t(ctrl[3]);

                switch (c) {
                case 0xF0: // stop
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

                // ======= LẤY XUNG FRAME + RESET =======
                int32_t pulses;
                noInterrupts();
                pulses = encoder_pulse_frame;
                encoder_pulse_frame = 0;
                interrupts();

                // ======= ĐỌC SENSOR =======
                line_signals = line_readSignals();
                ultra_signal = ultra_getSignal();

                // ======= LẤY TỐC ĐỘ ĐÃ LỌC =======
                float spd_f = Encoder::speed_get_rpm();

                // ======= BUILD FRAME 22 BYTE =======
                const uint8_t FRAME_LEN = 22;
                uint8_t frame[FRAME_LEN] = {0};

                // line sensors
                for (int i = 0; i < 5; ++i) {
                    frame[i] = line_signals ? line_signals[i] : 0;
                }

                // ultrasonic
                frame[5] = (uint8_t)(ultra_signal >> 8);
                frame[6] = (uint8_t)(ultra_signal & 0xFF);

                // encoder pulses per frame (0..255)
                if (pulses < 0)   pulses = 0;
                if (pulses > 255) pulses = 255;
                frame[19] = (uint8_t)pulses;

                // encoder speed rpm (uint16 big-endian)
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
                if (!servo.attached()) servo.attach(SERVO_PIN);
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
            // Kick ultrasonic định kỳ
            if ((tick_1ms - tick_kickUltra) >= TIME_KICK_ULTRA) {
                ultra_kick();
                tick_kickUltra = tick_1ms;
            }

            // Speed loop: cập nhật tốc độ encoder (accumulation + IIR)
            if (speed_loop_tick) {
                speed_loop_tick = false;
                Encoder::speed_update();
                // Tại đây sau này thêm PID tốc độ nếu cần
            }

            // Position loop: chưa dùng, sau này thêm PID vị trí
            if (position_loop_tick) {
                position_loop_tick = false;
                // TODO: PID vị trí
            }

            // Motor & steering
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
