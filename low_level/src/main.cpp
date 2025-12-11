#include <Arduino.h>
#include <ESP32Servo.h>

#include "bluetooth.hpp"
#include "sensors.hpp"

/* ======================= CONFIG ======================= */

#define TIME_SEND_SIGNAL       50   // ms, dùng cho tính tốc độ
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

// Encoder: channel A dùng đếm xung, channel B debug / direction
#define ENCODER_CHANNEL_A_PIN  33
#define ENCODER_CHANNEL_B_PIN  25
#define ENCODER_PPR            44      // pulses per revolution (edge A)

#define SERVO_PIN              23

#define DEBUG_ENCODER_ISR      1       // 1: hiện debug ISR, 0: tắt

/* ======================= ENCODER STATE ======================= */

// Tổng xung (cho quãng đường)
static volatile int32_t encoder_pulse_total = 0;

// Xung trong một frame (dùng tính tốc độ, gửi về MATLAB)
static volatile int32_t encoder_pulse_frame = 0;

// Tốc độ tính được (rpm)
static volatile float   encoder_speed = 0.0f;

/* ======================= SENSORS ======================= */

uint8_t*  line_signals = nullptr;  // 5 bytes
uint16_t  ultra_signal = 0;
uint16_t  mpu_signals[6] = {0};

/* ======================= ACTUATORS (đơn giản) ======================= */

Servo servo;

/* ======================= COMMUNICATION ======================= */

Network server(5946);
static uint8_t  cmd         = 0x00;
static uint8_t* payload     = nullptr;

static uint8_t  speed       = 0;
static uint16_t steer_angle = 75;

static volatile bool flag_read_line          = false;
static volatile bool flag_read_ultra         = false;
static volatile bool flag_read_mpu           = false;
static volatile bool flag_run_drive_forward  = false; 
static volatile bool flag_run_drive_backward = false;
static volatile bool flag_run_drive_stop     = false;
static volatile bool flag_disable_steer      = false;
static volatile bool flag_turn_steer         = false;

static volatile bool flag_op_stop_motor      = false;

static volatile unsigned long tick_1ms       = 0;
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
void driveForward(uint8_t spd);
void stopMotor();
void setSteerAngle(uint16_t angle);

/* ======================= SETUP ======================= */

void setup()
{
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
    pinMode(MOTOR_PWM_PIN,   OUTPUT);

    servo.attach(SERVO_PIN);
    servo.write(steer_angle);    // center
    stopMotor();

    /* -------- Encoder setup (A & B) -------- */
    pinMode(ENCODER_CHANNEL_A_PIN, INPUT_PULLUP);
    pinMode(ENCODER_CHANNEL_B_PIN, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENCODER_CHANNEL_A_PIN),
                    ISR_encoder_A,
                    RISING);

    attachInterrupt(digitalPinToInterrupt(ENCODER_CHANNEL_B_PIN),
                    ISR_encoder_B,
                    CHANGE);

    Serial.println("[SETUP] Encoder interrupts attached");

    /* -------- Network setup -------- */
    Serial.println("[SETUP] Start to configurate network");
    while (!server.begin())
    {
        Serial.println("[SETUP] Configuration failed");
        delay(1000);
    }
    Serial.println("[SETUP] Network configured");

    /* -------- Timer interrupt setup: 1 ms tick -------- */
    Timer0_Cfg = timerBegin(0, 80, true);      // 80 MHz / 80 = 1 MHz
    timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);
    timerAlarmWrite(Timer0_Cfg, 1000, true);   // 1000 ticks = 1 ms
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

void loop()
{
    // FreeRTOS xử lý
}

/* ======================= TIMER ISR ======================= */

void IRAM_ATTR Timer0_ISR()
{
    tick_1ms++;
}

/* ======================= ENCODER ISRs ======================= */

void IRAM_ATTR ISR_encoder_A()
{
    encoder_pulse_total++;
    encoder_pulse_frame++;

#if DEBUG_ENCODER_ISR
    static uint32_t lastPrintA = 0;
    uint32_t now = micros();
    if (now - lastPrintA > 10000)  // ~10 ms
    {
        ets_printf("[ENC A] total=%d frame=%d\n",
                   (int)encoder_pulse_total,
                   (int)encoder_pulse_frame);
        lastPrintA = now;
    }
#endif
}

void IRAM_ATTR ISR_encoder_B()
{
#if DEBUG_ENCODER_ISR
    static uint32_t lastPrintB = 0;
    uint32_t now = micros();
    if (now - lastPrintB > 20000)  // ~20 ms
    {
        int levelA = gpio_get_level((gpio_num_t)ENCODER_CHANNEL_A_PIN);
        int levelB = gpio_get_level((gpio_num_t)ENCODER_CHANNEL_B_PIN);
        ets_printf("[ENC B] A=%d B=%d\n", levelA, levelB);
        lastPrintB = now;
    }
#endif
}

/* ======================= COMMUNICATION TASK ======================= */

void com_process(void *parameter)
{
    (void)parameter;

    // dùng cho tính dt trong OPERATION
    static uint32_t last_frame_ms = 0;

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
                        case 0xFF:   // IDLE -> OPERATION
                            robot_mode = OPERATION_MODE;
                            last_frame_ms = tick_1ms;
                            Serial.println("[MODE] Switch to OPERATION_MODE");
                            break;

                        case 0xFE:   // IDLE stay
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
                        case 0xDF:
                        {
                            uint8_t spd;
                            if (server.getUint8(spd))
                            {
                                speed = spd;
                                flag_run_drive_forward = true;
                                Serial.print("[IDLE CMD] FWD speed=");
                                Serial.println(speed);
                            }
                            else ok = false;
                            break;
                        }

                        // [0xDE] [speed] : backward
                        case 0xDE:
                        {
                            uint8_t spd;
                            if (server.getUint8(spd))
                            {
                                speed = spd;
                                flag_run_drive_backward = true;
                                Serial.print("[IDLE CMD] REV speed=");
                                Serial.println(speed);
                            }
                            else ok = false;
                            break;
                        }

                        case 0xDD:    // stop
                            flag_run_drive_stop = true;
                            Serial.println("[IDLE CMD] STOP");
                            break;

                        case 0xDC:    // disable steer
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
                                Serial.print("[IDLE CMD] STEER=");
                                Serial.println(steer_angle);
                            }
                            else ok = false;
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
                        Serial.print("[IDLE] Data receive at tick=");
                        Serial.println(tick_1ms);
                    }
                }
                break;
            }

            case OPERATION_MODE:
            {
                /* Mỗi lần MATLAB gửi 4 byte control -> ESP32 xử lý và gửi lại 22 byte */

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
                            Serial.println("[OPE CMD] STOP");
                            break;

                        case 0xF1: // forward + steering
                            speed       = spd;
                            steer_angle = angle;
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

                    // ======= LẤY XUNG + RESET =======
                    int32_t pulses;
                    noInterrupts();
                    pulses = encoder_pulse_frame;
                    encoder_pulse_frame = 0;
                    interrupts();

                    // ======= TÍNH TỐC ĐỘ RPM =======
                    uint32_t now_ms = tick_1ms;
                    uint32_t dt_ms  = (last_frame_ms == 0)
                                      ? TIME_SEND_SIGNAL
                                      : (now_ms - last_frame_ms);
                    if (dt_ms == 0) dt_ms = TIME_SEND_SIGNAL;
                    last_frame_ms = now_ms;

                    float pulses_f     = (float)pulses;
                    float pulses_per_s = pulses_f * (1000.0f / (float)dt_ms);
                    float rev_per_s    = pulses_per_s / (float)ENCODER_PPR;
                    float rpm          = rev_per_s * 60.0f;
                    if (rpm < 0.0f) rpm = 0.0f;
                    encoder_speed = rpm;

                    Serial.print("[OPE FRAME] pulses=");
                    Serial.print(pulses);
                    Serial.print(" dt_ms=");
                    Serial.print(dt_ms);
                    Serial.print(" rpm=");
                    Serial.println(encoder_speed);

                    // ======= ĐỌC SENSOR =======
                    line_signals = line_readSignals();
                    ultra_signal = ultra_getSignal();

                    // ======= BUILD FRAME 22 BYTE =======
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

                    // encoder pulses per frame (0..255)
                    if (pulses < 0)   pulses = 0;
                    if (pulses > 255) pulses = 255;
                    frame[19] = (uint8_t)pulses;

                    // encoder speed rpm (uint16 big-endian)
                    float    spd_f = encoder_speed;
                    if (spd_f < 0.0f)     spd_f = 0.0f;
                    if (spd_f > 65535.0f) spd_f = 65535.0f;
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

void IDLE_process(void *parameter)
{
    (void)parameter;

    for (;;)
    {
        if (robot_mode == IDLE_MODE)
        {
            if (flag_read_line)
            {
                flag_read_line = false;
                line_signals = line_readSignals();
                if (line_signals != nullptr)
                {
                    server.transmitArrayUint8(line_signals, 5);
                }
            }

            if (flag_read_ultra)
            {
                flag_read_ultra = false;

                ultra_signal = ultra_getSignal();

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

            if (flag_run_drive_forward)
            {
                flag_run_drive_forward = false;
                driveForward(speed);
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

            if (flag_disable_steer)
            {
                flag_disable_steer = false;
                if (servo.attached()) servo.detach();
            }

            if (flag_turn_steer)
            {
                flag_turn_steer = false;
                if (!servo.attached()) servo.attach(SERVO_PIN);
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
    servo.write(steer_angle);

    for (;;)
    {
        if (robot_mode == OPERATION_MODE)
        {
            // Kick ultrasonic định kỳ
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

                uint16_t angle = steer_angle;
                if (angle > 180) angle = 180;
                servo.write(angle);
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
