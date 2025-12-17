#include "app.hpp"

#include <Arduino.h>
#include <ESP32Servo.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "bluetooth.hpp"
#include "actuators.hpp"
#include "sensors.hpp"

#include "pin.hpp"
#include "cfg.hpp"

#include "appcore/AppState.hpp"
#include "proto/CmdParser.hpp"
#include "control/Control.hpp"
#include "telemetry/telemetry.hpp"
#include "sched/TickScheduler.hpp"

// ===== Globals (ít, có ý nghĩa) =====
static Network network;
static Servo   servo;

static AppState    state;
static ControlOut  control;

static TickScheduler flag;

// ===== Task prototypes (ngắn) =====
static void handle_communication(void*);
static void handle_control(void*);
static void handle_communication(void*);
static void handle_ultra(void*);

// ===== Setup =====
void App_setup()
{
  Serial.begin(115200);

  g_net.begin();

  // HW init (pin/cfg)
  line_setup(LINE_SENSOR_IDX_1_PIN, LINE_SENSOR_IDX_2_PIN, LINE_SENSOR_IDX_3_PIN,
             LINE_SENSOR_IDX_4_PIN, LINE_SENSOR_IDX_5_PIN);

  ultra_setup(ULTRASONIC_TRIG_PIN, ULTRASONIC_ECHO_PIN);
  attachInterrupt(ULTRASONIC_ECHO_PIN, hanlder_ultra_echo, CHANGE);

  MotorPWM11::begin(MOTOR_PWM_PIN, MOTOR_OUT_1_PIN, MOTOR_OUT_2_PIN, PWM_CH_MOTOR, PWM_FREQ_HZ);

  pinMode(ENCODER_CHANNEL_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_CHANNEL_B_PIN, INPUT_PULLUP);
  attachInterrupt(ENCODER_CHANNEL_A_PIN, Encoder::isr_encoder_AB, CHANGE);
  attachInterrupt(ENCODER_CHANNEL_B_PIN, Encoder::isr_encoder_AB, CHANGE);

  servo.setPeriodHertz(50);
  servo.attach(SERVO_PIN);

  // state init
  AppState_init(state);
  Control_init(servo);
  Control_stop(control);

  // tasks
  xTaskCreatePinnedToCore(handle_communication,    "comm",  4096, nullptr, 3, &flag.hComm, 0);
  xTaskCreatePinnedToCore(handle_control, "ctrl",  4096, nullptr, 2, &flag.hCtrl, 0);
  xTaskCreatePinnedToCore(Task_Motor,   "motor", 4096, nullptr, 4, nullptr, 1);
  xTaskCreatePinnedToCore(handle_ultra,   "ultra", 2048, nullptr, 1, &flag.hUltra, 1);

  // scheduler timer
  TickScheduler_begin(flag);
}

void App_loop()
{
  vTaskDelay(pdMS_TO_TICKS(1));
}

// ===== COMM task: RX + dispatch + telemetry =====
static void handle_communication(void*)
{
  for(;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // timeout safety
    if (AppState_isTimedOut(g_state, millis(), OP_TIMEOUT_MS)) {
      AppState_setMode(g_state, Mode::IDLE);
      g_state.op_enable = false;
      Control_stop(g_u);
      // notify motor (event-driven) bằng cơ chế bạn chọn (notify/queue)
    }

    // RX drain
    Cmd c{};
    while (CmdParser_tryRead(g_net, c)) {
      AppState_onRx(g_state, millis());

      // IDLE ACK policy
      if (g_state.mode == Mode::IDLE) g_net.transmitUint8(0x20);

    }

    // Telemetry policy (ví dụ chỉ OPERATION)
    if (g_state.mode == Mode::OPERATION) {
      uint8_t* line = line_readSignals();
      uint16_t ultra = ultra_getSignal();
      int16_t mpu6[6] = {0};
      int32_t enc_total = Encoder::encoder_get_total();
      int8_t enc_delta = (int8_t)(enc_total); // thay bằng delta logic bạn có sẵn
      int16_t speed_i16 = (int16_t)(Encoder::speed_get_hz_out() * 10.0f);

      auto fr = TelemetryOp22::pack(line, ultra, mpu6, enc_delta, speed_i16);
      TelemetryOp22::send(g_net, fr);
    }
  }
}

// ===== CONTROL task: PID / update setpoints =====
static void Task_Control(void*)
{
  for(;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    if (g_state.mode != Mode::OPERATION || !g_state.op_enable) continue;

    // update g_u here (PID)
    // Control_setPwm(g_u, +1, pwm11);
    // Control_setServo(g_u, deg);
    // bump seq + notify motor
  }
}

// ===== MOTOR task: apply only on event =====
static void Task_Motor(void*)
{
  for(;;) {
    // chờ notify/queue “control updated”
    vTaskDelay(pdMS_TO_TICKS(1));
    Control_applyMotor(g_state, g_u);
  }
}

// ===== ULTRA task =====
static void Task_Ultra(void*)
{
  for(;;) {
    uint32_t bits = 0;
    xTaskNotifyWait(0, 0xFFFFFFFFu, &bits, portMAX_DELAY);
    ultra_kick();
  }
}