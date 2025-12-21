// esp32/src/app/app.cpp
#include "app.hpp"

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

#include "bluetooth.hpp"
#include "sensors.hpp"
#include "actuators.hpp"

#include "cfg.hpp"
#include "pin.hpp"

#include "state/state.hpp"
#include "protocol/protocol.hpp"
#include "control/control.hpp"
#include "telemetry/telemetry.hpp"

// ======================= GLOBALS =======================

static Network g_net;

// RTOS task handles
static TaskHandle_t gTaskComm  = nullptr;
static TaskHandle_t gTaskPID   = nullptr;
static TaskHandle_t gTaskUltra = nullptr;

// 1 ms tick timer
static hw_timer_t* gTimer1ms = nullptr;
static volatile uint32_t gTickMs = 0;

// PID state
static Control::SpeedCtrlState gPid;
static Control::ValueBuffer    gPidDbg;

// Ultra period (ms)
static volatile uint16_t gUltraPeriodMs = ULTRA_TRIG_PERIOD_MS_DEFAULT;

enum class OpDriveMode : uint8_t {
  PWM_OPEN_LOOP     = 0,
  SPEED_CLOSED_LOOP = 1
};

struct OperationSetpoint {
  OpDriveMode mode      = OpDriveMode::PWM_OPEN_LOOP;
  bool        enabled   = false;

  uint16_t    pwm_cmd   = 0;        // open-loop duty 0..2047
  float       speed_ref = 0.0f;     // closed-loop ref [Hz] (only when using float-speed mode)

  uint16_t    steer_deg = SERVO_MID_DEG;
};

static OperationSetpoint gOpSp;

// Param buffers (raw 30 bytes)
static uint8_t gLineParams[30];
static uint8_t gMpuParams[30];
static uint8_t gUltraParams[30];
static uint8_t gMotorParams[30];
static uint8_t gServoParams[30];
static uint8_t gPidParams[30];

// ======================= FORWARD DECLS =======================

static void Task_Comm(void* arg);
static void Task_PID(void* arg);
static void Task_Ultra(void* arg);

static void handleSystemCmd(const Protocol::RxFrame& f);
static void handleIdleCmd(const Protocol::RxFrame& f);
static void handleOperationCmd(const Protocol::RxFrame& f);

static inline uint16_t clampDuty11(uint16_t v)
{
  if (v > 2047) return 2047;
  return v;
}

static inline uint16_t clampServoDeg(uint16_t deg)
{
  if (deg < SERVO_MIN_DEG) return SERVO_MIN_DEG;
  if (deg > SERVO_MAX_DEG) return SERVO_MAX_DEG;
  return deg;
}

static inline void applySteer(uint16_t deg)
{
  deg = clampServoDeg(deg);
  gOpSp.steer_deg = deg;
  Control::controlActuators(0, deg);
}

// ======================= 1 ms timer ISR =======================

void IRAM_ATTR onTimer1ms()
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  gTickMs++;
  uint32_t t = gTickMs;

  // COMM always runs
  if ((t % TS_COMMUNICATION_MS) == 0 && gTaskComm) {
    vTaskNotifyGiveFromISR(gTaskComm, &xHigherPriorityTaskWoken);
  }

  // PID + ULTRA only in OPERATION
  if (State::isOperation()) {
    // PID offset 5 ms
    const uint32_t PID_OFFSET_MS = 5;
    if (t >= PID_OFFSET_MS &&
        ((t - PID_OFFSET_MS) % TS_CONTROLLER_MS) == 0 &&
        gTaskPID) {
      vTaskNotifyGiveFromISR(gTaskPID, &xHigherPriorityTaskWoken);
    }

    // ULTRA TRIG
    uint16_t up = gUltraPeriodMs;
    if (up > 0 && (t % up) == 0 && gTaskUltra) {
      vTaskNotifyGiveFromISR(gTaskUltra, &xHigherPriorityTaskWoken);
    }
  }

  if (t >= TS_RESET_TMR) {
    gTickMs = 0;
  }

  if (xHigherPriorityTaskWoken == pdTRUE) {
    portYIELD_FROM_ISR();
  }
}

// ======================= SETUP / LOOP =======================

void App_setup()
{
  Serial.begin(115200);

  // Network
  g_net.begin();

  // Sensors
  line_setup(LINE_SENSOR_IDX_1_PIN, LINE_SENSOR_IDX_2_PIN, LINE_SENSOR_IDX_3_PIN,
             LINE_SENSOR_IDX_4_PIN, LINE_SENSOR_IDX_5_PIN);

  ultra_setup(ULTRASONIC_TRIG_PIN, ULTRASONIC_ECHO_PIN);
  attachInterrupt(ULTRASONIC_ECHO_PIN, hanlder_ultra_echo, CHANGE);

  // Motor
  MotorPWM11::begin(MOTOR_PWM_PIN, MOTOR_OUT_1_PIN, MOTOR_OUT_2_PIN,
                    PWM_CH_MOTOR, PWM_FREQ_HZ);
  MotorPWM11::setDeadband(MOTOR_DEADBAND_DUTY_DEFAULT);

  // Encoder
  Encoder::speed_filter_init();
  Encoder::HybridConfig hc;
  Encoder::encoder_begin(ENCODER_CHANNEL_A_PIN, ENCODER_CHANNEL_B_PIN,
                         ENC_EFFECTIVE_PPR, hc);

  // State
  State::setMode(State::Mode::IDLE);
  gOpSp = OperationSetpoint();

  // PID
  Control::PID_setupFromCfg(gPid);

  // RTOS tasks
  xTaskCreatePinnedToCore(Task_Comm,  "comm",  4096, nullptr, 3, &gTaskComm, 0);
  xTaskCreatePinnedToCore(Task_PID,   "pid",   4096, nullptr, 4, &gTaskPID,  0);
  xTaskCreatePinnedToCore(Task_Ultra, "ultra", 2048, nullptr, 1, &gTaskUltra, 1);

  // 1 ms timer
  gTimer1ms = timerBegin(0, 80, true);
  timerAttachInterrupt(gTimer1ms, &onTimer1ms, true);
  timerAlarmWrite(gTimer1ms, 1000, true);
  timerAlarmEnable(gTimer1ms);
}

void App_loop()
{
  vTaskDelay(pdMS_TO_TICKS(100));
}

// ======================= TASK: COMM =======================

static void Task_Comm(void* arg)
{
  (void)arg;

  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    Protocol::RxFrame f{};
    while (Protocol::tryRead(g_net, f)) {
      switch (f.group) {
        case Protocol::Group::SYSTEM:
          handleSystemCmd(f);
          g_net.transmitUint8(Protocol::ACK_IDLE);
          break;

        case Protocol::Group::IDLE:
          handleIdleCmd(f);
          g_net.transmitUint8(Protocol::ACK_IDLE);
          break;

        case Protocol::Group::OPERATION:
          handleOperationCmd(f);
          break;

        default:
          break;
      }
    }

    // Telemetry (OPERATION only) - 22 bytes
    if (State::isOperation()) {
      uint8_t* line5 = line_readSignals();
      int16_t  mpu6_i16[6] = {0,0,0,0,0,0};

      // speed Hz -> int16 (Hz*100)
      float hz = Encoder::speed_get_hz_out();
      int16_t speed_i16 = (int16_t)(hz * 100.0f + (hz >= 0 ? 0.5f : -0.5f));

      // encoder count within dt (pulse count in this telemetry period)
      int32_t enc_dt = Encoder::encoder_snapshot_delta_and_reset_accum();
      if (enc_dt > 127)  enc_dt = 127;
      if (enc_dt < -128) enc_dt = -128;
      int8_t enc_i8 = (int8_t)enc_dt;

      // steer
      int16_t steer_i16 = (int16_t)gOpSp.steer_deg;

      // Debug
      Serial.printf("[TEL] hz=%.2f sp_i16=%d enc_dt=%d steer=%d\n",
                    (double)hz, (int)speed_i16, (int)enc_i8, (int)steer_i16);

      Serial.printf("[ENCDBG] total=%ld period_us=%lu rawP=%.2f rawA=%.2f out=%.2f\n",
                    (long)Encoder::encoder_get_total(),
                    (unsigned long)Encoder::encoder_get_period_us(),
                    (double)Encoder::speed_get_hz_raw_period(),
                    (double)Encoder::speed_get_hz_raw_accum(),
                    (double)Encoder::speed_get_hz_out());

      Telemetry::updateOpTelem22(line5, mpu6_i16, enc_i8, speed_i16, steer_i16);
      Telemetry::sendOpTelem22(g_net);
    }
  }
}

// ======================= TASK: PID / CONTROL =======================

static void Task_PID(void* arg)
{
  (void)arg;
  const float Ts_s = (float)TS_CONTROLLER_MS / 1000.0f;

  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    if (!State::isOperation())                         continue;
    if (!gOpSp.enabled)                                continue;
    if (gOpSp.mode != OpDriveMode::SPEED_CLOSED_LOOP)  continue;

    // Update hybrid speed estimate
    Encoder::hybrid_update(Ts_s);

    float v_meas_hz = fabsf(Encoder::speed_get_hz_out());
    float v_ref_hz  = gOpSp.speed_ref;
    float ref_abs   = fabsf(v_ref_hz);

    uint16_t duty_out = 0;
    Control::PID_update(gPid, ref_abs, v_meas_hz, Ts_s, duty_out, &gPidDbg);

    duty_out = clampDuty11(duty_out);

    if (v_ref_hz >= 0.0f) MotorPWM11::driveForward(duty_out);
    else                  MotorPWM11::driveBackward(duty_out);

    // Debug (only fields guaranteed by your ValueBuffer usage)
    Serial.printf("[PIDDBG] ref=%.2f fdb=%.2f err=%.2f u=%.2f duty=%u\n",
                  (double)gPidDbg.speed_ref,
                  (double)gPidDbg.speed_fdb,
                  (double)gPidDbg.err,
                  (double)gPidDbg.u_raw,
                  (unsigned)gPidDbg.duty_out);
  }
}

// ======================= TASK: ULTRA =======================

static void Task_Ultra(void* arg)
{
  (void)arg;
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    ultra_kick();
  }
}

// ======================= HANDLERS =======================

static void handleSystemCmd(const Protocol::RxFrame& f)
{
  using Protocol::Cmd;

  switch (f.cmd) {
    case Cmd::CMD_GLOBAL_OPERATION:
      State::setMode(State::Mode::OPERATION);
      gOpSp = OperationSetpoint();
      MotorPWM11::stopMotor();
      break;

    case Cmd::CMD_GLOBAL_IDLE:
      State::setMode(State::Mode::IDLE);
      gOpSp = OperationSetpoint();
      MotorPWM11::stopMotor();
      break;

    case Cmd::CMD_GLOBAL_PING_MODE:
      g_net.transmitUint8(State::isOperation() ? 1 : 0);
      break;

    case Cmd::CMD_GLOBAL_EMRGENCY_STOP:
      State::setMode(State::Mode::IDLE);
      gOpSp = OperationSetpoint();
      MotorPWM11::stopMotor();
      break;

    default:
      break;
  }
}

static void handleIdleCmd(const Protocol::RxFrame& f)
{
  using namespace Protocol;

  Serial.printf("[IDLE] CMD=0x%02X\n", (uint8_t)f.cmd);

  switch (f.cmd) {
    case Cmd::CMD_IDLE_SENSOR_LINE_READ: {
      uint8_t* sig = line_readSignals();
      g_net.transmitArrayUint8(sig, 5);
      break;
    }

    case Cmd::CMD_IDLE_SENSOR_ULTRA_KICK:
      ultra_kick();
      break;

    case Cmd::CMD_IDLE_SENSOR_ULTRA_READ: {
      uint16_t d = ultra_getSignal();
      uint8_t buf[2] = {(uint8_t)(d & 0xFF), (uint8_t)((d >> 8) & 0xFF)};
      g_net.transmitArrayUint8(buf, 2);
      break;
    }

    case Cmd::CMD_IDLE_ENCODER_ENABLE:
      Encoder::speed_filter_init();
      break;

    case Cmd::CMD_IDLE_ENCODER_DISABLE:
      break;

    case Cmd::CMD_IDLE_ACTUATOR_MOTOR_PWM_FWD: {
      uint16_t duty = clampDuty11(f.u16);
      MotorPWM11::driveForward(duty);
      break;
    }

    case Cmd::CMD_IDLE_ACTUATOR_MOTOR_PWM_BWD: {
      uint16_t duty = clampDuty11(f.u16);
      MotorPWM11::driveBackward(duty);
      break;
    }

    case Cmd::CMD_IDLE_ACTUATOR_MOTOR_STOP:
      MotorPWM11::stopMotor();
      break;

    case Cmd::CMD_IDLE_ACTUATOR_SERVO_WRITE: {
      applySteer(f.u16);
      Serial.printf("[STEER][IDLE] ang=%u\n", (unsigned)gOpSp.steer_deg);
      break;
    }

    case Cmd::CMD_IDLE_ACTUATOR_SERVO_WRITE_CENTER: {
      applySteer(SERVO_MID_DEG);
      Serial.printf("[STEER][IDLE] center=%u\n", (unsigned)gOpSp.steer_deg);
      break;
    }

    case Cmd::CMD_IDLE_SET_ULTRA_PARAMS: {
      memcpy(gUltraParams, f.bytes30, 30);
      uint16_t trigMs = (uint16_t)(gUltraParams[0] | (gUltraParams[1] << 8));
      if (trigMs == 0) trigMs = ULTRA_TRIG_PERIOD_MS_DEFAULT;
      gUltraPeriodMs = trigMs;
      Serial.printf("[ULTRA] period_ms=%u\n", (unsigned)gUltraPeriodMs);
      break;
    }

    case Cmd::CMD_IDLE_SET_PID_PARAMS: {
      memcpy(gPidParams, f.bytes30, 30);

      float kp, ki, kd, windup, slew;
      uint16_t dutyMin, dutyMax, deadband;

      memcpy(&kp,      &gPidParams[0],  4);
      memcpy(&ki,      &gPidParams[4],  4);
      memcpy(&kd,      &gPidParams[8],  4);
      memcpy(&windup,  &gPidParams[12], 4);
      memcpy(&dutyMin, &gPidParams[16], 2);
      memcpy(&dutyMax, &gPidParams[18], 2);
      memcpy(&slew,    &gPidParams[20], 4);
      memcpy(&deadband,&gPidParams[24], 2);

      Control::PID_init(gPid, kp, ki, kd, windup);
      Control::PID_setOutputLimits(gPid, dutyMin, dutyMax);
      Control::PID_setDerivativeFilter(gPid, PID_TAU_D_DEFAULT);
      Control::PID_setDutySlew(gPid, slew);

      if (deadband <= 2047) gPid.deadband_duty = deadband;

      Serial.printf("[PIDCFG] kp=%.3f ki=%.3f kd=%.3f wind=%.3f out=[%u..%u] slew=%.3f dead=%u\n",
                    (double)kp, (double)ki, (double)kd, (double)windup,
                    (unsigned)dutyMin, (unsigned)dutyMax, (double)slew, (unsigned)gPid.deadband_duty);
      break;
    }

    default:
      break;
  }
}

static void handleOperationCmd(const Protocol::RxFrame& f)
{
  using Protocol::Cmd;

  // Idea to match:
  // - CMD_OP_SPD_FWD / CMD_OP_SPD_BWD carry payload "11-bit speed" + "u16 angle".
  // - Use Protocol fields speed_u16 + angle_u16 for these cmds (same as OP5B layout).
  // - This app treats those as OPEN-LOOP duty11 + steer angle (PID loop remains available for future float-speed mode).
  //
  // Requirement on Protocol::payloadOf():
  // - CMD_OP_SPD_FWD / CMD_OP_SPD_BWD must be decoded as Payload::OP5B
  //   (or otherwise fill out.speed_u16 and out.angle_u16).

  switch (f.cmd) {

    case Cmd::CMD_OP_PWM_FWD:
    case Cmd::CMD_OP_PWM_BWD: {
      uint16_t duty = clampDuty11(f.u16);

      gOpSp.mode    = OpDriveMode::PWM_OPEN_LOOP;
      gOpSp.enabled = true;
      gOpSp.pwm_cmd = duty;

      if (f.cmd == Cmd::CMD_OP_PWM_FWD) MotorPWM11::driveForward(duty);
      else                              MotorPWM11::driveBackward(duty);

      Serial.printf("[OP] PWM cmd=0x%02X duty=%u steer=%u\n",
                    (unsigned)f.cmd, (unsigned)duty, (unsigned)gOpSp.steer_deg);
      break;
    }

    // NEW: speed cmd includes steer angle (duty11 + angle_u16)
    case Cmd::CMD_OP_SPD_FWD:
    case Cmd::CMD_OP_SPD_BWD: {
      uint16_t duty11 = clampDuty11(f.speed_u16);
      uint16_t ang    = clampServoDeg(f.angle_u16);

      applySteer(ang);

      gOpSp.mode    = OpDriveMode::PWM_OPEN_LOOP;
      gOpSp.enabled = true;
      gOpSp.pwm_cmd = duty11;

      if (f.cmd == Cmd::CMD_OP_SPD_FWD) MotorPWM11::driveForward(duty11);
      else                              MotorPWM11::driveBackward(duty11);

      Serial.printf("[OP] SPD+ANG cmd=0x%02X duty11=%u ang=%u\n",
                    (unsigned)f.cmd, (unsigned)duty11, (unsigned)ang);
      break;
    }

    // Legacy combined control: speed_u16 + angle_u16 (4 bytes payload)
    case Cmd::CMD_OP_LEGACY_CTRL_5B: {
      uint16_t duty11 = clampDuty11(f.speed_u16);
      uint16_t ang    = clampServoDeg(f.angle_u16);

      applySteer(ang);

      gOpSp.mode    = OpDriveMode::PWM_OPEN_LOOP;
      gOpSp.enabled = true;
      gOpSp.pwm_cmd = duty11;

      MotorPWM11::driveForward(duty11); // legacy assumes forward; direction must be encoded by MATLAB if needed

      Serial.printf("[OP] LEGACY5B duty11=%u ang=%u\n",
                    (unsigned)duty11, (unsigned)ang);
      break;
    }

    case Cmd::CMD_OP_BRAKE:
      gOpSp = OperationSetpoint();
      MotorPWM11::stopMotor();
      Serial.printf("[OP] BRAKE\n");
      break;

    default:
      break;
  }
}
