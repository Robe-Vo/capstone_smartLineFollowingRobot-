#include "idle_handlers.hpp"
#include <string.h>

// Help functions
// Local decode helpers (kept inside this .cpp; not additional library helpers)
static inline uint16_t u16_le(const uint8_t* p)
{
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}
static inline int16_t i16_le(const uint8_t* p)
{
  return (int16_t)u16_le(p);
}
static inline float f32_le(const uint8_t* p)
{
  float v;
  uint8_t b[4] = { p[0], p[1], p[2], p[3] };
  memcpy(&v, b, 4);
  return v;
}


namespace IdleHandlers
{
  static inline uint16_t sat_pwm11(uint16_t v)
  {
    return (v > 2047) ? 2047 : v;
  }

  // ================= MOTOR =================

  void motor_enable(const Protocol::RxFrame& f)
  {
    (void)f;
    idle_params.motor_isWorking = true;

    // Action: ensure output is safe (start with brake / 0 duty)
    Drive::brake();
    idle_params.motor_mode = 0;
    // ACK byte
    net.transmitUint8(0x20);
  }

  void motor_disable(const Protocol::RxFrame& f)
  {
    (void)f;
    // Action: stop motor immediately
    Drive::brake();

    idle_params.motor_isWorking = false;
    idle_params.motor_mode = 0;
  }

  void motor_spd_fwd(const Protocol::RxFrame& f)
  {
    // ACK byte
    net.transmitUint8(0x20);
    // Your Protocol RX stores 2B speed into f.u16 for U16_LE payload.
    // Action here: mark "speed mode forward" and store last request.
    // Actual PID update should run in your periodic control task.
    idle_params.motor_isWorking = true;
    idle_params.motor_mode = 1;

    // Minimal direct action (safe): do NOT guess PID here.
    // Keep motor stopped until your control loop consumes the request.
    Drive::brake();

    // Store request in a place that exists today:
    // - You currently do NOT have a field for target speed in Idle_values.
    // - So you must consume f.u16 immediately in your control loop or add a field later.
    // Here we store it into ultra_signal as a placeholder is wrong -> do not do that.
    // Therefore: no persistent storage here, only mode flag.
    (void)f;
  }

  void motor_spd_bwd(const Protocol::RxFrame& f)
  {
    // ACK byte
    net.transmitUint8(0x20);
    idle_params.motor_isWorking = true;
    idle_params.motor_mode = 2;

    Drive::brake();
    (void)f;
  }

  void motor_pwm_fwd(const Protocol::RxFrame& f)
  {
    // ACK byte
    net.transmitUint8(0x20);
    // Payload is 2B u16 (PWM), already decoded into f.u16
    idle_params.motor_isWorking = true;
    idle_params.motor_mode = 3;

    // Action: apply PWM now
    uint16_t pwm = sat_pwm11(f.u16);
    Drive::setPWM(pwm, true);
  }

  void motor_pwm_bwd(const Protocol::RxFrame& f)
  {
    // ACK byte
    net.transmitUint8(0x20);
    idle_params.motor_isWorking = true;
    idle_params.motor_mode = 4;

    uint16_t pwm = sat_pwm11(f.u16);
    Drive::setPWM(pwm, false);
  }

  void motor_brake(const Protocol::RxFrame& f)
  {
    // ACK byte
    net.transmitUint8(0x20);
    (void)f;
    Drive::brake();
    idle_params.motor_mode = 0;
  }

  // ================= SERVO =================

  void servo_enable(const Protocol::RxFrame& f)
  {
    // ACK byte
    net.transmitUint8(0x20);
    (void)f;
    idle_params.servo_isWorking = true;

    // Action: enable servo and hold center
    Steer::enable();
    Steer::writeMidAngle();
    idle_params.servo_writeAngle = SERVO_MID_DEG;
  }

  void servo_disable(const Protocol::RxFrame& f)
  {
    // ACK byte
    net.transmitUint8(0x20);
    (void)f;
    // Action: detach servo
    Steer::disable();
    idle_params.servo_isWorking = false;
  }

  void servo_write(const Protocol::RxFrame& f)
  {
    // ACK byte
    net.transmitUint8(0x20);
    // IDLE servo write is 2B angle -> your tryRead decodes I16_LE into f.i16
    // but your RxFrame also has angle_i16 for OP payload.
    // For CMD_IDLE_ACTUATOR_SERVO_WRITE, payload is I16_LE -> use f.i16.
    idle_params.servo_isWorking = true;

    int16_t ang = f.i16;
    idle_params.servo_writeAngle = ang;

    // Action: ensure enabled, then write
    Steer::enable();
    Steer::writeAngle(ang);
  }

  void servo_write_center(const Protocol::RxFrame& f)
  {
    // ACK byte
    net.transmitUint8(0x20);
    (void)f;
    idle_params.servo_isWorking = true;
    idle_params.servo_writeAngle = SERVO_MID_DEG;

    Steer::enable();
    Steer::writeMidAngle();
  }

  void servo_read(const Protocol::RxFrame& f)
  {
    // ACK byte
    net.transmitUint8(0x20);
    (void)f;
    // You do not have a "get angle" API in Steer namespace.
    // Action: none at actuator level.
    // Practical behavior: respond using the last commanded angle (idle_params.servo_writeAngle)
    // but TX response is outside this handler because handler has no Network reference.
  }

  // ================= SENSOR READ =================

  void sensor_read_line(const Protocol::RxFrame& f)
  {
    // ACK byte
    net.transmitUint8(0x20);
    (void)f;
    // Action: sample immediately and store pointer (line_readSignals returns uint8_t*)
    idle_params.line_signals = line_readSignals();
    net.transmitArrayUint8(idle_params.line_signals,5); // Transmit 5 uint8 
  }

  void sensor_read_ultra(const Protocol::RxFrame& f)
  {
    // ACK byte
    net.transmitUint8(0x20);
    (void)f;
    // Action: sample immediately
    ultra_enable_isr();
    ultra_kick();
    delay(200);
    ultra_disable_isr();
    idle_params.ultra_signal = ultra_getSignal();

    // Decode distance value
    uint8_t b0 = (uint8_t)(idle_params.ultra_signal & 0xFF);        // low byte  = 0x34
    uint8_t b1 = (uint8_t)((idle_params.ultra_signal >> 8) & 0xFF); // high byte = 0x12

    net.transmitUint8(b0);
    net.transmitUint8(b1);
  }

  void sensor_read_mpu(const Protocol::RxFrame& f)
  {
    // ACK byte
    net.transmitUint8(0x20);
    (void)f;
    // MPU functions are commented in sensors.hpp. No callable API exists here.
    // Action: none.
  }

  // ================= SET PARAMS (30B) =================
  // Each SET_* receives f.bytes30[30]. You must define a packing spec on PC.
  // Here I only show safe "store-only" stubs because there is no cfg struct in your codebase
  // to write into from these 30 bytes.

  void set_line_params(const Protocol::RxFrame& f)
  {
    // ACK byte
    net.transmitUint8(0x20);
    (void)f;
    // Action: not implemented because no line-param storage + no spec defined in code.
  }

  void set_ultra_params(const Protocol::RxFrame& f)
  {
    // ACK byte
    net.transmitUint8(0x20);
    (void)f;
  }

  void set_mpu_params(const Protocol::RxFrame& f)
  {
    // ACK byte
    net.transmitUint8(0x20);
    (void)f;
  }

  void set_motor_params(const Protocol::RxFrame& f)
  {
    // ACK byte
    net.transmitUint8(0x20);

    // ===================== PAYLOAD (30 bytes) =====================
    // bytes30[0..1] : uint16 (LE) MOTOR_DEADBAND_DUTY_DEFAULT   (0..2047)
    // bytes30[2..29]: reserved (set 0)
    //
    // Total = 2 bytes used, 28 bytes reserved
    // ==============================================================
    uint16_t deadband = u16_le(&f.bytes30[0]);
    if (deadband > 2047) deadband = 2047;

    // Runtime update (cfg.hpp is now extern vars)
    MOTOR_DEADBAND_DUTY_DEFAULT = deadband;

    // Optional: if your Controller::Cfg uses deadband, mirror into current PID cfg
    pid_params.deadband = MOTOR_DEADBAND_DUTY_DEFAULT;
  }

  void set_servo_params(const Protocol::RxFrame& f)
  {
    // ACK byte
    net.transmitUint8(0x20);

    // ===================== PAYLOAD (30 bytes) =====================
    // bytes30[0..1] : int16  (LE) SERVO_MIN_DEG
    // bytes30[2..3] : int16  (LE) SERVO_MID_DEG
    // bytes30[4..5] : int16  (LE) SERVO_MAX_DEG
    // bytes30[6..29]: reserved (set 0)
    //
    // Total = 6 bytes used, 24 bytes reserved
    // ==============================================================
    int16_t smin = i16_le(&f.bytes30[0]);
    int16_t smid = i16_le(&f.bytes30[2]);
    int16_t smax = i16_le(&f.bytes30[4]);

    // Basic sanity (keep order, avoid invalid clamp)
    if (smin > smax) { int16_t t = smin; smin = smax; smax = t; }
    if (smid < smin) smid = smin;
    if (smid > smax) smid = smax;

    SERVO_MIN_DEG = (uint16_t)smin;
    SERVO_MID_DEG = (uint16_t)smid;
    SERVO_MAX_DEG = (uint16_t)smax;

    // Optional: keep IDLE state consistent
    idle_params.servo_writeAngle = (int16_t)SERVO_MID_DEG;
  }

  void set_pid_params(const Protocol::RxFrame& f)
  {
    // ACK byte
    net.transmitUint8(0x20);

    // ===================== PAYLOAD (30 bytes) =====================
    // Little-endian layout (float32 + u16):
    //
    // bytes30[0..3]   : float32 kp
    // bytes30[4..7]   : float32 ki
    // bytes30[8..11]  : float32 kd
    // bytes30[12..15] : float32 i_limit (windup clamp, >=0)
    // bytes30[16..17] : uint16  out_min (0..2047)
    // bytes30[18..19] : uint16  out_max (0..2047)
    // bytes30[20..23] : float32 d_tau_s (>=0)
    // bytes30[24..27] : float32 out_slew_per_s (>=0)
    // bytes30[28..29] : reserved
    //
    // Total = 28 bytes used, 2 bytes reserved
    // ==============================================================
    float kp = f32_le(&f.bytes30[0]);
    float ki = f32_le(&f.bytes30[4]);
    float kd = f32_le(&f.bytes30[8]);

    float i_limit = f32_le(&f.bytes30[12]);
    uint16_t out_min = u16_le(&f.bytes30[16]);
    uint16_t out_max = u16_le(&f.bytes30[18]);

    float d_tau_s = f32_le(&f.bytes30[20]);
    float out_slew_per_s = f32_le(&f.bytes30[24]);

    // Sanity clamps
    if (i_limit < 0.0f) i_limit = 0.0f;
    if (d_tau_s < 0.0f) d_tau_s = 0.0f;
    if (out_slew_per_s < 0.0f) out_slew_per_s = 0.0f;

    if (out_min > 2047) out_min = 2047;
    if (out_max > 2047) out_max = 2047;
    if (out_min > out_max) { uint16_t t = out_min; out_min = out_max; out_max = t; }

    // Update cfg.hpp runtime vars (defaults now behave as live config)
    PID_KP_DEFAULT = kp;
    PID_KI_DEFAULT = ki;
    PID_KD_DEFAULT = kd;

    PID_WINDUP_LIMIT_DEFAULT = i_limit;
    PID_DUTY_MIN_DEFAULT = out_min;
    PID_DUTY_MAX_DEFAULT = out_max;

    PID_TAU_D_DEFAULT = d_tau_s;
    PID_DUTY_SLEW_PER_S_DEFAULT = out_slew_per_s;

    // Update the Controller::Cfg actually used by Controller::update(...)
    pid_params.kp = PID_KP_DEFAULT;
    pid_params.ki = PID_KI_DEFAULT;
    pid_params.kd = PID_KD_DEFAULT;

    pid_params.i_limit = PID_WINDUP_LIMIT_DEFAULT;
    pid_params.out_min = PID_DUTY_MIN_DEFAULT;
    pid_params.out_max = PID_DUTY_MAX_DEFAULT;

    pid_params.d_tau_s = PID_TAU_D_DEFAULT;
    pid_params.out_slew_per_s = PID_DUTY_SLEW_PER_S_DEFAULT;

    // Keep deadband consistent with motor config
    pid_params.deadband = MOTOR_DEADBAND_DUTY_DEFAULT;

    // Reset controller state to avoid jump from old integrator
    Controller::reset(controller_params);
  }

} // namespace IdleHandlers
