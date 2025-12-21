// lib/dispatch/dispatch.cpp
#include "dispatch.hpp"
#include "protocol.hpp"

// handlers headers
#include "system_handlers.hpp"
#include "idle_handlers.hpp"
#include "op_handlers.hpp"

namespace Dispatch {

  static Handler s_table[256];

  static inline void reg(Protocol::Cmd c, Handler h) {
    s_table[(uint8_t)c] = h;
  }

  void init()
  {
    // default: no handler
    for (int i = 0; i < 256; ++i) s_table[i] = nullptr;

    // ================= SYSTEM =================
    reg(Protocol::Cmd::CMD_GLOBAL_IDLE,          SystemHandlers::to_idle);
    reg(Protocol::Cmd::CMD_GLOBAL_OPERATION,     SystemHandlers::to_operation);
    reg(Protocol::Cmd::CMD_GLOBAL_PING_MODE,     SystemHandlers::ping_mode);
    reg(Protocol::Cmd::CMD_GLOBAL_EMRGENCY_STOP, SystemHandlers::emergency_stop);

    // ================= IDLE: MOTOR =================
    reg(Protocol::Cmd::CMD_IDLE_ACTUATOR_MOTOR_ENABLE,  IdleHandlers::motor_enable);
    reg(Protocol::Cmd::CMD_IDLE_ACTUATOR_MOTOR_DISABLE, IdleHandlers::motor_disable);
    reg(Protocol::Cmd::CMD_IDLE_ACTUATOR_MOTOR_SPD_FWD, IdleHandlers::motor_spd_fwd);
    reg(Protocol::Cmd::CMD_IDLE_ACTUATOR_MOTOR_SPD_BWD, IdleHandlers::motor_spd_bwd);
    reg(Protocol::Cmd::CMD_IDLE_ACTUATOR_MOTOR_PWM_FWD, IdleHandlers::motor_pwm_fwd);
    reg(Protocol::Cmd::CMD_IDLE_ACTUATOR_MOTOR_PWM_BWD, IdleHandlers::motor_pwm_bwd);
    reg(Protocol::Cmd::CMD_IDLE_ACTUATOR_MOTOR_BRAKE,   IdleHandlers::motor_brake);

    // ================= IDLE: SERVO =================
    reg(Protocol::Cmd::CMD_IDLE_ACTUATOR_SERVO_ENABLE,       IdleHandlers::servo_enable);
    reg(Protocol::Cmd::CMD_IDLE_ACTUATOR_SERVO_DISABLE,      IdleHandlers::servo_disable);
    reg(Protocol::Cmd::CMD_IDLE_ACTUATOR_SERVO_WRITE,        IdleHandlers::servo_write);
    reg(Protocol::Cmd::CMD_IDLE_ACTUATOR_SERVO_WRITE_CENTER, IdleHandlers::servo_write_center);
    reg(Protocol::Cmd::CMD_IDLE_ACTUATOR_SERVO_READ,         IdleHandlers::servo_read);

    // ================= IDLE: SENSOR =================
    reg(Protocol::Cmd::CMD_IDLE_SENSOR_LINE_READ,  IdleHandlers::sensor_read_line);
    reg(Protocol::Cmd::CMD_IDLE_SENSOR_ULTRA_READ, IdleHandlers::sensor_read_ultra);
    reg(Protocol::Cmd::CMD_IDLE_SENSOR_MPU_READ,   IdleHandlers::sensor_read_mpu);

    // ================= IDLE: SET PARAMS (30B) =================
    reg(Protocol::Cmd::CMD_IDLE_SET_LINE_PARAMS,  IdleHandlers::set_line_params);
    reg(Protocol::Cmd::CMD_IDLE_SET_ULTRA_PARAMS, IdleHandlers::set_ultra_params);
    reg(Protocol::Cmd::CMD_IDLE_SET_MPU_PARAMS,   IdleHandlers::set_mpu_params);
    reg(Protocol::Cmd::CMD_IDLE_SET_MOTOR_PARAMS, IdleHandlers::set_motor_params);
    reg(Protocol::Cmd::CMD_IDLE_SET_SERVO_PARAMS, IdleHandlers::set_servo_params);
    reg(Protocol::Cmd::CMD_IDLE_SET_PID_PARAMS,   IdleHandlers::set_pid_params);

    // ================= OPERATION =================
    reg(Protocol::Cmd::CMD_OP_SPD_FWD, OpHandlers::op_spd_fwd);
    reg(Protocol::Cmd::CMD_OP_SPD_BWD, OpHandlers::op_spd_bwd);
    reg(Protocol::Cmd::CMD_OP_PWM_FWD, OpHandlers::op_pwm_fwd);
    reg(Protocol::Cmd::CMD_OP_PWM_BWD, OpHandlers::op_pwm_bwd);
    reg(Protocol::Cmd::CMD_OP_BRAKE,   OpHandlers::op_brake);
  }

  // Run fucntion matches to cmd
  bool run(const Protocol::RxFrame& f)
  {
    // Allow SYSTEM always
    if (f.group == Protocol::Group::SYSTEM) {
      auto h = s_table[(uint8_t)f.cmd];
      if (!h) return false;
      h(f);
      return true;
    }

    // Allow only matching group by robotMode
    if (robotMode == Mode::IDLE) {
      if (f.group != Protocol::Group::IDLE) return false;
    } else { // Mode::OPERATION
      if (f.group != Protocol::Group::OPERATION) return false;
    }

    auto h = s_table[(uint8_t)f.cmd];
    if (!h) return false;
    h(f);
    return true;

  }

} // namespace Dispatch
