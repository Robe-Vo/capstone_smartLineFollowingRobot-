// ======================= protocol.cpp (IF/ELSE ONLY, CONSISTENT LITTLE-ENDIAN) =======================

#include "protocol.hpp"

namespace Protocol {

static inline bool readU8(Network& net, uint8_t& v) { return net.getUint8(v); }
static inline bool readN(Network& net, uint8_t* buf, size_t n) { return net.getArrayUint8(buf, n); }

static inline void clearIdlePayload(IdleCmd& out) {
    out.len = 0;
    memset(out.data, 0, sizeof(out.data));
}

bool isIdleCmd(uint8_t c, uint8_t& len) {
    len = 0;

    // -------- Len = 0 --------
    if (c == (uint8_t)Cmd::CMD_IDLE_SENSOR_LINE_READ  ||
        c == (uint8_t)Cmd::CMD_IDLE_SENSOR_ULTRA_READ ||
        c == (uint8_t)Cmd::CMD_IDLE_SENSOR_ULTRA_KICK ||
        c == (uint8_t)Cmd::CMD_IDLE_SENSOR_MPU_READ   ||
        c == (uint8_t)Cmd::CMD_IDLE_ENCODER_ENABLE    ||
        c == (uint8_t)Cmd::CMD_IDLE_ENCODER_DISABLE   ||
        c == (uint8_t)Cmd::CMD_IDLE_ACTUATOR_MOTOR_ENABLE  ||
        c == (uint8_t)Cmd::CMD_IDLE_ACTUATOR_MOTOR_DISABLE ||
        c == (uint8_t)Cmd::CMD_IDLE_ACTUATOR_MOTOR_STOP    ||
        c == (uint8_t)Cmd::CMD_IDLE_ACTUATOR_SERVO_ENABLE  ||
        c == (uint8_t)Cmd::CMD_IDLE_ACTUATOR_SERVO_DISABLE ||
        c == (uint8_t)Cmd::CMD_IDLE_ACTUATOR_SERVO_READ    ||
        c == (uint8_t)Cmd::CMD_IDLE_ACTUATOR_SERVO_WRITE_CENTER)
    {
        len = 0;
        return true;
    }

    // -------- Len = 2 --------
    if (c == (uint8_t)Cmd::CMD_IDLE_ACTUATOR_SERVO_WRITE) {
        len = 2;
        return true;
    }

    // -------- Len = 30 (set params blocks) --------
    if (c == (uint8_t)Cmd::CMD_IDLE_SET_LINE_PARAMS  ||
        c == (uint8_t)Cmd::CMD_IDLE_SET_MPU_PARAMS   ||
        c == (uint8_t)Cmd::CMD_IDLE_SET_ULTRA_PARAMS ||
        c == (uint8_t)Cmd::CMD_IDLE_SET_MOTOR_PARAMS ||
        c == (uint8_t)Cmd::CMD_IDLE_SET_SERVO_PARAMS ||
        c == (uint8_t)Cmd::CMD_IDLE_SET_PID_PARAMS)
    {
        len = 30;
        return true;
    }

    // -------- Optional: enable these only if you really send payload in IDLE for motor --------
    // if (c == (uint8_t)Cmd::CMD_IDLE_ACTUATOR_MOTOR_PWM_FWD ||
    //     c == (uint8_t)Cmd::CMD_IDLE_ACTUATOR_MOTOR_PWM_BWD)
    // {
    //     len = 2; // pwm_u16
    //     return true;
    // }
    //
    // if (c == (uint8_t)Cmd::CMD_IDLE_ACTUATOR_MOTOR_SPD_FWD ||
    //     c == (uint8_t)Cmd::CMD_IDLE_ACTUATOR_MOTOR_SPD_BWD)
    // {
    //     len = 4; // speed_f32
    //     return true;
    // }

    return false;
}

bool isOperationCmd(uint8_t c, uint8_t& len) {
    len = 0;

    if (c == (uint8_t)Cmd::CMD_OP_PWM_FWD ||
        c == (uint8_t)Cmd::CMD_OP_PWM_BWD ||
        c == (uint8_t)Cmd::CMD_OP_SPD_FWD ||
        c == (uint8_t)Cmd::CMD_OP_SPD_BWD)
    {
        len = 6; // value(4) + angle(2)
        return true;
    }

    if (c == (uint8_t)Cmd::CMD_OP_BRAKE) {
        len = 0;
        return true;
    }

    return false;
}

static inline void decodeIdleParams(IdleCmd& out) {
    // Uses if/else only.
    const uint8_t* p = out.data;
    const uint8_t cmd = out.cmd;

    // LINE params
    if (cmd == (uint8_t)Cmd::CMD_IDLE_SET_LINE_PARAMS) {
        if (out.len >= 6) {
            out.line_sample_ms = u16_le(p + 0);
            out.line_threshold = u16_le(p + 2);
            out.line_reserved0 = u16_le(p + 4);
        }
        return;
    }

    // MPU params
    if (cmd == (uint8_t)Cmd::CMD_IDLE_SET_MPU_PARAMS) {
        if (out.len >= 14) {
            out.mpu_sample_rate_hz = u16_le(p + 0);
            out.mpu_bias_ax = f32_le(p + 2);
            out.mpu_bias_ay = f32_le(p + 6);
            out.mpu_bias_az = f32_le(p + 10);
        }
        return;
    }

    // ULTRA params
    if (cmd == (uint8_t)Cmd::CMD_IDLE_SET_ULTRA_PARAMS) {
        if (out.len >= 6) {
            out.ultra_kick_period_ms = u16_le(p + 0);
            out.ultra_timeout_ms     = u16_le(p + 2);
            out.ultra_reserved0      = u16_le(p + 4);
        }
        return;
    }

    // MOTOR params
    if (cmd == (uint8_t)Cmd::CMD_IDLE_SET_MOTOR_PARAMS) {
        if (out.len >= 8) {
            out.motor_pwm_freq_hz = u16_le(p + 0);
            out.motor_pwm_max     = u16_le(p + 2);
            out.motor_ff          = f32_le(p + 4);
        }
        return;
    }

    // SERVO params
    if (cmd == (uint8_t)Cmd::CMD_IDLE_SET_SERVO_PARAMS) {
        if (out.len >= 6) {
            out.servo_center = u16_le(p + 0);
            out.servo_min    = u16_le(p + 2);
            out.servo_max    = u16_le(p + 4);
        }
        return;
    }

    // PID params
    if (cmd == (uint8_t)Cmd::CMD_IDLE_SET_PID_PARAMS) {
        if (out.len >= 16) {
            out.motor_kp     = f32_le(p + 0);
            out.motor_ki     = f32_le(p + 4);
            out.motor_kd     = f32_le(p + 8);
            out.motor_windup = f32_le(p + 12);
        }
        return;
    }
}

RxResult poll(Network& net, Mode currentMode) {
    RxResult r;
    r.ok = false;
    r.mode = currentMode;
    r.flag_cmd = 0;

    uint8_t cmd;
    if (!readU8(net, cmd)) return r;

    r.flag_cmd = cmd;

    // Apply mode switch (no payload)
    if (cmd == (uint8_t)Cmd::CMD_GLOBAL_OPERATION) r.mode = Mode::OPERATION;
    else if (cmd == (uint8_t)Cmd::CMD_GLOBAL_IDLE) r.mode = Mode::IDLE;

    // System cmd: done
    if (isSystemCmd(cmd)) {
        r.ok = true;
        return r;
    }

    // OPERATION mode parse
    if (r.mode == Mode::OPERATION) {
        r.op.cmd = cmd;

        uint8_t need = 0;
        if (isOperationCmd(cmd, need) && need > 0) {
            uint8_t buf[6];
            if (!readN(net, buf, sizeof(buf))) return r;

            r.op.angle = u16_le(&buf[4]);

            if (cmd == (uint8_t)Cmd::CMD_OP_PWM_FWD || cmd == (uint8_t)Cmd::CMD_OP_PWM_BWD) {
                r.op.pwm = u16_le(&buf[0]); // buf[2..3] reserved
            } else if (cmd == (uint8_t)Cmd::CMD_OP_SPD_FWD || cmd == (uint8_t)Cmd::CMD_OP_SPD_BWD) {
                r.op.speed = f32_le(&buf[0]);
            }
        }

        r.ok = true;
        return r;
    }

    // IDLE mode parse
    if (r.mode == Mode::IDLE) {
        r.idle.cmd = cmd;
        clearIdlePayload(r.idle);

        uint8_t need = 0;
        if (isIdleCmd(cmd, need) && need > 0) {
            if (need > sizeof(r.idle.data)) return r;
            if (!readN(net, r.idle.data, need)) return r;
            r.idle.len = need;

            // decode set-params into typed fields
            decodeIdleParams(r.idle);

            // also decode simple 2-byte servo write into typed fields if you want:
            // (kept raw here; App can parse from data[] if needed)
        } else {
            r.idle.len = 0;
        }

        r.ok = true;
        return r;
    }

    // fallback
    r.ok = true;
    return r;
}

} // namespace Protocol
