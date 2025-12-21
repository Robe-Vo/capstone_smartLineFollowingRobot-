#include "op_handlers.hpp"

namespace OpHandlers 
{
    void op_spd_fwd(const Protocol::RxFrame& f)
    {
        op_params.direction = true;
        op_params.speed = f.speed_f;
    }

    void op_spd_bwd(const Protocol::RxFrame& f)
    {
        op_params.direction = false;
        op_params.speed = f.speed_f;
    }

    void op_pwm_fwd(const Protocol::RxFrame& f)
    {
        op_params.useController = false;
        op_params.direction = true;
        op_params.pwm = f.pwm_u16;
    }

    void op_pwm_bwd(const Protocol::RxFrame& f)
    {
        op_params.useController = false;
        op_params.direction = false;
        op_params.speed = f.pwm_u16;
    }

    // Stop - block action
    void op_brake(const Protocol::RxFrame& f)
    {
        Steer::disable();
        Drive::brake();
    }
}