// ======================= actuators.cpp =======================
#include "actuators.hpp"

namespace Encoder
{
    // 0: EMA bậc 1, 1: IIR bậc 2 (cascade)
    static uint8_t g_mode  = 0;

    // alpha, beta trong [0..1]
    static float   g_alpha = 0.3f;
    static float   g_beta  = 0.0f;

    // trạng thái lọc
    static float   g_y1    = 0.0f;
    static float   g_y2    = 0.0f;

    // output cuối cùng
    static float   g_rpm_out = 0.0f;

    void speed_filter_init()
    {
        g_mode   = 0;
        g_alpha  = 0.3f;
        g_beta   = 0.0f;
        g_y1     = 0.0f;
        g_y2     = 0.0f;
        g_rpm_out= 0.0f;
    }

    // mode: 0 = EMA bậc 1, 1 = IIR bậc 2 (cascade)
    // alpha_raw, beta_raw: uint16 scale 1/1000 (0..1000 -> 0..1.000)
    void speed_filter_config(uint8_t mode, uint16_t alpha_raw, uint16_t beta_raw)
    {
        g_mode = (mode > 1) ? 0 : mode;

        float a = (float)alpha_raw / 1000.0f;
        float b = (float)beta_raw  / 1000.0f;

        if (a < 0.0f) a = 0.0f;
        if (a > 1.0f) a = 1.0f;
        if (b < 0.0f) b = 0.0f;
        if (b > 1.0f) b = 1.0f;

        g_alpha = a;
        g_beta  = b;

        // reset trạng thái lọc khi đổi thông số
        g_y1     = 0.0f;
        g_y2     = 0.0f;
        g_rpm_out= 0.0f;
    }

    void speed_filter_update(float rpm_raw)
    {
        switch (g_mode)
        {
        case 0: // EMA bậc 1
        default:
            g_y1 = g_alpha * rpm_raw + (1.0f - g_alpha) * g_y1;
            g_rpm_out = g_y1;
            break;

        case 1: // IIR bậc 2: cascade 2 EMA
            g_y1 = g_alpha * rpm_raw + (1.0f - g_alpha) * g_y1;
            g_y2 = g_beta  * g_y1    + (1.0f - g_beta ) * g_y2;
            g_rpm_out = g_y2;
            break;
        }
    }

    float speed_get_rpm()
    {
        return g_rpm_out;
    }
}
