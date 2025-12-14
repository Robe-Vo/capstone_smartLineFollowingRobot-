// ======================= actuators.cpp =======================
#include "actuators.hpp"
#include <algorithm>

namespace Encoder
{
    // 0: EMA bậc 1, 1: IIR bậc 2 (cascade)
    static uint8_t g_mode = 0;

    // alpha, beta trong [0..1]
    static float g_alpha = 0.3f;
    static float g_beta  = 0.0f;

    // trạng thái lọc
    static float g_y1 = 0.0f;
    static float g_y2 = 0.0f;

    // output cuối cùng
    static float g_rpm_out = 0.0f;

    // ===== PRO: hybrid + median =====
    static uint16_t g_rpm_switch = 200;  // rpm, default
    static const uint8_t MEDIAN_MAX = 5;
    static uint8_t  g_median_len   = 3;  // 1/3/5
    static float    g_median_buf[MEDIAN_MAX];
    static uint8_t  g_median_idx   = 0;
    static uint8_t  g_median_count = 0;

    // ----------------- lõi IIR cũ -----------------
    void speed_filter_init()
    {
        g_mode  = 0;
        g_alpha = 0.3f;
        g_beta  = 0.0f;
        g_y1    = 0.0f;
        g_y2    = 0.0f;
        g_rpm_out = 0.0f;

        g_rpm_switch  = 200;
        g_median_len  = 3;
        g_median_idx  = 0;
        g_median_count = 0;
        for (uint8_t i = 0; i < MEDIAN_MAX; ++i)
            g_median_buf[i] = 0.0f;
    }

    // mode: 0 = EMA bậc 1, 1 = IIR bậc 2 (cascade)
    // alpha_raw, beta_raw: uint16 scale 1/1000 (0..1000 -> 0..1.000)
    void speed_filter_config(uint8_t mode, uint16_t alpha_raw, uint16_t beta_raw)
    {
        g_mode = (mode > 1) ? 0 : mode;

        float a = (float)alpha_raw / 1000.0f;
        float b = (float)beta_raw / 1000.0f;

        if (a < 0.0f) a = 0.0f;
        if (a > 1.0f) a = 1.0f;
        if (b < 0.0f) b = 0.0f;
        if (b > 1.0f) b = 1.0f;

        g_alpha = a;
        g_beta  = b;

        // reset trạng thái lọc khi đổi thông số
        g_y1 = 0.0f;
        g_y2 = 0.0f;
        g_rpm_out = 0.0f;
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
            g_y2 = g_beta  * g_y1  + (1.0f - g_beta ) * g_y2;
            g_rpm_out = g_y2;
            break;
        }
    }

    float speed_get_rpm()
    {
        return g_rpm_out;
    }

    // ----------------- PRO: cấu hình hybrid + median -----------------
    void pro_config(uint16_t rpm_switch_raw, uint8_t median_len)
    {
        g_rpm_switch = rpm_switch_raw;   // rpm, Q0

        if (median_len < 1) median_len = 1;
        if (median_len > MEDIAN_MAX) median_len = MEDIAN_MAX;
        // đảm bảo cửa sổ lẻ để lấy median
        if ((median_len % 2) == 0)
            median_len -= 1;

        g_median_len   = median_len;
        g_median_idx   = 0;
        g_median_count = 0;

        for (uint8_t i = 0; i < MEDIAN_MAX; ++i)
            g_median_buf[i] = 0.0f;
    }

    // ----------------- PRO: cập nhật hybrid mỗi frame -----------------
    void pro_update(float rpm_period, float rpm_accum, uint16_t pulses_frame)
    {
        float rpm_model = 0.0f;
        bool has_period = (rpm_period > 0.0f);
        bool has_accum  = (pulses_frame > 0 && rpm_accum > 0.0f);

        if (!has_period && !has_accum)
        {
            rpm_model = 0.0f;
        }
        else if (has_period && !has_accum)
        {
            rpm_model = rpm_period;
        }
        else if (!has_period && has_accum)
        {
            rpm_model = rpm_accum;
        }
        else
        {
            // Hybrid: tốc độ thấp → period, tốc độ cao → accumulation
            rpm_model = (rpm_accum >= (float)g_rpm_switch) ? rpm_accum : rpm_period;
        }

        // ---- median filter trên rpm_model ----
        float x = rpm_model;

        if (g_median_len > 1)
        {
            g_median_buf[g_median_idx] = x;
            g_median_idx = (g_median_idx + 1) % g_median_len;
            if (g_median_count < g_median_len)
                ++g_median_count;

            float tmp[MEDIAN_MAX];
            for (uint8_t i = 0; i < g_median_count; ++i)
                tmp[i] = g_median_buf[i];

            std::sort(tmp, tmp + g_median_count);
            float med = tmp[g_median_count / 2];

            speed_filter_update(med);
        }
        else
        {
            // median_len == 1 → bỏ qua median, vào thẳng IIR
            speed_filter_update(x);
        }
    }
}
