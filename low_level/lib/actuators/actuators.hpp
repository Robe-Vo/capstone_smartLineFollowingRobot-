// ======================= actuators.hpp =======================
#pragma once

#include <Arduino.h>
#include <stdint.h>

namespace Encoder
{
    // mode lọc: 0 = EMA bậc 1, 1 = IIR bậc 2 (cascade 2 EMA)
    void speed_filter_init();

    // Giữ nguyên API cũ (dùng nội bộ, vẫn public để tương thích)
    void speed_filter_config(uint8_t mode, uint16_t alpha_raw, uint16_t beta_raw);
    void speed_filter_update(float rpm_raw);
    float speed_get_rpm();

    // PRO: cấu hình hybrid + median
    // rpm_switch_raw: ngưỡng chuyển period -> accumulation (rpm, Q0)
    // median_len: độ dài cửa sổ median (1,3,5)
    void pro_config(uint16_t rpm_switch_raw, uint8_t median_len);

    // PRO: cập nhật hybrid mỗi frame
    //  - rpm_period: tốc độ ước lượng từ period (có thể 0 nếu không đủ cạnh)
    //  - rpm_accum : tốc độ ước lượng từ accumulation frame
    //  - pulses_frame: số xung trong frame (dùng để detect very-low-speed)
    void pro_update(float rpm_period, float rpm_accum, uint16_t pulses_frame);
}
