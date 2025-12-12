#include "actuators.hpp"

static volatile int32_t g_totalCount = 0;

static uint16_t g_ppr;
static uint16_t g_dt_speed_ms;
static uint16_t g_accum_window_ms;
static uint8_t  g_accum_N;

static float g_alpha;          // IIR filter
static float g_rpm_filt = 0;

static int32_t acc_delta = 0;
static uint8_t acc_count = 0;

// ================== Encoder ======================

void Encoder::updateCount()
{
    g_totalCount++;
}

void Encoder::init(uint16_t ppr,
                   uint16_t dt_speed_ms,
                   uint16_t accum_window_ms,
                   float alpha)
{
    g_ppr = (ppr == 0 ? 1 : ppr);
    g_dt_speed_ms = dt_speed_ms;
    g_accum_window_ms = accum_window_ms;

    g_accum_N = accum_window_ms / dt_speed_ms;
    if (g_accum_N < 1) g_accum_N = 1;

    g_alpha = alpha;
    g_rpm_filt = 0;
    acc_delta = 0;
    acc_count = 0;
}

void Encoder::updateSpeed()
{
    static int32_t lastCount = 0;

    int32_t now;
    noInterrupts();
    now = g_totalCount;
    interrupts();

    int32_t delta = now - lastCount;
    lastCount = now;

    acc_delta += delta;
    acc_count++;

    if (acc_count >= g_accum_N)
    {
        float pulses_per_s = (acc_delta * 1000.0f) /
                             (float)g_accum_window_ms;

        float rpm_raw = (pulses_per_s / g_ppr) * 60.0f;

        if (rpm_raw < 0) rpm_raw = 0;

        // IIR smoothing
        g_rpm_filt = g_alpha * rpm_raw
                   + (1 - g_alpha) * g_rpm_filt;

        acc_delta = 0;
        acc_count = 0;
    }
}

float Encoder::getRPM()
{
    return g_rpm_filt;
}

// ================== Motor ======================

void Motor::init()
{
    ledcSetup(0, 20000, 10);
    ledcAttachPin(25, 0);
}

void Motor::setPWM(int pwm)
{
    pwm = constrain(pwm, 0, 1023);
    ledcWrite(0, pwm);
}
