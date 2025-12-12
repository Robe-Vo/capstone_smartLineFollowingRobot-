#include "actuators.hpp"

/** =========================================================================================================
 *                                          Drive function
   =========================================================================================================*/

// ======================= Private function definitions =======================

// Change direction of motor according to state "direction"
void Actuators::Drive::change_direction()
{
    switch (this->direction)
    {
        // IDLE
        case 0:
            digitalWrite(MOTOR_OUT_1_PIN, false);
            digitalWrite(MOTOR_OUT_2_PIN, false);
            encoder_calculation_flag = false;
            break;

        // Run forward
        case 1:
            digitalWrite(MOTOR_OUT_1_PIN, true);
            digitalWrite(MOTOR_OUT_2_PIN, false);
            break;

        // Run backward
        case 2:
            digitalWrite(MOTOR_OUT_1_PIN, false);
            digitalWrite(MOTOR_OUT_2_PIN, true);
            break;

        default:
            digitalWrite(MOTOR_OUT_1_PIN, false);
            digitalWrite(MOTOR_OUT_2_PIN, false);
            encoder_calculation_flag = false;
            direction = 0;
            break;
    }
}

void Actuators::Drive::update_direction(bool dir_fwd)
{
    if (dir_fwd)
    {
        this->direction = 1; // forward
    }
    else
    {
        this->direction = 2; // backward
    }
    change_direction();
}

// ======================= Get function definitions =======================

int8_t Actuators::Drive::get_directionStatus()
{
    return this->direction;
}

int16_t Actuators::Drive::get_respondedSpeed()
{
    // cast float to int16 for external use
    if (responded_speed < 0.0f) return 0;
    if (responded_speed > 32767.0f) return 32767;
    return static_cast<int16_t>(responded_speed);
}

// Check motor rotation state based on encoder pulses
bool Actuators::Drive::isRotating(unsigned long tmr)
{
    // If no pulse has been received yet, consider not rotating after wait time
    if (!rotate_flag)
    {
        if ((tmr - timer) > WAIT_RESPONSE_TIME)
        {
            return false;
        }
        else
        {
            return true;
        }
    }
    else
    {
        // At least one pulse received; check time since last pulse
        if ((tmr - timer) > WAIT_RESPONSE_TIME)
        {
            return false;
        }
        return true;
    }
}

// ======================= Public function definitions =======================

Actuators::Drive::Drive(uint8_t MOTOR_OUT_1_PIN,
                        uint8_t MOTOR_OUT_2_PIN,
                        uint8_t MOTOR_PWM_PIN,
                        uint8_t ENCODER_CHANNEL_1,
                        uint8_t ENCODER_CHANNEL_2,
                        size_t  RESOLUTION,
                        unsigned long wait_time)
{
    this->MOTOR_OUT_1_PIN    = MOTOR_OUT_1_PIN;
    this->MOTOR_OUT_2_PIN    = MOTOR_OUT_2_PIN;
    this->MOTOR_PWM_PIN      = MOTOR_PWM_PIN;
    this->ENCODER_CHANNEL_A  = ENCODER_CHANNEL_1;
    this->ENCODER_CHANNEL_B  = ENCODER_CHANNEL_2;
    this->ENCODER_RESOLUTION = RESOLUTION;
    this->WAIT_RESPONSE_TIME = wait_time;

    pinMode(this->MOTOR_OUT_1_PIN, OUTPUT);
    pinMode(this->MOTOR_OUT_2_PIN, OUTPUT);
    pinMode(this->MOTOR_PWM_PIN,   OUTPUT);
    pinMode(this->ENCODER_CHANNEL_A, INPUT);
    pinMode(this->ENCODER_CHANNEL_B, INPUT);

    digitalWrite(this->MOTOR_OUT_1_PIN, LOW);
    digitalWrite(this->MOTOR_OUT_2_PIN, LOW);
    analogWrite(this->MOTOR_PWM_PIN, 0);

    direction                = 0;
    encoder_count            = 0;
    encoder_calculation_flag = false;
    responded_speed          = 0.0f;
    timer                    = millis();
    rotate_flag              = false;
}

// Enable drive
void Actuators::Drive::enable()
{
    this->available = true;
}

// Disable drive
void Actuators::Drive::disable()
{
    available = false;
    direction = 0;
    change_direction();
    analogWrite(MOTOR_PWM_PIN, 0);
}

// Brake
void Actuators::Drive::brake()
{
    // Simple electrical brake
    digitalWrite(MOTOR_OUT_1_PIN, true);
    digitalWrite(MOTOR_OUT_2_PIN, true);
    analogWrite(MOTOR_PWM_PIN, 0);
    delay(50);
    direction = 0;
    change_direction();
}

// Start to control speed
bool Actuators::Drive::driving(uint8_t speed, bool direction_fwd)
{
    if (!available)
    {
        return false;
    }

    // If speed = 0 -> stop
    if (speed == 0)
    {
        direction = 0;
        change_direction();
        analogWrite(MOTOR_PWM_PIN, 0);
        return true;
    }

    analogWrite(MOTOR_PWM_PIN, speed);
    update_direction(direction_fwd);
    return true;
}

// Interrupt function for calculation of Encoder's channel A
void Actuators::Drive::Endcoder_channel_A_ISR()
{
    if (digitalRead(ENCODER_CHANNEL_A) == digitalRead(ENCODER_CHANNEL_B))
    {
        encoder_count++; // Increase encoder pulse accumulate
    }
    else
    {
        encoder_count--; // Decrease encoder pulse accumulate
    }

    unsigned long tmr = millis(); // Take instance time

    if (encoder_calculation_flag)
    {
        unsigned long dt = tmr - timer;
        if (dt > 0)
        {
            // Pulses per second = ENCODER_RESOLUTION * 1000 / dt
            responded_speed = (float)ENCODER_RESOLUTION * 1000.0f / (float)dt;
        }
    }
    else
    {
        encoder_calculation_flag = true;
    }

    timer       = tmr;
    rotate_flag = true;
}

// Interrupt function for calculation of Encoder's channel B
void Actuators::Drive::Endcoder_channel_B_ISR()
{
    if (digitalRead(ENCODER_CHANNEL_A) != digitalRead(ENCODER_CHANNEL_B))
    {
        encoder_count++; // Increase encoder pulse accumulate
    }
    else
    {
        encoder_count--; // Decrease encoder pulse accumulate
    }

    unsigned long tmr = millis(); // Take instance time

    if (encoder_calculation_flag)
    {
        unsigned long dt = tmr - timer;
        if (dt > 0)
        {
            responded_speed = (float)ENCODER_RESOLUTION * 1000.0f / (float)dt;
        }
    }
    else
    {
        encoder_calculation_flag = true;
    }

    timer       = tmr;
    rotate_flag = true;
}

/** =========================================================================================================
 *                                          Steer function
   =========================================================================================================*/

Actuators::Steer::Steer(uint8_t STEERING_PIN)
{
    this->STEERING_PIN = STEERING_PIN;
    servo.attach(this->STEERING_PIN);
}

int Actuators::Steer::saturation(int angle_deg)
{
    if (angle_deg < low_limit)
    {
        return low_limit;
    }
    else if (angle_deg > high_limit)
    {
        return high_limit;
    }
    return angle_deg;
}

// Turn servo by angle in degrees
void Actuators::Steer::turn(int angle_deg)
{
    servo.write(saturation(angle_deg));
}

// Enable
void Actuators::Steer::enable()
{
    if (!servo.attached())
    {
        servo.attach(STEERING_PIN);
    }
}

// Disable
void Actuators::Steer::disable()
{
    if (servo.attached())
    {
        servo.detach();
    }
}

// encoder_pulse_total phải được khai báo ở main.cpp
extern volatile int32_t encoder_pulse_total;
static uint8_t  N = 1;
static float    rpmBuf[MAX_N];
static uint8_t  idx = 0;
static float    rpmFilt = 0.0f;


namespace Encoder
{
    // ===== Cấu hình & trạng thái nội bộ =====

    // Giới hạn cửa sổ lọc
    static constexpr uint8_t WIN_MIN = 1;
    static constexpr uint8_t WIN_MAX = 32;

    // PPR của encoder
    static uint16_t g_encoderPPR = 1;

    // Bộ nhớ moving average cho tốc độ (rpm)
    static float    g_speedBuf[WIN_MAX];
    static uint8_t  g_winLen   = 1;      // N hiện tại
    static uint8_t  g_index    = 0;      // vị trí ghi tiếp theo
    static uint8_t  g_count    = 0;      // số mẫu hợp lệ hiện có
    static float    g_sum      = 0.0f;   // tổng rpm trong cửa sổ

    // Tổng count lần trước để tính delta
    static int32_t  g_lastCount = 0;

    // Tốc độ đã lọc
    static float    g_speed_rpm_filt = 0.0f;

    // ===== Hàm nội bộ =====

    static void resetFilter(uint8_t N)
    {
        if (N < WIN_MIN) N = WIN_MIN;
        if (N > WIN_MAX) N = WIN_MAX;

        g_winLen = N;
        g_index  = 0;
        g_count  = 0;
        g_sum    = 0.0f;

        for (uint8_t i = 0; i < WIN_MAX; ++i)
            g_speedBuf[i] = 0.0f;
    }

    // Cập nhật moving average 1D cho rpm_raw
    static float filter_update(float rpm_raw)
    {
        uint8_t N = g_winLen;

        // N = 1 => không lọc
        if (N <= 1)
        {
            g_speedBuf[0]    = rpm_raw;
            g_speed_rpm_filt = rpm_raw;
            g_sum            = rpm_raw;
            g_count          = 1;
            g_index          = 0;
            return rpm_raw;
        }

        if (g_count < N)
        {
            // Giai đoạn fill cửa sổ
            g_sum += rpm_raw;
            g_speedBuf[g_index] = rpm_raw;

            g_index++;
            if (g_index >= N)
                g_index = 0;

            g_count++;
        }
        else
        {
            // Đã đủ N mẫu: bỏ mẫu cũ, thêm mẫu mới
            float old = g_speedBuf[g_index];
            g_sum -= old;
            g_speedBuf[g_index] = rpm_raw;
            g_sum += rpm_raw;

            g_index++;
            if (g_index >= N)
                g_index = 0;
        }

        float denom = (g_count < N) ? (float)g_count : (float)N;
        if (denom <= 0.0f)
        {
            g_speed_rpm_filt = rpm_raw;
            return rpm_raw;
        }

        g_speed_rpm_filt = g_sum / denom;
        return g_speed_rpm_filt;
    }

    // ===== API public =====

    void speed_init(uint16_t encoderPPR)
    {
        if (encoderPPR == 0)
            encoderPPR = 1;

        g_encoderPPR    = encoderPPR;
        g_lastCount     = 0;
        g_speed_rpm_filt = 0.0f;

        resetFilter(1);   // mặc định N = 1 (không lọc)
    }

    void speed_setFilterWindow(uint8_t N)
    {
        resetFilter(N);
    }

    uint8_t speed_getFilterWindow()
    {
        return g_winLen;
    }

    void speed_update_ms(uint32_t dt_ms)
    {
        if (dt_ms == 0)
            return;

        int32_t total_now;

        // Đọc encoder_pulse_total an toàn
        noInterrupts();
        total_now = encoder_pulse_total;
        interrupts();

        int32_t delta = total_now - g_lastCount;
        g_lastCount   = total_now;

        // pulses/s
        float pulses_per_s = (float)delta * (1000.0f / (float)dt_ms);
        // rev/s
        float rev_per_s    = pulses_per_s / (float)g_encoderPPR;
        // rpm
        float rpm_raw      = rev_per_s * 60.0f;
        if (rpm_raw < 0.0f)
            rpm_raw = 0.0f;

        filter_update(rpm_raw);
    }

    float speed_get_rpm()
    {
        return g_speed_rpm_filt;
    }
}
