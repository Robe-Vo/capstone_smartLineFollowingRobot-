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

namespace Encoder {

    struct SpeedFilterState {
        float   buffer[SPEED_FILTER_MAX_WINDOW];
        uint8_t windowLen;   // N thực tế đang dùng
        uint8_t index;       // vị trí ghi tiếp theo
        uint8_t count;       // số mẫu hợp lệ hiện có (<= windowLen)
        float   sum;         // tổng các mẫu trong cửa sổ
    };

    static SpeedFilterState speedFilter;

    // Chuẩn hóa windowLen và reset trạng thái bộ lọc
    static void resetFilterState(uint8_t windowLen)
    {
        if (windowLen < SPEED_FILTER_MIN_WINDOW) {
            windowLen = SPEED_FILTER_MIN_WINDOW;
        }
        if (windowLen > SPEED_FILTER_MAX_WINDOW) {
            windowLen = SPEED_FILTER_MAX_WINDOW;
        }

        speedFilter.windowLen = windowLen;
        speedFilter.index     = 0;
        speedFilter.count     = 0;
        speedFilter.sum       = 0.0f;

        for (uint8_t i = 0; i < SPEED_FILTER_MAX_WINDOW; ++i) {
            speedFilter.buffer[i] = 0.0f;
        }
    }

    void initSpeedFilter(uint8_t windowLen)
    {
        resetFilterState(windowLen);
    }

    void setSpeedFilterWindow(uint8_t windowLen)
    {
        // Mỗi lần đổi N → reset để tránh sum bị lệch
        resetFilterState(windowLen);
    }

    uint8_t getSpeedFilterWindow()
    {
        return speedFilter.windowLen;
    }

    float updateSpeedFilter(float rpm_sample)
    {
        uint8_t N = speedFilter.windowLen;

        // Trường hợp N = 1: không lọc, trả về trực tiếp
        if (N <= 1) {
            speedFilter.buffer[0] = rpm_sample;
            speedFilter.sum       = rpm_sample;
            speedFilter.count     = 1;
            speedFilter.index     = 0;
            return rpm_sample;
        }

        if (speedFilter.count < N) {
            // Đang fill cửa sổ lần đầu
            speedFilter.sum += rpm_sample;
            speedFilter.buffer[speedFilter.index] = rpm_sample;

            speedFilter.index++;
            if (speedFilter.index >= N) {
                speedFilter.index = 0;
            }

            speedFilter.count++;
        } else {
            // Đã đủ N mẫu: thay thế mẫu cũ nhất
            float oldSample = speedFilter.buffer[speedFilter.index];
            speedFilter.sum -= oldSample;
            speedFilter.buffer[speedFilter.index] = rpm_sample;
            speedFilter.sum += rpm_sample;

            speedFilter.index++;
            if (speedFilter.index >= N) {
                speedFilter.index = 0;
            }
        }

        float denom = (speedFilter.count < N) ? static_cast<float>(speedFilter.count)
                                            : static_cast<float>(N);

        if (denom <= 0.0f) {
            return rpm_sample;
        }

        return speedFilter.sum / denom;
    }

} // namespace Actuator
