#include <Arduino.h>
// #include <Wire.h>

/* Line sensor */
void line_setup(uint8_t LINE_SENSOR_PIN_1,
                uint8_t LINE_SENSOR_PIN_2,
                uint8_t LINE_SENSOR_PIN_3,
                uint8_t LINE_SENSOR_PIN_4,
                uint8_t LINE_SENSOR_PIN_5);
uint8_t* line_readSignals(); // get ră uint8 signals

/* Ultra-sonics sensor */
void ultra_setup(uint8_t ULTRA_TRIG_PIN,uint8_t ULTRA_ECHO_PIN);
void ultra_kick();
void ultra_setHigh();
void ultra_setLow();
void IRAM_ATTR hanlder_ultra_echo();
uint16_t ultra_getSignal();

/* MPU6050 */
// void mpu_setup(uint8_t MPU_SDA_PIN,
//                     uint8_t MPU_SCL_PIN);
// uint16_t* mpu_getSignals(); // Return raw signals
// uint16_t* mpu_getSignals(size_t delta_t); // Return processed signals
