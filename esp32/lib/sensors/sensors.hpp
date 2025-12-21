#include <Arduino.h>
// #include <Wire.h>

#include "../../include/pin.hpp"
#include "../../include/cfg.hpp"


// General function
void sensors_setup();

/* Line sensor */
void line_setup();
uint8_t* line_readSignals(); // get ră uint8 signals

/* Ultra-sonics sensor */
void ultra_setup();
void ultra_kick();
void ultra_setHigh();
void ultra_setLow();
void IRAM_ATTR hanlder_ultra_echo();
uint16_t ultra_getSignal();
void ultra_disable_isr();
void ultra_enable_isr();

/* MPU6050 */
// void mpu_setup(uint8_t MPU_SDA_PIN,
//                     uint8_t MPU_SCL_PIN);
// uint16_t* mpu_getSignals(); // Return raw signals
// uint16_t* mpu_getSignals(size_t delta_t); // Return processed signals
