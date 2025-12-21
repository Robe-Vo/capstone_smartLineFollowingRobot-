#include <Arduino.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "system/system.hpp"

// Action
void setup()
{
    // Init
    Serial.begin(115200);
    robot_init();

    // Init interrupt
    xTaskCreatePinnedToCore(robot_calculation, "ctrl", 4096, nullptr, 4, &control_handle      , 1);
    xTaskCreatePinnedToCore(robot_calculation, "comm", 4096, nullptr, 2, &communication_handle, 0);
}


void loop()
{

}