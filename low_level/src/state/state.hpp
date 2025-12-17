#include <Arduino.h>

namespace State {
    enum class Mode: uint8_t 
    {
        IDLE = 0,
        OPERATION = 1,
    };
    

} // namespace State