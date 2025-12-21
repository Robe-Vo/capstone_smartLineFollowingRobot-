#pragma once
#include <Arduino.h>

namespace State {

enum class Mode : uint8_t {
    IDLE      = 0,
    OPERATION = 1,
};

Mode mode();
void setMode(Mode m);

// tiện dùng trong ISR / app
inline bool isOperation() {
    return mode() == Mode::OPERATION;
}

} // namespace State
