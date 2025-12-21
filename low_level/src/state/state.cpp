#include "state.hpp"

namespace State {

static volatile Mode g_mode = Mode::IDLE;

Mode mode()
{
    return g_mode;
}

void setMode(Mode m)
{
    g_mode = m;
}

} // namespace State
