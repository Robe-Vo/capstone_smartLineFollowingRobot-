#pragma once
#include <Arduino.h>
#include "dispatch.hpp"




namespace SystemHandlers
{
  void to_idle(const Protocol::RxFrame& f);
  void to_operation(const Protocol::RxFrame& f);
  void ping_mode(const Protocol::RxFrame& f);
  void emergency_stop(const Protocol::RxFrame& f);
}