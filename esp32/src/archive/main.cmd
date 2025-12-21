// ======================= src/main.cpp =======================
// Behavior:
// - Read 1-byte CMD + payload using Protocol::tryRead()
// - If cmd invalid (Group::UNKNOWN) -> print "INVALID CMD" to Serial Monitor
// - If valid -> print group/cmd + parsed payload fields
// - Always send ACK (0x20) back over Bluetooth for MATLAB test

#include <Arduino.h>

#include "bluetooth.hpp"
#include "protocol.hpp"   // your descriptor-table protocol

static Network g_net;

static const char* groupName(Protocol::Group g)
{
  switch (g) {
    case Protocol::Group::SYSTEM:    return "SYSTEM";
    case Protocol::Group::IDLE:      return "IDLE";
    case Protocol::Group::OPERATION: return "OPERATION";
    default:                         return "UNKNOWN";
  }
}

static const char* payloadName(Protocol::Payload p)
{
  switch (p) {
    case Protocol::Payload::NONE:    return "NONE";
    case Protocol::Payload::U16_LE:  return "U16_LE";
    case Protocol::Payload::I16_LE:  return "I16_LE";
    case Protocol::Payload::BYTES30: return "BYTES30";
    case Protocol::Payload::OP_PWM:  return "OP_PWM(u16+i16)";
    case Protocol::Payload::OP_SPD:  return "OP_SPD(speed+i16)";
    default:                         return "UNK";
  }
}

// Print payload content (for debug only)
static void printFrame(const Protocol::RxFrame& f)
{
  const uint8_t cmd_u8 = (uint8_t)f.cmd;

  Serial.printf("[RX] cmd=0x%02X group=%s payload=%s len=%u\n",
                (unsigned)cmd_u8, groupName(f.group), payloadName(f.payload),
                (unsigned)Protocol::payloadLen(cmd_u8));

  switch (f.payload) {
    case Protocol::Payload::NONE:
      break;

    case Protocol::Payload::U16_LE:
      Serial.printf("     u16=%u\n", (unsigned)f.u16);
      break;

    case Protocol::Payload::I16_LE:
      Serial.printf("     i16=%d\n", (int)f.i16);
      break;

    case Protocol::Payload::BYTES30:
      Serial.print("     bytes30[0..7]=");
      for (int i = 0; i < 8; ++i) {
        Serial.printf("%02X ", (unsigned)f.bytes30[i]);
      }
      Serial.println();
      break;

    case Protocol::Payload::OP_PWM:
      Serial.printf("     pwm_u16=%u (11-bit), angle_i16=%d\n",
                    (unsigned)f.pwm_u16, (int)f.angle_i16);
      break;

    case Protocol::Payload::OP_SPD:
      Serial.printf("     speed_f=%.3f, angle_i16=%d\n",
                    (double)f.speed_f, (int)f.angle_i16);
      break;

    default:
      break;
  }
}

void setup()
{
  Serial.begin(115200);
  delay(200);

  // Bluetooth SPP network
  g_net.begin();

  // Descriptor table must be initialized once
  Protocol::initDescriptorTable();

  Serial.println("=== CMD RX DEBUG STARTED ===");
  Serial.println("Send cmd frames via Bluetooth, check prints here.");
}

void loop()
{
  Protocol::RxFrame f{};

  // Drain all available frames
  while (Protocol::tryRead(g_net, f))
  {
    // Invalid cmd/group
    if (f.group == Protocol::Group::UNKNOWN) {
      Serial.printf("[RX] INVALID CMD: 0x%02X (Group::UNKNOWN)\n", (unsigned)(uint8_t)f.cmd);
      // still send ACK so MATLAB sees "received" and can move on
      g_net.transmitUint8(Protocol::ACK_IDLE);
      continue;
    }

    // Valid frame
    printFrame(f);

    // Here is the place you would execute each cmd:
    // - SYSTEM: handleSystemCmd(f)
    // - IDLE: handleIdleCmd(f)
    // - OPERATION: handleOperationCmd(f)

    // For now: only ACK back for testing
    g_net.transmitUint8(Protocol::ACK_IDLE);
  }

  delay(2);
}
