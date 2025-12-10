#include "udp.hpp"

Network::Network(uint16_t port)
    : port(port)
{
}

bool Network::begin()
{
    Serial.println("Connecting to WiFi (UDP mode)...");
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);

    WiFi.begin(ssid, password);

    unsigned long t0 = millis();
    const unsigned long timeoutMs = 15000; // 15 s

    while (WiFi.status() != WL_CONNECTED)
    {
        if (millis() - t0 > timeoutMs)
        {
            Serial.println("WiFi connect FAILED (timeout).");
            return false;
        }
        delay(250);
        Serial.print('.');
    }
    Serial.println();
    Serial.print("WiFi connected. IP: ");
    Serial.println(WiFi.localIP());

    // Start UDP listener on given port
    if (!udp.begin(port))
    {
        Serial.println("UDP begin FAILED.");
        return false;
    }

    Serial.print("UDP listening on port ");
    Serial.println(port);
    return true;
}

bool Network::ensureRemote() const
{
    if (!haveRemote)
    {
        // No remote endpoint has sent us anything yet
        return false;
    }
    return true;
}

// ---------------------------------------------------------------------
// Receive 1 byte (non-blocking)
// ---------------------------------------------------------------------
bool Network::getUint8(uint8_t &buffer)
{
    // If there is no data in current buffer, try to parse next packet
    if (udp.available() <= 0)
    {
        int packetSize = udp.parsePacket();
        if (packetSize <= 0)
        {
            // No packet available
            return false;
        }

        // Learn remote endpoint from this packet
        remoteIP   = udp.remoteIP();
        remotePort = udp.remotePort();
        haveRemote = true;
    }

    int c = udp.read();
    if (c < 0)
    {
        return false;
    }

    buffer = static_cast<uint8_t>(c);
    return true;
}

// ---------------------------------------------------------------------
// Receive exactly 'num' bytes in a single UDP datagram (non-blocking)
// Returns false if no packet, or size mismatch.
// This is usually what you want for "frames" (e.g. 19-byte command frame).
// ---------------------------------------------------------------------
bool Network::getArrayUint8(uint8_t* buffer, size_t num)
{
    if (!buffer || num == 0)
        return false;

    // Try to parse next UDP packet (non-blocking)
    int packetSize = udp.parsePacket();
    if (packetSize <= 0)
    {
        // No packet waiting
        return false;
    }

    // Learn remote endpoint
    remoteIP   = udp.remoteIP();
    remotePort = udp.remotePort();
    haveRemote = true;

    // If packet size is not what we expect, read & discard it
    if (packetSize != static_cast<int>(num))
    {
        uint8_t dumpBuf[64];
        while (udp.available() > 0)
        {
            int n = udp.read(dumpBuf, sizeof(dumpBuf));
            if (n <= 0) break;
        }
        // Size mismatch -> not our frame
        return false;
    }

    // Read exact number of bytes
    int readCount = udp.read(buffer, num);
    return (readCount == static_cast<int>(num));
}

// ---------------------------------------------------------------------
// Transmit 'num' bytes to last known remote endpoint over UDP
// ---------------------------------------------------------------------
bool Network::transmitArrayUint8(const uint8_t* buffer, size_t num)
{
    if (!buffer || num == 0)
        return false;

    if (!ensureRemote())
    {
        // Nobody has talked to us yet -> we don't know where to send
        Serial.println("UDP TX failed: no remote endpoint yet.");
        return false;
    }

    if (udp.beginPacket(remoteIP, remotePort) == 0)
    {
        Serial.println("udp.beginPacket() failed.");
        return false;
    }

    size_t written = udp.write(buffer, num);
    if (written != num)
    {
        Serial.println("UDP write() incomplete.");
        // We still need to endPacket() to flush / cancel
    }

    if (udp.endPacket() == 0)
    {
        Serial.println("udp.endPacket() failed.");
        return false;
    }

    // Uncomment if you want debug:
    // Serial.print("UDP TX "); Serial.print(written);
    // Serial.print("/"); Serial.print(num); Serial.println(" bytes");

    return (written == num);
}

// ---------------------------------------------------------------------
// Transmit a single byte
// ---------------------------------------------------------------------
bool Network::transmitUint8(uint8_t byte)
{
    return transmitArrayUint8(&byte, 1);
}
