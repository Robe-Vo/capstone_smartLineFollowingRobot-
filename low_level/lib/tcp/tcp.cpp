#include "tcp.hpp"

Network::Network(uint16_t port)
    : server(port), port(port)
{
}

bool Network::begin()
{
    Serial.println("Connecting to WiFi...");
    WiFi.mode(WIFI_STA);

    // If you were using WiFi.config() with a fixed 192.168.4.x IP, remove it
    // unless your PC is really in that subnet.
    // WiFi.config(local_IP, gateway, subnet);
    WiFi.setSleep(false);
    WiFi.begin(ssid, password);

    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 15000)
    {
        delay(500);
        Serial.print(".");
    }
    Serial.println();

    if (WiFi.status() != WL_CONNECTED)
    {
        Serial.println("WiFi connect FAILED");
        return false;
    }

    Serial.print("WiFi connected, IP = ");
    Serial.println(WiFi.localIP());

    server.begin();
    server.setNoDelay(true);
    Serial.print("TCP server started on port ");
    Serial.println(port);

    return true;
}

// Ensure we have a connected client
bool Network::ensureClient()
{
    // If we already have a connected client, use it
    if (client && client.connected())
        return true;

    // Otherwise, try to accept a new one
    WiFiClient newClient = server.available();
    if (!newClient)
        return false;   // no incoming client yet

    // We have a real client now – enable low-latency TCP
    newClient.setNoDelay(true);

    client = newClient;          // copy handle into member
    Serial.println("Client connected");
    return true;
}


bool Network::getUint8(uint8_t &buffer)
{
    if (!ensureClient())
        return false;

    if (!client.connected())
        return false;

    if (!client.available())
        return false;

    int c = client.read();
    if (c < 0)
        return false;

    buffer = static_cast<uint8_t>(c);
    return true;
}

bool Network::getArrayUint8(uint8_t* buffer, size_t num)
{
    if (!buffer || num == 0)
        return false;

    if (!ensureClient())
        return false;

    if (!client.connected())
        return false;

    if (client.available() < static_cast<int>(num))
        return false;   // not enough data yet

    int readCount = client.read(buffer, num);
    return (readCount == static_cast<int>(num));
}

bool Network::transmitArrayUint8(const uint8_t* buffer, size_t num)
{
    if (!buffer || num == 0)
        return false;

    if (!ensureClient())
        return false;

    if (!client.connected())
        return false;

    size_t written = client.write(buffer, num);

    if (written != num)
    {
        Serial.println("Client write failed, stopping client.");
        client.stop();      // close broken connection
        return false;
    }

    // Optional: only flush when you care about latency
    // client.flush();

    Serial.print("TX ");
    Serial.print(written);
    Serial.print("/");
    Serial.print(num);
    Serial.println(" bytes");

    return true;
}

bool Network::transmitUint8(uint8_t byte)
{
    if (!ensureClient())
        return false;

    if (!client.connected())
        return false;

    size_t written = client.write(&byte, 1);

    if (written != 1)
    {
        Serial.println("Client write failed (1 byte), stopping client.");
        client.stop();
        return false;
    }

    Serial.print("TX 1 byte: 0x");
    Serial.println(byte, HEX);
    return true;
}
