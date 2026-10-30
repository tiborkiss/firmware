#pragma once
#include <cassert>
#include <cstdint>
#include <vector>

constexpr int OUTPUT = 1;
constexpr int HIGH = 1;
constexpr int LOW = 0;
constexpr int MSBFIRST = 1;
constexpr int SPI_MODE0 = 0;
inline int pins[48] = {};
inline std::vector<unsigned> delays;
inline void pinMode(unsigned pin, int)
{
    assert(pin == 17 || pin == 19 || pin == 20);
}
inline void digitalWrite(unsigned pin, int value)
{
    assert(pin == 17 || pin == 19 || pin == 20);
    pins[pin] = value;
}
inline void delay(unsigned ms)
{
    delays.push_back(ms);
}
inline void yield() {}

struct SPISettings {
    uint32_t hz;
    SPISettings(uint32_t hz, int order, int mode) : hz(hz) { assert(order == MSBFIRST && mode == SPI_MODE0); }
};

class SPIClass
{
  public:
    struct Packet {
        uint8_t command;
        std::vector<uint8_t> data;
    };
    std::vector<Packet> packets;
    unsigned starts = 0;
    bool transaction = false;
    void begin() { starts++; }
    void beginTransaction(const SPISettings &settings)
    {
        assert(starts > 0 && !transaction && settings.hz == 4000000);
        transaction = true;
    }
    void endTransaction()
    {
        assert(transaction && pins[20] == HIGH);
        transaction = false;
    }
    void transfer(uint8_t value)
    {
        assert(transaction && pins[20] == LOW);
        if (pins[19] == LOW)
            packets.push_back({value, {}});
        else {
            assert(!packets.empty());
            packets.back().data.push_back(value);
        }
    }
    void transfer(void *buffer, void *, uint32_t size)
    {
        auto bytes = static_cast<uint8_t *>(buffer);
        for (uint32_t i = 0; i < size; i++)
            transfer(bytes[i]);
    }
};
