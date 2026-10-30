#pragma once
#include <algorithm>
#include <cstdint>
#include <vector>
using std::max;
using std::min;
enum OLEDDISPLAY_GEOMETRY { GEOMETRY_RAWMODE };

// Minimal bitmap owner; these tests exercise the production SPI driver, not OLEDDisplay itself.
class OLEDDisplay
{
  public:
    virtual ~OLEDDisplay() = default;
    virtual bool connect() { return false; }
    virtual void display() = 0;
    uint16_t displayWidth = 0;
    uint16_t displayHeight = 0;
    std::vector<uint8_t> front;
    std::vector<uint8_t> back;
    uint8_t *buffer = nullptr;
    uint8_t *buffer_back = nullptr;
    void setGeometry(OLEDDISPLAY_GEOMETRY, uint16_t width, uint16_t height)
    {
        displayWidth = width;
        displayHeight = height;
        front.resize(width * ((height + 7) / 8));
        back.resize(front.size());
        buffer = front.data();
        buffer_back = back.data();
    }

  protected:
    virtual void sendInitCommands() {}
    virtual void sendCommand(uint8_t) {}
    virtual int getBufferOffset() { return 0; }
};
