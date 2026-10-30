// Exercises the production NhdST7789Spi.h against a recording SPI/GPIO boundary.
// Pins the Newhaven register contract, inclusive edge addressing, dirty-page stride,
// color-only refresh and five blank/wake cycles. Regressions here can compile yet
// send OLED commands, centered windows, missing rows or stale colors to the panel.
// This is a host protocol test, not evidence of panel orientation or electrical timing.
#include "NhdST7789Spi.h"
#include <iostream>

class TestPanel : public ST7789Spi
{
  public:
    using ST7789Spi::sendInitCommands;
    using ST7789Spi::ST7789Spi;
};

static void expectPacket(const SPIClass::Packet &packet, uint8_t command, std::initializer_list<uint8_t> data)
{
    assert(packet.command == command && packet.data == std::vector<uint8_t>(data));
}

static void test_initialization_and_edges()
{
    SPIClass spi;
    TestPanel panel(&spi, 17, 19, 20, GEOMETRY_RAWMODE, 240, 240, 4000000, 0, 0, 0x88);
    assert(panel.connect());
    panel.sendInitCommands();
    assert((delays == std::vector<unsigned>{250, 250, 120, 10, 10}));
    assert(spi.packets.size() == 20);
    expectPacket(spi.packets[0], 0x28, {});
    expectPacket(spi.packets[1], 0x11, {});
    expectPacket(spi.packets[2], 0x36, {0x88});
    expectPacket(spi.packets[3], 0x3A, {0x55});
    expectPacket(spi.packets[4], 0xB2, {0x0C, 0x0C, 0, 0x33, 0x33});
    expectPacket(spi.packets[5], 0xB7, {0x35});
    expectPacket(spi.packets[6], 0xBB, {0x2B});
    expectPacket(spi.packets[7], 0xC0, {0x2C});
    expectPacket(spi.packets[8], 0xC2, {1, 0xFF});
    expectPacket(spi.packets[9], 0xC3, {0x11});
    expectPacket(spi.packets[10], 0xC4, {0x20});
    expectPacket(spi.packets[11], 0xC6, {0x0F});
    expectPacket(spi.packets[12], 0xD0, {0xA4, 0xA1});
    expectPacket(spi.packets[13], 0xE0, {0xD0, 0, 5, 0x0E, 0x15, 0x0D, 0x37, 0x43, 0x47, 9, 0x15, 0x12, 0x16, 0x19});
    expectPacket(spi.packets[14], 0xE1, {0xD0, 0, 5, 0x0D, 0x0C, 6, 0x2D, 0x44, 0x40, 0x0E, 0x1C, 0x18, 0x16, 0x19});
    expectPacket(spi.packets[15], 0x2A, {0, 0, 0, 239});
    expectPacket(spi.packets[16], 0x2B, {0, 0, 1, 63});
    expectPacket(spi.packets[17], 0x34, {});
    expectPacket(spi.packets[18], 0x21, {});
    expectPacket(spi.packets[19], 0x29, {});
}

static void test_render_and_wake()
{
    SPIClass spi;
    TestPanel panel(&spi, 17, 19, 20, GEOMETRY_RAWMODE, 240, 240, 4000000, 0, 0, 0x88);
    panel.connect();
    panel.setRGB(0xFFFF);
    panel.front.front() = 1;
    panel.front.back() = 0x80;
    panel.display();
    assert(spi.packets.size() == 3);
    expectPacket(spi.packets[0], 0x2A, {0, 0, 0, 239});
    expectPacket(spi.packets[1], 0x2B, {0, 0, 0, 239});
    const auto &pixels = spi.packets[2].data;
    assert(pixels.size() == 240 * 240 * 2);
    assert(pixels[0] == 255 && pixels[1] == 255 && pixels[2] == 0);
    assert(pixels[pixels.size() - 2] == 255 && pixels.back() == 255);
    spi.packets.clear();
    panel.front.back() = 0;
    panel.display();
#ifdef OLEDDISPLAY_DOUBLE_BUFFER
    expectPacket(spi.packets[0], 0x2A, {0, 239, 0, 239});
    expectPacket(spi.packets[1], 0x2B, {0, 232, 0, 239});
    assert(spi.packets[2].data.size() == 16);
    spi.packets.clear();
    panel.display();
    assert(spi.packets.empty());
#endif
    TFTColorRegion regions[3] = {{0, 0, 240, 240, 0xFFFF, 0x1234, true}, {239, 239, 1, 1, 0xFFFF, 0x5678, true}, {}};
    panel.setRGB(0xFFFF, regions);
    spi.packets.clear();
    panel.display();
    assert(spi.packets[2].data[2] == 0x34 && spi.packets[2].data.back() == 0x56);
    for (unsigned i = 0; i < 5; i++) {
        spi.packets.clear();
        panel.displayOff();
        panel.displayOn();
        expectPacket(spi.packets[0], 0x28, {});
        expectPacket(spi.packets[1], 0x29, {});
        panel.front[10] ^= 1;
        panel.display();
        assert(spi.packets.size() == 5 && spi.starts == 1);
    }
    spi.packets.clear();
    panel.flipScreenVertically();
    expectPacket(spi.packets[0], 0x36, {0x48});
    panel.resetOrientation();
    expectPacket(spi.packets[1], 0x36, {0x88});
}

static void test_offset_and_partial_page()
{
    SPIClass spi;
    TestPanel panel(&spi, 17, 19, 20, GEOMETRY_RAWMODE, 240, 239, 4000000, 0, 80, 0x88);
    panel.connect();
    panel.display();
    expectPacket(spi.packets[1], 0x2B, {0, 80, 1, 62});
    assert(spi.packets[2].data.size() == 240 * 239 * 2);
}

int main()
{
    test_initialization_and_edges();
    test_render_and_wake();
    test_offset_and_partial_page();
    std::cout << "PASS: Newhaven init, windows, pixel stride, color regions, orientation commands, five blank/wake cycles\n";
}
