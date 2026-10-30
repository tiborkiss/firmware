/**
 * The MIT License (MIT)
 *
 * Copyright (c) 2018 by ThingPulse, Daniel Eichhorn
 * Copyright (c) 2018 by Fabrice Weinberg
 * Copyright (c) 2024 by Heltec AutoMation
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

// Adapted from meshtastic/st7789@92bae2e4a307afb430c3b0bc3d661c55ee1565f0; see UPSTREAM.
#pragma once

#ifdef WIO_ST7789_LCD_BENCH
#include "OLEDDisplay.h"
#include <SPI.h>
#include <cstdlib>
#include <initializer_list>

struct TFTColorRegion {
    int16_t x;
    int16_t y;
    int16_t width;
    int16_t height;
    uint16_t onColorBe;
    uint16_t offColorBe;
    bool enabled = false;
};

class ST7789Spi : public OLEDDisplay
{
  public:
    ST7789Spi(SPIClass *spi, uint8_t rst, uint8_t dc, uint8_t cs, OLEDDISPLAY_GEOMETRY geometry, uint16_t width, uint16_t height,
              uint32_t clockHz, uint16_t xOffset, uint16_t yOffset, uint8_t panelMadctl)
        : _rst(rst), _dc(dc), _cs(cs), _spi(spi), _spiSettings(clockHz, MSBFIRST, SPI_MODE0), _xOffset(xOffset),
          _yOffset(yOffset), _panelMadctl(panelMadctl), _madctl(panelMadctl)
    {
        setGeometry(geometry, width, height);
        setRGB(0x07E0);
    }

    ~ST7789Spi() override
    {
        free(_scanlineBuf);
        free(_rowOnColors);
        free(_rowOffColors);
    }

    bool connect() override
    {
        _buffheight = (displayHeight + 7) / 8;
        pinMode(_cs, OUTPUT);
        digitalWrite(_cs, HIGH);
        pinMode(_dc, OUTPUT);
        digitalWrite(_dc, HIGH);
        pinMode(_rst, OUTPUT);
        _spi->begin();
        digitalWrite(_rst, LOW);
        delay(250);
        digitalWrite(_rst, HIGH);
        delay(250);
        _forceRefresh = true;
        return true;
    }

    void display() override
    {
        // Allocate before committing the dirty bitmap: an allocation failure must remain retryable.
        ensureScanlineBuffer(displayWidth);
        if (_scanlineBuf == nullptr)
            return;

        const uint16_t fallbackOnColorBe = _rgb;
        const uint16_t fallbackOffColorBe = 0x0000;
        _activeColorRegionCount = countColorRegions();
        const bool hasActiveColorRegions = (_activeColorRegionCount > 0);
        const uint32_t colorRegionHash = hasActiveColorRegions ? hashColorRegions() : 0;
        const bool colorRegionStateChanged = (hasActiveColorRegions != _hadColorRegionsLastFrame) ||
                                             (hasActiveColorRegions && colorRegionHash != _lastColorRegionHash);
        _hadColorRegionsLastFrame = hasActiveColorRegions;
        _lastColorRegionHash = colorRegionHash;

        uint16_t minBoundX = 0;
        uint16_t maxBoundX = displayWidth - 1;
        uint16_t minBoundY = 0;
        uint16_t maxBoundY = _buffheight - 1;
#ifdef OLEDDISPLAY_DOUBLE_BUFFER
        minBoundX = UINT16_MAX;
        maxBoundX = 0;
        minBoundY = UINT16_MAX;
        maxBoundY = 0;
        for (uint16_t y = 0; y < _buffheight; y++) {
            for (uint16_t x = 0; x < displayWidth; x++) {
                const uint16_t pos = x + y * displayWidth;
                if (buffer[pos] != buffer_back[pos]) {
                    minBoundY = min(minBoundY, y);
                    maxBoundY = max(maxBoundY, y);
                    minBoundX = min(minBoundX, x);
                    maxBoundX = max(maxBoundX, x);
                }
                buffer_back[pos] = buffer[pos];
            }
            yield();
        }
        if (minBoundY == UINT16_MAX && !colorRegionStateChanged && !_forceRefresh)
            return;
        if (colorRegionStateChanged || _forceRefresh) {
            minBoundY = 0;
            maxBoundY = _buffheight - 1;
            minBoundX = 0;
            maxBoundX = displayWidth - 1;
        }
#endif
        _forceRefresh = false;
        const uint16_t pixelCount = maxBoundX - minBoundX + 1;
        const bool useRowColorCache = (_activeColorRegionCount > 1);
        if (useRowColorCache)
            ensureRowColorBuffers(pixelCount);

        const uint16_t rowStart = minBoundY * 8;
        const uint16_t rowEndExclusive = min(displayHeight, static_cast<uint16_t>((maxBoundY + 1) * 8));
        _spi->beginTransaction(_spiSettings);
        digitalWrite(_cs, LOW);
        setAddrWindow(minBoundX, rowStart, pixelCount, rowEndExclusive - rowStart);
        for (uint16_t pixelY = rowStart; pixelY < rowEndExclusive; pixelY++) {
            const uint16_t srcY = pixelY >> 3;
            const uint8_t bitMask = static_cast<uint8_t>(1u << (pixelY & 0x07));
            const uint8_t *srcRow = &buffer[srcY * displayWidth];
            if (useRowColorCache && _rowOnColors != nullptr && _rowOffColors != nullptr) {
                prepareRowColors(pixelY, minBoundX, maxBoundX, fallbackOnColorBe, fallbackOffColorBe, _activeColorRegionCount);
                for (uint16_t x = minBoundX; x <= maxBoundX; x++) {
                    const uint16_t idx = x - minBoundX;
                    _scanlineBuf[idx] = (srcRow[x] & bitMask) ? _rowOnColors[idx] : _rowOffColors[idx];
                }
            } else {
                for (uint16_t x = minBoundX; x <= maxBoundX; x++) {
                    _scanlineBuf[x - minBoundX] = resolveTFTColorPixel(x, pixelY, (srcRow[x] & bitMask) != 0, fallbackOnColorBe,
                                                                       fallbackOffColorBe, _activeColorRegionCount);
                }
            }
            _spi->transfer(reinterpret_cast<void *>(_scanlineBuf), nullptr, static_cast<uint32_t>(2 * pixelCount));
        }
        digitalWrite(_cs, HIGH);
        _spi->endTransaction();
    }

    void resetOrientation() { setOrientation(_panelMadctl); }
    void flipScreenVertically() { setOrientation(_panelMadctl ^ 0xC0); }
    void mirrorScreen() { setOrientation(_panelMadctl ^ 0x40); }

    void setRGB(uint16_t color, TFTColorRegion *colorRegions = nullptr)
    {
        _colorRegions = colorRegions;
        const uint16_t colorBe = (color >> 8) | (color << 8);
        if (_rgb != colorBe)
            _forceRefresh = true;
        _rgb = colorBe;
    }

    void displayOn() { sendCommand(0x29); }
    void displayOff() { sendCommand(0x28); }

  protected:
    void sendInitCommands() override
    {
        // Newhaven product specification rev. 1, printed pp. 11-12; reset is in connect().
        sendCommand(0x28);
        sendCommand(0x11);
        delay(120);
        sendRegister(0x36, {_madctl});
        sendRegister(0x3A, {0x55});
        sendRegister(0xB2, {0x0C, 0x0C, 0x00, 0x33, 0x33});
        sendRegister(0xB7, {0x35});
        sendRegister(0xBB, {0x2B});
        sendRegister(0xC0, {0x2C});
        sendRegister(0xC2, {0x01, 0xFF});
        sendRegister(0xC3, {0x11});
        sendRegister(0xC4, {0x20});
        sendRegister(0xC6, {0x0F});
        sendRegister(0xD0, {0xA4, 0xA1});
        sendRegister(0xE0, {0xD0, 0x00, 0x05, 0x0E, 0x15, 0x0D, 0x37, 0x43, 0x47, 0x09, 0x15, 0x12, 0x16, 0x19});
        sendRegister(0xE1, {0xD0, 0x00, 0x05, 0x0D, 0x0C, 0x06, 0x2D, 0x44, 0x40, 0x0E, 0x1C, 0x18, 0x16, 0x19});
        sendRegister(0x2A, {0x00, 0x00, 0x00, 0xEF});
        sendRegister(0x2B, {0x00, 0x00, 0x01, 0x3F});
        delay(10);
        sendCommand(0x34); // TE output is unused on this assembly.
        sendCommand(0x21);
        sendCommand(0x29);
        delay(10);
#ifdef WIO_LCD_BOOT_FLASH
        // Bench visibility probe: solid fills prove init+bus+panel independently of the UI stack.
        uint16_t line[displayWidth > 0 ? displayWidth : 1];
        for (uint16_t c : {0xF800, 0x07E0, 0x001F, 0xFFFF}) {
            const uint16_t be = (c >> 8) | (c << 8);
            for (uint16_t i = 0; i < displayWidth; i++)
                line[i] = be;
            _spi->beginTransaction(_spiSettings);
            digitalWrite(_cs, LOW);
            setAddrWindow(0, 0, displayWidth, displayHeight);
            for (uint16_t y = 0; y < displayHeight; y++)
                _spi->transfer(line, nullptr, 2 * displayWidth);
            digitalWrite(_cs, HIGH);
            _spi->endTransaction();
            delay(400);
        }
#endif
        _forceRefresh = true;
    }

    void sendCommand(uint8_t command) override { sendRegister(command, {}); }
    int getBufferOffset() override { return 0; }

  private:
    uint8_t _rst;
    uint8_t _dc;
    uint8_t _cs;
    SPIClass *_spi;
    SPISettings _spiSettings;
    uint16_t _xOffset;
    uint16_t _yOffset;
    uint8_t _panelMadctl;
    uint8_t _madctl;
    uint16_t _rgb = 0xFFFF;
    uint16_t _buffheight = 0;
    uint16_t *_scanlineBuf = nullptr;
    uint16_t _scanlineBufCapacity = 0;
    uint16_t *_rowOnColors = nullptr;
    uint16_t *_rowOffColors = nullptr;
    uint16_t _rowColorCapacity = 0;
    TFTColorRegion *_colorRegions = nullptr;
    uint8_t _activeColorRegionCount = 0;
    uint32_t _lastColorRegionHash = 0;
    bool _hadColorRegionsLastFrame = false;
    bool _forceRefresh = true;

    void setOrientation(uint8_t madctl)
    {
        _madctl = madctl;
        sendRegister(0x36, {_madctl});
        _forceRefresh = true;
        delay(10);
    }

    void sendRegister(uint8_t command, std::initializer_list<uint8_t> data)
    {
        _spi->beginTransaction(_spiSettings);
        digitalWrite(_cs, LOW);
        writeCommand(command);
        for (uint8_t value : data)
            _spi->transfer(value);
        digitalWrite(_cs, HIGH);
        _spi->endTransaction();
    }

    void writeCommand(uint8_t command)
    {
        digitalWrite(_dc, LOW);
        _spi->transfer(command);
        digitalWrite(_dc, HIGH);
    }

    void write32(uint32_t value)
    {
        _spi->transfer(value >> 24);
        _spi->transfer(value >> 16);
        _spi->transfer(value >> 8);
        _spi->transfer(value);
    }

    void setAddrWindow(uint16_t x, uint16_t y, uint16_t width, uint16_t height)
    {
        x += _xOffset;
        y += _yOffset;
        writeCommand(0x2A);
        write32((static_cast<uint32_t>(x) << 16) | (x + width - 1));
        writeCommand(0x2B);
        write32((static_cast<uint32_t>(y) << 16) | (y + height - 1));
        writeCommand(0x2C);
    }

    uint16_t resolveTFTColorPixel(int16_t x, int16_t y, bool pixelSet, uint16_t fallbackOnColorBe, uint16_t fallbackOffColorBe,
                                  uint8_t regionCount)
    {
        if (_colorRegions != nullptr) {
            for (int16_t index = static_cast<int16_t>(regionCount) - 1; index >= 0; --index) {
                const TFTColorRegion &region = _colorRegions[index];
                if (x >= region.x && x < (region.x + region.width) && y >= region.y && y < (region.y + region.height))
                    return pixelSet ? region.onColorBe : region.offColorBe;
            }
        }
        return pixelSet ? fallbackOnColorBe : fallbackOffColorBe;
    }

    uint32_t hashColorRegions() const
    {
        uint32_t hash = 2166136261u;
        if (_colorRegions == nullptr || _activeColorRegionCount == 0)
            return hash;
        for (uint8_t i = 0; i < _activeColorRegionCount; i++) {
            const TFTColorRegion &region = _colorRegions[i];
            const uint16_t fields[] = {static_cast<uint16_t>(region.x),
                                       static_cast<uint16_t>(region.y),
                                       static_cast<uint16_t>(region.width),
                                       static_cast<uint16_t>(region.height),
                                       region.onColorBe,
                                       region.offColorBe};
            for (const uint16_t value : fields) {
                hash ^= static_cast<uint8_t>(value & 0xFF);
                hash *= 16777619u;
                hash ^= static_cast<uint8_t>((value >> 8) & 0xFF);
                hash *= 16777619u;
            }
        }
        return hash;
    }

    uint8_t countColorRegions() const
    {
        if (_colorRegions == nullptr)
            return 0;
        uint8_t regionCount = 0;
        while (regionCount < 48 && _colorRegions[regionCount].enabled)
            regionCount++;
        return regionCount;
    }

    void ensureRowColorBuffers(uint16_t width)
    {
        if (width <= _rowColorCapacity && _rowOnColors != nullptr && _rowOffColors != nullptr)
            return;
        free(_rowOnColors);
        free(_rowOffColors);
        _rowColorCapacity = 0;
        _rowOnColors = static_cast<uint16_t *>(malloc(width * sizeof(uint16_t)));
        _rowOffColors = static_cast<uint16_t *>(malloc(width * sizeof(uint16_t)));
        if (_rowOnColors != nullptr && _rowOffColors != nullptr) {
            _rowColorCapacity = width;
            return;
        }
        free(_rowOnColors);
        free(_rowOffColors);
        _rowOnColors = nullptr;
        _rowOffColors = nullptr;
    }

    void prepareRowColors(int16_t pixelY, int16_t minX, int16_t maxX, uint16_t fallbackOnColorBe, uint16_t fallbackOffColorBe,
                          uint8_t regionCount)
    {
        const uint16_t width = static_cast<uint16_t>(maxX - minX + 1);
        for (uint16_t i = 0; i < width; i++) {
            _rowOnColors[i] = fallbackOnColorBe;
            _rowOffColors[i] = fallbackOffColorBe;
        }
        // Later regions take precedence, as in the upstream per-pixel path.
        for (uint8_t i = 0; i < regionCount; i++) {
            const TFTColorRegion &region = _colorRegions[i];
            if (pixelY < region.y || pixelY >= (region.y + region.height))
                continue;
            const int16_t regionEnd = region.x + region.width - 1;
            const int16_t spanStart = max(minX, region.x);
            const int16_t spanEnd = min(maxX, regionEnd);
            if (spanStart > spanEnd)
                continue;
            for (uint16_t idx = spanStart - minX; idx <= static_cast<uint16_t>(spanEnd - minX); idx++) {
                _rowOnColors[idx] = region.onColorBe;
                _rowOffColors[idx] = region.offColorBe;
            }
        }
    }

    void ensureScanlineBuffer(uint16_t width)
    {
        if (width <= _scanlineBufCapacity && _scanlineBuf != nullptr)
            return;
        free(_scanlineBuf);
        _scanlineBufCapacity = 0;
        _scanlineBuf = static_cast<uint16_t *>(malloc(width * sizeof(uint16_t)));
        if (_scanlineBuf != nullptr)
            _scanlineBufCapacity = width;
    }
};
#endif
