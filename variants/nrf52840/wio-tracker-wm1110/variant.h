/*
  Copyright (c) 2014-2015 Arduino LLC.  All right reserved.
  Copyright (c) 2016 Sandeep Mistry All right reserved.
  Copyright (c) 2018, Adafruit Industries (adafruit.com)

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.
  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef _VARIANT_WIO_TRACKER_WM1110_
#define _VARIANT_WIO_TRACKER_WM1110_

/** Master clock frequency */
#define VARIANT_MCK (64000000ul)

#define USE_LFXO // Board uses 32khz crystal for LF
// define USE_LFRC    // Board uses RC for LF

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "WVariant.h"

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

// Number of pins defined in PinDescription array
#define PINS_COUNT (48)
#define NUM_DIGITAL_PINS (48)
#define NUM_ANALOG_INPUTS (6)
#define NUM_ANALOG_OUTPUTS (0)

#define WIRE_INTERFACES_COUNT 1

// We rely on the nrf52840 USB controller to tell us if we are hooked to a power supply
#define NRF_APM

#define PIN_3V3_EN (32 + 1) // P1.01, Power to Sensors

#define PIN_WIRE_SDA (0 + 5) // P0.05
#define PIN_WIRE_SCL (0 + 4) // P0.04

#define PIN_LED1 (0 + 6) // P0.06
#define LED_GREEN PIN_LED1

#define LED_STATE_ON 0

#define BUTTON_PIN (32 + 2) // P1.02

/*
 * Serial interfaces
 */
#define PIN_SERIAL1_RX (0 + 24) // P0.24
#define PIN_SERIAL1_TX (0 + 25) // P0.25

#define PIN_SERIAL2_RX (0 + 6) // P0.06
#define PIN_SERIAL2_TX (0 + 8) // P0.08

#ifdef WIO_ST7789_LCD_BENCH
#define SPI_INTERFACES_COUNT 2
#define PIN_SPI1_MISO (0 + 22)
#define PIN_SPI1_MOSI (0 + 23)
#define PIN_SPI1_SCK (0 + 21)
#define PIN_SPI1_NSS (0 + 20)

#define USE_ST7789
#define HAS_SPI_TFT 1
#define TFT_WIDTH 240
#define TFT_HEIGHT 240
#define ST7789_RESET (0 + 17)
#define ST7789_RS (0 + 19) // J6.12 WRX is D/C in four-wire SPI.
#define ST7789_NSS PIN_SPI1_NSS
#define ST7789_TE (0 + 14)
#define WIO_LCD_SPI_HZ 4000000
#define WIO_LCD_X_OFFSET 0
#define WIO_LCD_Y_OFFSET 0
#define WIO_LCD_MADCTL 0xC8
// Custom board: Q5 (BSS84) high-side switch, gate = pwr_lcd_n on P0.15, active low
// (100k pull-up R4 keeps it off at boot). Dev bench: P0.15 is the disconnected
// rotary-press pin; driving it low there is inert.
#ifdef WIO_LCD_PWR_GATE
#if !defined(WIO_ST7789_LCD_BENCH) || !defined(USE_ST7789)
#error WIO_LCD_PWR_GATE needs the ST7789 bench pin definitions
#endif
// Full gating (R20 fitted, R21 removed, R19 fitted, R17 NC): Q5 switches both the
// backlight and the LCD controller supply; VTFT_CTRL paths manage the rail.
#define VTFT_CTRL (0 + 15)
#else
// Backlight-only assembly (R19 fitted; R21 keeps controller VDD always on).
// Do not define VTFT_LEDA; there is no separate backlight GPIO in either build.
#define WIO_LCD_BL_PIN (0 + 15)
#endif
#else
#define SPI_INTERFACES_COUNT 1
#endif

#define PIN_SPI_MISO (32 + 15) // P1.15 47
#define PIN_SPI_MOSI (32 + 14) // P1.14 46
#define PIN_SPI_SCK (32 + 13)  // P1.13 45
#define PIN_SPI_NSS (32 + 12)  // P1.12 44

#define LORA_RESET (32 + 10) // P1.10 10 // RST
#define LORA_DIO1 (0 + 2)    // P0.02 2 // IRQ
#define LORA_DIO2 (32 + 11)  // P1.11 43 // BUSY
#define LORA_SCK PIN_SPI_SCK
#define LORA_MISO PIN_SPI_MISO
#define LORA_MOSI PIN_SPI_MOSI
#define LORA_CS PIN_SPI_NSS

// supported modules list
#define USE_LR1110

#define LR1110_IRQ_PIN LORA_DIO1
#define LR1110_NRESET_PIN LORA_RESET
#define LR1110_BUSY_PIN LORA_DIO2
#define LR1110_SPI_NSS_PIN LORA_CS
#define LR1110_SPI_SCK_PIN LORA_SCK
#define LR1110_SPI_MOSI_PIN LORA_MOSI
#define LR1110_SPI_MISO_PIN LORA_MISO

#define LR11X0_DIO3_TCXO_VOLTAGE 1.6
#define LR11X0_DIO_AS_RF_SWITCH

#define LR1110_GNSS_ANT_PIN (32 + 5) // P1.05 37

#define GPS_RX_PIN PIN_SERIAL1_RX
#define GPS_TX_PIN PIN_SERIAL1_TX

#define HAS_GPS 1

#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#endif // _VARIANT_WIO_TRACKER_WM1110_
