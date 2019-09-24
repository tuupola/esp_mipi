/*

This code is based on Espressif provided SPI Master example which was
released to Public Domain: https://goo.gl/ksC2Ln

Copyright (c) 2017-2018 Espressif Systems (Shanghai) PTE LTD
Copyright (c) 2019 Mika Tuupola

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*/

#include <stdint.h>
#include <driver/spi_master.h>

#include "sdkconfig.h"

#define DISPLAY_WIDTH   (CONFIG_ST7735S_DISPLAY_WIDTH)
#define DISPLAY_HEIGHT  (CONFIG_ST7735S_DISPLAY_HEIGHT)
#define DISPLAY_DEPTH   (16)

#define SPI_MAX_TRANSFER_SIZE   (DISPLAY_WIDTH * DISPLAY_HEIGHT * DISPLAY_DEPTH)

typedef struct {
    uint8_t cmd;
    uint8_t data[16];
    uint8_t bytes;
} lcd_init_cmd_t;

void st7735s_blit(spi_device_handle_t spi, uint16_t x1, uint16_t y1, uint16_t w, uint16_t h, uint16_t *bitmap);
void st7735s_putpixel(spi_device_handle_t spi, uint16_t x1, uint16_t y1, uint16_t colour);
void st7735s_init(spi_device_handle_t *spi);
void st7735s_ioctl(spi_device_handle_t spi, uint8_t command, uint8_t *data, size_t size);

#define ST7735S_NOP         0x00
#define ST7735S_SWRESET     0x01 /* Software Reset */
#define ST7735S_RDDID       0x04 /* Read Display ID */
#define ST7735S_RDDST       0x09 /* Read Display Status */
#define ST7735S_RDDPM       0x0a /* Read Display Power Mode */
#define ST7735S_RDDMADCTL   0x0b /* Read Display MADCTL */
#define ST7735S_RDDCOLMOD   0x0c /* Read Display Pixel Format */
#define ST7735S_RDDIM       0x0d /* Read Display Image Mode */
#define ST7735S_RDDSM       0x0e /* Read Display Signal Mode */
#define ST7735S_RDDSDR      0x0f /* Read Display Self-Diagnostic Result */
#define ST7735S_SLPIN       0x10 /* Sleep In */
#define ST7735S_SLPOUT      0x11 /* Sleep Out */
#define ST7735S_PTLON       0x12 /* Partial Display Mode On */
#define ST7735S_NORON       0x13 /* Normal Display Mode On */
#define ST7735S_INVOFF      0x20 /* Display Inversion Off */
#define ST7735S_INVON       0x21 /* Display Inversion On */
#define ST7735S_GAMSET      0x26 /* Gamma Set */
#define ST7735S_DISPOFF     0x28 /* Display Off */
#define ST7735S_DISPON      0x29 /* Display On */
#define ST7735S_CASET       0x2a /* Column Address Set */
#define ST7735S_RASET       0x2b /* Row Address Set */
#define ST7735S_RAMWR       0x2c /* Memory Write */
#define ST7735S_RGBSET      0x2d /* Color Setting 4k, 65k, 262k */
#define ST7735S_RAMRD       0x2e /* Memory Read */
#define ST7735S_PTLAR       0x30 /* Partial Area */
#define ST7735S_SCRLAR      0x33 /* Scroll Area Set */
#define ST7735S_TEOFF       0x34 /* Tearing Effect Line OFF */
#define ST7735S_TEON        0x35 /* Tearing Effect Line ON */
#define ST7735S_MADCTL      0x36 /* Memory Data Access Control */
#define ST7735S_VSCSAD      0x37 /* Vertical Scroll Start Address of RAM */
#define ST7735S_IDMOFF      0x38 /* Idle Mode Off */
#define ST7735S_IDMON       0x39 /* Idle Mode On */
#define ST7735S_COLMOD      0x3a /* Interface Pixel Format */
#define ST7735S_RDID1       0xda /* Read ID1 Value */
#define ST7735S_RDID2       0xdb /* Read ID2 Value */
#define ST7735S_RDID3       0xdc /* Read ID3 Value */
#define ST7735S_FRMCTR1     0xb1 /* Frame Rate Control in normal mode, full colors */
#define ST7735S_FRMCTR2     0xb2 /* Frame Rate Control in idle mode, 8 colors */
#define ST7735S_FRMCTR3     0xb3 /* Frame Rate Control in partial mode, full colors */
#define ST7735S_INVCTR      0xb4 /* Display Inversion Control */
#define ST7735S_PWCTR1      0xc0 /* Power Control 1 */
#define ST7735S_PWCTR2      0xc1 /* Power Control 2 */
#define ST7735S_PWCTR3      0xc2 /* Power Control 3 in normal mode, full colors */
#define ST7735S_PWCTR4      0xc3 /* Power Control 4 in idle mode 8colors */
#define ST7735S_PWCTR5      0xc4 /* Power Control 5 in partial mode, full colors */
#define ST7735S_VMCTR1      0xc5 /* VCOM Control 1 */
#define ST7735S_VMOFCTR     0xc7 /* VCOM Offset Control */
#define ST7735S_WRID2       0xd1 /* Write ID2 Value */
#define ST7735S_WRID3       0xd2 /* Write ID3 Value */
#define ST7735S_NVFCTR1     0xd9 /* NVM Control Status */
#define ST7735S_NVFCTR2     0xde /* NVM Read Command */
#define ST7735S_NVFCTR3     0xdf /* NVM Write Command */
#define ST7735S_GMCTRP1     0xe0 /* Gamma '+'Polarity Correctio Characteristics Setting */
#define ST7735S_GMCTRN1     0xe1 /* Gamma '-'Polarity Correctio Characteristics Setting */
#define ST7735S_GCV         0xfc /* Gate Pump Clock Frequency Variable */
