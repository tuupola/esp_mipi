# ESP ST7735S Driver

Low level ESP ST7735S display driver. Supports M5Stick out of the box.

[![Software License](https://img.shields.io/badge/license-MIT-brightgreen.svg?style=flat-square)](LICENSE.md)

## Usage

Display size and orientation, SPI speed and all the pins can be changed with `menuconfig`. Defaults will work with an M5Stack.

```
$ make menuconfig
```

Note that this is a low level driver. It provides only putpixel, blit and ioctl functions. It is meant to be used as a building block for a graphics library such as [copepod](https://github.com/tuupola/copepod).

```c
#include <driver/spi_master.h>
#include <stdlib.h>

#include "st7735s.h"

spi_device_handle_t spi;
st7735s_init(&spi);

/* Draw a random pixel on the screen. */
uint16_t color = rand() % 0xffff;
int16_t x0 = rand() % DISPLAY_WIDTH;
int16_t y0 = rand() % DISPLAY_HEIGHT;

st7735s_putpixel(spi, x0, y0, color);

/* Draw a random rectangle on the screen. */
uint16_t w = rand() % 32;
uint16_t h = rand() % 32;
uint8_t bitmap[32 * 32 * DISPLAY_DEPTH];
wmemset(bitmap, color, 32 * 32 * DISPLAY_DEPTH);

st7735s_blit(spi, x0, y0, w, h, bitmap)

/* Turn display inversion on. */
st7735s_ioctl(spi, ST7735S_INVON, 0, 0);

/* Read display MADCTL setting. */
uint8_t buffer[2];
st7735s_ioctl(spi, ST7735S_RDDMADCTL, buffer, 2);
```


## License

The MIT License (MIT). Please see [License File](LICENSE.md) for more information.

Based on Espressif provided SPI Master example which was released to Public Domain: https://goo.gl/ksC2Ln
