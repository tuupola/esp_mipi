# MIPI DCS Display Driver

Low level driver for displays supporting the [MIPI Display Commant Set](https://www.mipi.org/specifications/display-command-set). Currently tested with ST7735S and ILI9341.

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

#include "mipi-display.h"

spi_device_handle_t spi;
mipi_display_init(&spi);

/* Draw a random pixel on the screen. */
uint16_t color = rand() % 0xffff;
int16_t x0 = rand() % DISPLAY_WIDTH;
int16_t y0 = rand() % DISPLAY_HEIGHT;

mipi_display_put_pixel(spi, x0, y0, color);

/* Draw a random rectangle on the screen. */
uint16_t w = rand() % 32;
uint16_t h = rand() % 32;
uint8_t bitmap[32 * 32 * DISPLAY_DEPTH];
wmemset(bitmap, color, 32 * 32 * DISPLAY_DEPTH);

mipi_display_blit(spi, x0, y0, w, h, bitmap)
```

You can also issue any command defined in [mipi-dcs.h](mipi-dcs.h).

```c
#include <driver/spi_master.h>
#include <stdlib.h>

#include "mipi-dcs.h"
#include "mipi-display.h"

spi_device_handle_t spi;
mipi_display_init(&spi);

/* Turn display inversion on. */
mipi_display_ioctl(spi, MIPI_DCS_ENTER_INVERT_MODE, 0, 0);

/* Read display ADDRESS_MODE setting. */
uint8_t buffer[2];
mipi_display_ioctl(spi, MIPI_DCS_GET_ADDRESS_MODE, buffer, 2);
```

## License

The MIT License (MIT). Please see [License File](LICENSE.md) for more information.

Based on Espressif provided SPI Master example which was released to Public Domain: https://goo.gl/ksC2Ln
