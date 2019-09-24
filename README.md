# ESP ST7735S Driver

Low level ESP ST7735S display driver. Supports M5Stick out of the box.

[![Software License](https://img.shields.io/badge/license-MIT-brightgreen.svg?style=flat-square)](LICENSE.md)

## Usage

First initialise the ST7735S driver. SPI speed and pins can be changed with `$ make menuconfig`. Default settings will work with M5Stick. This drives is low lever. It provides only blit, putpixel and ioctl functions. It is meant to be used with an external graphics library such as [copepod](https://github.com/tuupola/copepod). For example usage see [M5Stick graphics example](https://github.com/tuupola/esp-examples/tree/master/015-m5stick-gfx).

```c
#include <driver/spi_master.h>

#include "st7735s.h"

spi_device_handle_t spi;
st7735s_init(&spi);

st7735s_blit(spi, x0, y0, w, h, &bitmap);
st7735s_putpixel(spi, x0, y0, color);
st7735s_ioctl(spi, command, &data, size);
```

## License

The MIT License (MIT). Please see [License File](LICENSE.md) for more information.

Based on Espressif provided SPI Master example which was released to Public Domain: https://goo.gl/ksC2Ln
