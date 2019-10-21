# MIPI DCS Display Driver

Low level driver for displays supporting the [MIPI Display Command Set](https://www.mipi.org/specifications/display-command-set). Currently tested with ST7735S, ST7789V and ILI9341.

[![Software License](https://img.shields.io/badge/license-MIT-brightgreen.svg?style=flat-square)](LICENSE.md)

## Usage

Display and SPI parameters all the pins can be changed with `menuconfig`. You can find predefined values for some popular board in the [sdkconfig](https://github.com/tuupola/esp_mipi/tree/master/sdkconfig) folder. To use them copy one of the files to your project root before running menuconfig.

```
$ cp components/esp_mipi/sdkconfig/m5stack.defaults sdkconfig.defaults
$ make menuconfig
```

If you have already run `menuconfig` before, do a `defconfig` first.

```
$ cp components/esp_mipi/sdkconfig/m5stack.defaults sdkconfig.defaults
$ make defconfig
$ make menuconfig
```

Note that this is a low level driver. It provides only `init`, `write`, `ioctl` and `close` functions. It is meant to be used as a building block for a graphics library such as [copepod](https://github.com/tuupola/copepod). For basic usage see also [MIPI driver speedtest](https://github.com/tuupola/esp-examples/tree/master/016-mipi-speedtest).

```c
#include <driver/spi_master.h>
#include <stdlib.h>

#include "mipi_display.h"

spi_device_handle_t spi;

void *memset16(uint16_t *source, uint16_t value, size_t count)
{
    uint16_t *ptr = source;

    while (count--) {
        *ptr++ = value;
    }
    return source;
}

void app_main()
{
    uint16_t color;
    int16_t x0;
    int16_t y0;

    mipi_display_init(&spi);

    /* Initialise back buffer */
    uint8_t *buffer = (uint8_t *) heap_caps_malloc(
        (DISPLAY_WIDTH * (DISPLAY_DEPTH / 8) * DISPLAY_HEIGHT),
        MALLOC_CAP_DMA | MALLOC_CAP_32BIT
    );

    /* Clear screen with black color */
    memset16((uint16_t *) buffer, 0x0000, DISPLAY_WIDTH * DISPLAY_HEIGHT);
    mipi_display_write(spi, 0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT, buffer);

    /* Draw a 100 random pixels on the screen. */
    for (uint8_t i = 0; i < 100; i++) {
        x0 = rand() % DISPLAY_WIDTH;
        y0 = rand() % DISPLAY_HEIGHT;
        color = rand() % 0xffff;
        mipi_display_write(spi, x0, y0, 1, 1, (uint8_t *) &color);
    }

    /* Draw 10 random 32x32 rectangles on the screen. */
    for (uint8_t i = 0; i < 10; i++) {
        x0 = rand() % (DISPLAY_WIDTH - 32);
        y0 = rand() % (DISPLAY_HEIGHT - 32);
        color = rand() % 0xffff;
        memset16((uint16_t *) buffer, color, 32 * 32);
        mipi_display_write(spi, x0, y0, 32, 32, buffer);
    }

    /* Clean shutdown */
    mipi_display_close(spi);

}
```

You can also issue any command defined in [mipi_dcs.h](mipi_dcs.h).

```c
#include <driver/spi_master.h>
#include <stdlib.h>

#include "mipi_dcs.h"
#include "mipi_display.h"

spi_device_handle_t spi;

void app_main()
{
    mipi_display_init(&spi);

    /* Turn display inversion on. */
    mipi_display_ioctl(spi, MIPI_DCS_ENTER_INVERT_MODE, 0, 0);

    /* Read display address mode setting. */
    uint8_t buffer[2];
    mipi_display_ioctl(spi, MIPI_DCS_GET_ADDRESS_MODE, buffer, 2);

    /* Clean shutdown */
    mipi_display_close(spi);
}
```

```

## License

The MIT License (MIT). Please see [License File](LICENSE.md) for more information.

Based on Espressif provided SPI Master example which was released to Public Domain: https://goo.gl/ksC2Ln
