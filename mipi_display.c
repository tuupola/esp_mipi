/*

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

/*

This code is based on Espressif provided SPI Master example which was
released to Public Domain: https://goo.gl/ksC2Ln

*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <driver/spi_master.h>
#include <soc/gpio_struct.h>
#include <driver/gpio.h>
#include <esp_log.h>

#include "sdkconfig.h"
#include "mipi_dcs.h"
#include "mipi_display.h"

static const char *TAG = "mipi_display";
static const uint8_t DELAY_BIT = 1 << 7;

static SemaphoreHandle_t mutex;

DRAM_ATTR static const lcd_init_cmd_t st_init_commands[] = {
    {MIPI_DCS_SOFT_RESET, {0}, 0 | DELAY_BIT},
    {MIPI_DCS_SET_ADDRESS_MODE, {MIPI_DISPLAY_ADDRESS_MODE}, 1},
    {MIPI_DCS_SET_PIXEL_FORMAT, {CONFIG_MIPI_DISPLAY_PIXEL_FORMAT}, 1},
#ifdef CONFIG_MIPI_DISPLAY_INVERT
    {MIPI_DCS_ENTER_INVERT_MODE, {0}, 0},
#else
    {MIPI_DCS_EXIT_INVERT_MODE, {0}, 0},
#endif
    {MIPI_DCS_EXIT_SLEEP_MODE, {0}, 0 | DELAY_BIT},
    {MIPI_DCS_SET_DISPLAY_ON, {0}, 0 | DELAY_BIT},
    /* End of commands . */
    {0, {0}, 0xff},
};

/* Uses spi_device_transmit, which waits until the transfer is complete. */
static void mipi_display_command(spi_device_handle_t spi, const uint8_t command)
{
    spi_transaction_t transaction;
    memset(&transaction, 0, sizeof(transaction));

    /* Command is 1 byte ie 8 bits */
    transaction.length = 1 * 8;
    /* The data is the cmd itself */
    transaction.tx_buffer = &command;
    /* DC needs to be set to 0. */
    transaction.user = (void *) 0;
    ESP_LOGD(TAG, "Sending command 0x%02x", (uint8_t)command);
    ESP_ERROR_CHECK(spi_device_polling_transmit(spi, &transaction));
}

/* Uses spi_device_transmit, which waits until the transfer is complete. */
static void mipi_display_write_data(spi_device_handle_t spi, const uint8_t *data, size_t length)
{
    if (0 == length) {
        return;
    };

    spi_transaction_t transaction;
    memset(&transaction, 0, sizeof(transaction));

     /* Length in bits */
    transaction.length = length * 8;
    transaction.tx_buffer = data;
     /* DC needs to be set to 1 */
    transaction.user = (void *) 1;

    ESP_LOG_BUFFER_HEX_LEVEL(TAG, data, length, ESP_LOG_DEBUG);
    ESP_ERROR_CHECK(spi_device_polling_transmit(spi, &transaction));
}

static void mipi_display_read_data(spi_device_handle_t spi, uint8_t *data, size_t length)
{
    if (0 == length) {
        return;
    };

    spi_transaction_t transaction;
    memset(&transaction, 0, sizeof(transaction));

     /* Length in bits */
    transaction.length = length * 8;
    transaction.rxlength = length * 8;
    transaction.rx_buffer = data;
    //transaction.flags = SPI_TRANS_USE_RXDATA;
     /* DC needs to be set to 1 */
    transaction.user = (void *) 1;


    ESP_LOG_BUFFER_HEX_LEVEL(TAG, data, length, ESP_LOG_WARN);
    ESP_ERROR_CHECK(spi_device_polling_transmit(spi, &transaction));
    ESP_LOG_BUFFER_HEX_LEVEL(TAG, data, length, ESP_LOG_INFO);
}


/* This function is called (in irq context!) just before a transmission starts. */
/* It will set the DC line to the value indicated in the user field. */
static void mipi_display_pre_callback(spi_transaction_t *transaction)
{
    uint32_t dc = (uint32_t) transaction->user;
    gpio_set_level(CONFIG_MIPI_DISPLAY_PIN_DC, dc);
}

static void mipi_display_spi_master_init(spi_device_handle_t *spi)
{
    spi_bus_config_t buscfg = {
        .miso_io_num = CONFIG_MIPI_DISPLAY_PIN_MISO,
        .mosi_io_num = CONFIG_MIPI_DISPLAY_PIN_MOSI,
        .sclk_io_num = CONFIG_MIPI_DISPLAY_PIN_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        /* Max transfer size in bytes */
        .max_transfer_sz = SPI_MAX_TRANSFER_SIZE
    };
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = CONFIG_SPI_CLOCK_SPEED_HZ,
        .mode = 0,
        .spics_io_num = CONFIG_MIPI_DISPLAY_PIN_CS,
        .queue_size = 64,
        /* Handles the D/C line */
        .pre_cb = mipi_display_pre_callback,
        .flags = SPI_DEVICE_NO_DUMMY
    };
    ESP_ERROR_CHECK(spi_bus_initialize(HSPI_HOST, &buscfg, 1));
    ESP_ERROR_CHECK(spi_bus_add_device(HSPI_HOST, &devcfg, spi));
}

void mipi_display_init(spi_device_handle_t *spi)
{
    uint8_t cmd = 0;

    mutex = xSemaphoreCreateMutex();

	gpio_set_direction(CONFIG_MIPI_DISPLAY_PIN_CS, GPIO_MODE_OUTPUT);
	gpio_set_level(CONFIG_MIPI_DISPLAY_PIN_CS, 0);

    /* Init non spi gpio. */
    gpio_set_direction(CONFIG_MIPI_DISPLAY_PIN_DC, GPIO_MODE_OUTPUT);
    gpio_set_direction(CONFIG_MIPI_DISPLAY_PIN_RST, GPIO_MODE_OUTPUT);

    /* Init spi driver. */
    mipi_display_spi_master_init(spi);
    vTaskDelay(100 / portTICK_RATE_MS);

    /* Reset the display. */
    gpio_set_level(CONFIG_MIPI_DISPLAY_PIN_RST, 0);
    vTaskDelay(100 / portTICK_RATE_MS);
    gpio_set_level(CONFIG_MIPI_DISPLAY_PIN_RST, 1);
    vTaskDelay(100 / portTICK_RATE_MS);


    /* Send all the commands. */
    while (st_init_commands[cmd].bytes != 0xff) {
        mipi_display_command(*spi, st_init_commands[cmd].cmd);
        mipi_display_write_data(*spi, st_init_commands[cmd].data, st_init_commands[cmd].bytes & 0x1F);
        if (st_init_commands[cmd].bytes & DELAY_BIT) {
            ESP_LOGD(TAG, "Delaying after command 0x%02x", (uint8_t)st_init_commands[cmd].cmd);
            vTaskDelay(200 / portTICK_RATE_MS);
        }
        cmd++;
    }

    ESP_LOGI(TAG, "Display initialized.");

    /* Enable backlight */
    if (CONFIG_MIPI_DISPLAY_PIN_BL > 0) {
        gpio_set_direction(CONFIG_MIPI_DISPLAY_PIN_BL, GPIO_MODE_OUTPUT);
        gpio_set_level(CONFIG_MIPI_DISPLAY_PIN_BL, 1);
    }

    spi_device_acquire_bus(*spi, portMAX_DELAY);

}

void mipi_display_blit(spi_device_handle_t spi, uint16_t x1, uint16_t y1, uint16_t w, uint16_t h, uint16_t *bitmap)
{
    if (0 == w || 0 == h) {
        return;
    }

    x1 = x1 + CONFIG_MIPI_DISPLAY_OFFSET_X;
    y1 = y1 + CONFIG_MIPI_DISPLAY_OFFSET_Y;

    int32_t x2 = x1 + w - 1;
    int32_t y2 = y1 + h - 1;
    uint32_t size = w * h;

    spi_transaction_t command;
    spi_transaction_t data;

    xSemaphoreTake(mutex, portMAX_DELAY);

    memset(&command, 0, sizeof(spi_transaction_t));
    command.length = 8;
    command.user = (void *) 0;
    command.flags = SPI_TRANS_USE_TXDATA;

    memset(&data, 0, sizeof(spi_transaction_t));
    data.length = 8 * 4;
    data.user = (void *) 1;
    data.flags = SPI_TRANS_USE_TXDATA;

    command.tx_data[0] = MIPI_DCS_SET_COLUMN_ADDRESS;
    ESP_ERROR_CHECK(spi_device_polling_transmit(spi, &command));

    data.tx_data[0] = x1 >> 8;
    data.tx_data[1] = x1 & 0xff;
    data.tx_data[2] = x2 >> 8;
    data.tx_data[3] = x2 & 0xff;
    ESP_ERROR_CHECK(spi_device_polling_transmit(spi, &data));

    command.tx_data[0] = MIPI_DCS_SET_PAGE_ADDRESS;
    ESP_ERROR_CHECK(spi_device_polling_transmit(spi, &command));

    data.tx_data[0] = y1 >> 8;
    data.tx_data[1] = y1 & 0xff;
    data.tx_data[2] = y2 >> 8;
    data.tx_data[3] = y2 & 0xff;
    ESP_ERROR_CHECK(spi_device_polling_transmit(spi, &data));

    command.tx_data[0] = MIPI_DCS_WRITE_MEMORY_START;
    ESP_ERROR_CHECK(spi_device_polling_transmit(spi, &command));

    data.rxlength = 0;
    data.tx_buffer = bitmap;
    /* Transfer size in bits */
    data.length = size * DISPLAY_DEPTH;
    /* Clear SPI_TRANS_USE_TXDATA flag */
    data.flags = 0;

    if (data.length > SPI_MAX_TRANSFER_SIZE / 2) {
        ESP_ERROR_CHECK(spi_device_transmit(spi, &data));
    } else {
        ESP_ERROR_CHECK(spi_device_polling_transmit(spi, &data));
    }

    xSemaphoreGive(mutex);
}

void mipi_display_put_pixel(spi_device_handle_t spi, uint16_t x1, uint16_t y1, uint16_t colour)
{
    mipi_display_blit(spi, x1, y1, 1, 1, &colour);
}

void mipi_display_ioctl(spi_device_handle_t spi, const uint8_t command, uint8_t *data, size_t size)
{
    xSemaphoreTake(mutex, portMAX_DELAY);

    switch (command) {
        case MIPI_DCS_GET_COMPRESSION_MODE:
        case MIPI_DCS_GET_DISPLAY_ID:
        case MIPI_DCS_GET_RED_CHANNEL:
        case MIPI_DCS_GET_GREEN_CHANNEL:
        case MIPI_DCS_GET_BLUE_CHANNEL:
        case MIPI_DCS_GET_DISPLAY_STATUS:
        case MIPI_DCS_GET_POWER_MODE:
        case MIPI_DCS_GET_ADDRESS_MODE:
        case MIPI_DCS_GET_PIXEL_FORMAT:
        case MIPI_DCS_GET_DISPLAY_MODE:
        case MIPI_DCS_GET_SIGNAL_MODE:
        case MIPI_DCS_GET_DIAGNOSTIC_RESULT:
        case MIPI_DCS_GET_SCANLINE:
        case MIPI_DCS_GET_DISPLAY_BRIGHTNESS:
        case MIPI_DCS_GET_CONTROL_DISPLAY:
        case MIPI_DCS_GET_POWER_SAVE:
        case MIPI_DCS_READ_DDB_START:
        case MIPI_DCS_READ_DDB_CONTINUE:
            mipi_display_command(spi, command);
            mipi_display_read_data(spi, data, size);
            break;
        default:
            mipi_display_command(spi, command);
            mipi_display_write_data(spi, data, size);
    }

    xSemaphoreGive(mutex);
}

void mipi_display_close(spi_device_handle_t spi)
{
    spi_device_release_bus(spi);
}