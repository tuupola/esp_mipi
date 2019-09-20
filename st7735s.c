/*

This code is based on Espressif provided SPI Master example which was
released to Public Domain: https://goo.gl/ksC2Ln


Copyright (c) 2017-2018 Espressif Systems (Shanghai) PTE LTD
Copyright (c) 2018 Mika Tuupola

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
#include "ili9341.h"

static const char *TAG = "ili9341";
static const uint8_t DELAY_BIT = 1 << 7;

static SemaphoreHandle_t mutex;

DRAM_ATTR static const lcd_init_cmd_t st_init_commands[]={
    {0x01, {0}, 0 | DELAY_BIT},
    {0x11, {0}, 0 | DELAY_BIT},

    {0xB1, {0x01, 0x2C, 0x2D}, 3}, // Frame Rate Control
    {0xB2, {0x01, 0x2C, 0x2D}, 3}, // Frame Rate Control
    {0xB3, {0x01, 0x2C, 0x2D, 0x01, 0x2C, 0x2D}, 6}, // Frame Rate Control
    {0xB4, {0x07}, 1}, // Display Inversion Control
    {0xC0, {0xA2, 0x02, 0x84}, 3}, // Power Control 1
    {0xC1, {0xC5}, 1}, // Power Control 2
    {0xC2, {0x0A, 0x00}, 2}, // Power Control 3
    {0xC3, {0x8A, 0x2A}, 2}, // Power Control 4
    {0xC4, {0x8A, 0xEE}, 2}, // Power Control 5
    {0xC5, {0x0E}, 1}, // VCOM Control 1
    {0x20, {0}, 0}, // Display Inversion Off
    {0x36, {0xC0}, 1}, // Memory Data Access Control 0xC8
    {0x3A, {0x05}, 1}, // Interface Pixel Format
    {0x2A, {0x00, 0x02, 0x00, 0x81}, 4}, // Column Address Set
    {0x2B, {0x00, 0x01, 0x00, 0xA0}, 4}, // Row Address Set
    {0x21, {0}, 0}, // Display Inversion On
    {0xE0, {0x02, 0x1C, 0x07, 0x12, 0x37, 0x32, 0x29, 0x2D, 0x29, 0x25, 0x2B, 0x39, 0x00, 0x01, 0x03, 0x10}, 16}, // Gamma (‘+’polarity) Correction Characteristics Setting
    {0xE1, {0x03, 0x1D, 0x07, 0x06, 0x2E, 0x2C, 0x29, 0x2D, 0x2E, 0x2E, 0x37, 0x3F, 0x00, 0x00, 0x02, 0x10}, 16}, // Gamma ‘-’polarity Correction Characteristics Setting
    {0x13, {0}, 0 | DELAY_BIT}, // Normal Display Mode On
    {0x29, {0}, 0 | DELAY_BIT}, // Display On
    /* End of commands . */
    {0, {0}, 0xff},
};

/* Uses spi_device_transmit, which waits until the transfer is complete. */
static void ili9341_command(spi_device_handle_t spi, const uint8_t command)
{
    spi_transaction_t transaction;

    memset(&transaction, 0, sizeof(transaction));
    transaction.length = 1 * 8; /* Command is 1 byte ie 8 bits. */
    transaction.tx_buffer = &command; /* The data is the cmd itself. */
    transaction.user = (void*)0; /* DC needs to be set to 0. */
    ESP_ERROR_CHECK(spi_device_transmit(spi, &transaction));
}

/* Uses spi_device_transmit, which waits until the transfer is complete. */
static void ili9341_data(spi_device_handle_t spi, const uint8_t *data, uint16_t length)
{
    spi_transaction_t transaction;

    if (0 == length) { return; };
    memset(&transaction, 0, sizeof(transaction));
    transaction.length = length * 8; /* Length in bits. */
    transaction.tx_buffer = data;
    transaction.user = (void*)1; /* DC needs to be set to 1. */
    ESP_ERROR_CHECK(spi_device_transmit(spi, &transaction));
}


/* This function is called (in irq context!) just before a transmission starts. */
/* It will set the DC line to the value indicated in the user field. */
static void ili9341_pre_callback(spi_transaction_t *transaction)
{
    int dc=(int)transaction->user;
    gpio_set_level(CONFIG_ILI9341_PIN_DC, dc);
}

static void ili9341_wait(spi_device_handle_t spi)
{
    spi_transaction_t *rtrans;

    /* TODO: This should be all transactions. */
    for (uint8_t i = 0; i <= 5; i++) {
        ESP_ERROR_CHECK(spi_device_get_trans_result(spi, &rtrans, portMAX_DELAY));
        /* Do something with the result. */
    }
}

static void ili9431_spi_master_init(spi_device_handle_t *spi)
{
    spi_bus_config_t buscfg = {
        .miso_io_num = CONFIG_ILI9341_PIN_MISO,
        .mosi_io_num = CONFIG_ILI9341_PIN_MOSI,
        .sclk_io_num = CONFIG_ILI9341_PIN_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        /* Max transfer size in bytes */
        .max_transfer_sz = SPI_MAX_TRANSFER_SIZE
    };
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = CONFIG_SPI_CLOCK_SPEED_HZ,
        .mode = 0,
        .spics_io_num = CONFIG_ILI9341_PIN_CS,
        .queue_size = 64,
        /* Handles the D/C line */
        .pre_cb = ili9341_pre_callback,
        .flags = SPI_DEVICE_NO_DUMMY
    };
    ESP_ERROR_CHECK(spi_bus_initialize(HSPI_HOST, &buscfg, 1));
    ESP_ERROR_CHECK(spi_bus_add_device(HSPI_HOST, &devcfg, spi));
}

void ili9341_init(spi_device_handle_t *spi)
{
    uint8_t cmd = 0;

    mutex = xSemaphoreCreateMutex();

    /* Init spi driver. */
    ili9431_spi_master_init(spi);

    /* Init non spi gpio. */
    gpio_set_direction(CONFIG_ILI9341_PIN_DC, GPIO_MODE_OUTPUT);
    gpio_set_direction(CONFIG_ILI9341_PIN_RST, GPIO_MODE_OUTPUT);
    gpio_set_direction(CONFIG_ILI9341_PIN_BCKL, GPIO_MODE_OUTPUT);

    /* Reset the display. */
    gpio_set_level(CONFIG_ILI9341_PIN_RST, 0);
    vTaskDelay(100 / portTICK_RATE_MS);
    gpio_set_level(CONFIG_ILI9341_PIN_RST, 1);
    vTaskDelay(100 / portTICK_RATE_MS);

    ESP_LOGI(TAG, "Initialize the display.");

    /* Send all the commands. */
    whilest[cmd].bytes != 0xff) {
        ili9341_command(*spist[cmd].cmd);
        ili9341_data(*spist[cmd].datast[cmd].bytes & 0x1F);
        ifst[cmd].bytes & DELAY_BIT) {
            vTaskDelay(100 / portTICK_RATE_MS);
        }
        cmd++;
    }

    /* Enable backlight. */
    gpio_set_level(CONFIG_ILI9341_PIN_BCKL, 1);
}

void ili9431_blit(spi_device_handle_t spi, uint16_t x1, uint16_t y1, uint16_t w, uint16_t h, uint16_t *bitmap)
{
    if (0 == w || 0 == h) {
        return;
    }

    int x;

    int32_t x2 = x1 + w - 1;
    int32_t y2 = y1 + h - 1;

    static spi_transaction_t trans[6];
    uint32_t size = w * h;

    xSemaphoreTake(mutex, portMAX_DELAY);

    /* In theory, it's better to initialize trans and data only once and hang */
    /* on to the initialized variables. We allocate them on the stack, so we need */
    /* to re-init them each call. */
    for (x = 0; x < 6; x++) {
        memset(&trans[x], 0, sizeof(spi_transaction_t));
        if (0 == (x&1)) {
            /* Even transfers are commands. */
            trans[x].length = 8;
            trans[x].user = (void*)0;
        } else {
            /* Odd transfers are data. */
            trans[x].length = 8 * 4;
            trans[x].user = (void*)1;
        }
        trans[x].flags = SPI_TRANS_USE_TXDATA;
    }

    /* Column Address Set */
    trans[0].tx_data[0] = ILI9341_CASET;
    trans[1].tx_data[0] = x1 >> 8;
    trans[1].tx_data[1] = x1 & 0xff;
    trans[1].tx_data[2] = x2 >> 8;
    trans[1].tx_data[3] = x2 & 0xff;
    /* Page Address Set */
    trans[2].tx_data[0] = ILI9341_PASET;
    trans[3].tx_data[0] = y1 >> 8;
    trans[3].tx_data[1] = y1 & 0xff;
    trans[3].tx_data[2] = y2 >> 8;
    trans[3].tx_data[3] = y2 & 0xff;
    /* Memory Write */
    trans[4].tx_data[0] = ILI9341_RAMWR;
    trans[5].tx_buffer = bitmap;
    /* Transfer size in bits */
    trans[5].length = size * DISPLAY_DEPTH;
    /* Clear SPI_TRANS_USE_TXDATA flag */
    trans[5].flags = 0;

    for (x = 0; x <= 5; x++) {
        ESP_ERROR_CHECK(spi_device_queue_trans(spi, &trans[x], portMAX_DELAY));
    }
    /* Could do stuff here... */
    ili9341_wait(spi);

    xSemaphoreGive(mutex);
}

void ili9431_putpixel(spi_device_handle_t spi, uint16_t x1, uint16_t y1, uint16_t colour)
{
    ili9431_blit(spi, x1, y1, 1, 1, &colour);
}
