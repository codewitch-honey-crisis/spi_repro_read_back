/* SPI Master example
   This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
extern "C" { void app_main(); }

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_err.h"

#ifdef ESP_WROVER_KIT
#define LCD_HOST    HSPI_HOST

#define PIN_NUM_MISO GPIO_NUM_25
#define PIN_NUM_MOSI GPIO_NUM_23
#define PIN_NUM_CLK  GPIO_NUM_19
#define PIN_NUM_CS   GPIO_NUM_22

#define PIN_NUM_DC   GPIO_NUM_21
#define PIN_NUM_RST  GPIO_NUM_18
#define PIN_NUM_BCKL GPIO_NUM_5
#else
#define LCD_HOST    VSPI_HOST

#define PIN_NUM_MISO GPIO_NUM_19
#define PIN_NUM_MOSI GPIO_NUM_23
#define PIN_NUM_CLK  GPIO_NUM_18
#define PIN_NUM_CS   GPIO_NUM_5

#define PIN_NUM_DC   GPIO_NUM_2
#define PIN_NUM_RST  GPIO_NUM_4
#define PIN_NUM_BCKL GPIO_NUM_15
#endif

#define DMA_CHAN    2

#define CLOCK_SPEED 10*1000*1000
#define PARALLEL_LINES 16
#define READBACK_SIZE 128
// when i set this to 1 the code hangs
#define DUMMY_BITS 0
//#define USE_NO_DUMMY_FLAG


/*
 The LCD needs a bunch of command/argument values to be initialized. They are stored in this struct.
*/
typedef struct {
    uint8_t cmd;
    uint8_t data[16];
    uint8_t databytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} lcd_init_cmd_t;

DRAM_ATTR static const lcd_init_cmd_t ili_init_cmds[]={
    /* Power contorl B, power control = 0, DC_ENA = 1 */
    {0xCF, {0x00, 0x83, 0X30}, 3},
    /* Power on sequence control,
     * cp1 keeps 1 frame, 1st frame enable
     * vcl = 0, ddvdh=3, vgh=1, vgl=2
     * DDVDH_ENH=1
     */
    {0xED, {0x64, 0x03, 0X12, 0X81}, 4},
    /* Driver timing control A,
     * non-overlap=default +1
     * EQ=default - 1, CR=default
     * pre-charge=default - 1
     */
    {0xE8, {0x85, 0x01, 0x79}, 3},
    /* Power control A, Vcore=1.6V, DDVDH=5.6V */
    {0xCB, {0x39, 0x2C, 0x00, 0x34, 0x02}, 5},
    /* Pump ratio control, DDVDH=2xVCl */
    {0xF7, {0x20}, 1},
    /* Driver timing control, all=0 unit */
    {0xEA, {0x00, 0x00}, 2},
    /* Power control 1, GVDD=4.75V */
    {0xC0, {0x26}, 1},
    /* Power control 2, DDVDH=VCl*2, VGH=VCl*7, VGL=-VCl*3 */
    {0xC1, {0x11}, 1},
    /* VCOM control 1, VCOMH=4.025V, VCOML=-0.950V */
    {0xC5, {0x35, 0x3E}, 2},
    /* VCOM control 2, VCOMH=VMH-2, VCOML=VML-2 */
    {0xC7, {0xBE}, 1},
    /* Memory access contorl, MX=MY=0, MV=1, ML=0, BGR=1, MH=0 */
    {0x36, {0x28}, 1},
    /* Pixel format, 16bits/pixel for RGB/MCU interface */
    {0x3A, {0x55}, 1},
    /* Frame rate control, f=fosc, 70Hz fps */
    {0xB1, {0x00, 0x1B}, 2},
    /* Enable 3G, disabled */
    {0xF2, {0x08}, 1},
    /* Gamma set, curve 1 */
    {0x26, {0x01}, 1},
    /* Positive gamma correction */
    {0xE0, {0x1F, 0x1A, 0x18, 0x0A, 0x0F, 0x06, 0x45, 0X87, 0x32, 0x0A, 0x07, 0x02, 0x07, 0x05, 0x00}, 15},
    /* Negative gamma correction */
    {0XE1, {0x00, 0x25, 0x27, 0x05, 0x10, 0x09, 0x3A, 0x78, 0x4D, 0x05, 0x18, 0x0D, 0x38, 0x3A, 0x1F}, 15},
    /* Column address set, SC=0, EC=0xEF */
    {0x2A, {0x00, 0x00, 0x00, 0xEF}, 4},
    /* Page address set, SP=0, EP=0x013F */
    {0x2B, {0x00, 0x00, 0x01, 0x3f}, 4},
    /* Memory write */
    {0x2C, {0}, 0},
    /* Entry mode set, Low vol detect disabled, normal display */
    {0xB7, {0x07}, 1},
    /* Display function control */
    {0xB6, {0x0A, 0x82, 0x27, 0x00}, 4},
    /* Sleep out */
    {0x11, {0}, 0x80},
    /* Display on */
    {0x29, {0}, 0x80},
    {0, {0}, 0xff},
};

/* Send a command to the LCD. Uses spi_device_polling_transmit, which waits
 * until the transfer is complete.
 *
 * Since command transactions are usually small, they are handled in polling
 * mode for higher speed. The overhead of interrupt transactions is more than
 * just waiting for the transaction to complete.
 */
void lcd_cmd(spi_device_handle_t spi, const uint8_t cmd)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=8;                     //Command is 8 bits
    t.tx_buffer=&cmd;               //The data is the cmd itself
    //t.user=(void*)0;                //D/C needs to be set to 0
    gpio_set_level(PIN_NUM_DC, 0);
    spi_device_polling_transmit(spi, &t);  //Transmit!
    gpio_set_level(PIN_NUM_DC, 1);
}

/* Send data to the LCD. Uses spi_device_polling_transmit, which waits until the
 * transfer is complete.
 *
 * Since data transactions are usually small, they are handled in polling
 * mode for higher speed. The overhead of interrupt transactions is more than
 * just waiting for the transaction to complete.
 */
void lcd_data(spi_device_handle_t spi, const uint8_t *data, int len)
{
    spi_transaction_t t;
    if (len==0) return;             //no need to send anything
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=len*8;                 //Len is in bytes, transaction length is in bits.
    t.tx_buffer=data;               //Data
    //t.user=(void*)1;                //D/C needs to be set to 1
//    gpio_set_level(PIN_NUM_DC, 1);
    spi_device_polling_transmit(spi, &t);  //Transmit!
    
}

//Initialize the display
void lcd_init(spi_device_handle_t spi)
{
    int cmd=0;
    const lcd_init_cmd_t* lcd_init_cmds;

    //Initialize non-SPI GPIOs
    gpio_set_direction(PIN_NUM_DC, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_RST, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_BCKL, GPIO_MODE_OUTPUT);

    //Reset the display
    gpio_set_level(PIN_NUM_RST, 0);
    vTaskDelay(100 / portTICK_RATE_MS);
    gpio_set_level(PIN_NUM_RST, 1);
    vTaskDelay(100 / portTICK_RATE_MS);

        lcd_init_cmds = ili_init_cmds;

    //Send all the commands
    while (lcd_init_cmds[cmd].databytes!=0xff) {
        lcd_cmd(spi, lcd_init_cmds[cmd].cmd);
        lcd_data(spi, lcd_init_cmds[cmd].data, lcd_init_cmds[cmd].databytes&0x1F);
        if (lcd_init_cmds[cmd].databytes&0x80) {
            vTaskDelay(100 / portTICK_RATE_MS);
        }
        cmd++;
    }

    ///Enable backlight
    gpio_set_level(PIN_NUM_BCKL, 0);
}

uint16_t fdata[320*PARALLEL_LINES];

static void fill_screen(spi_device_handle_t spi, uint16_t color)
{
    uint8_t data[4];
    lcd_cmd(spi,0x2a);
    data[0]=data[1]=0;
    data[2]=320>>8;
    data[3]=320&0xFF;
    lcd_data(spi,data,4);
    lcd_cmd(spi,0x2b);
    data[0]=data[1]=0;
    data[2]=240>>8;
    data[3]=240&0xFF;
    lcd_data(spi,data,4);
    lcd_cmd(spi,0x2c);
    
    for(int i = 0;i<320*PARALLEL_LINES;++i) fdata[i]=color;
    for(int y = 0;y<240;y+=PARALLEL_LINES) {
        lcd_data(spi,(uint8_t*)fdata,320*2*PARALLEL_LINES);
    }
}
static void read_screen(spi_device_handle_t spi, uint8_t* buffer) {
    uint8_t data[4];
    lcd_cmd(spi,0x2a);
    data[0]=data[1]=0;
    data[2]=READBACK_SIZE>>8;
    data[3]=READBACK_SIZE&0xFF;
    lcd_data(spi,data,4);
    lcd_cmd(spi,0x2b);
    data[0]=data[1]=0;
    data[2]=READBACK_SIZE>>8;
    data[3]=READBACK_SIZE&0xFF;
    lcd_data(spi,data,4);
    lcd_cmd(spi,0x2e);
    spi_transaction_t trans;
    trans.addr=trans.cmd=trans.rxlength=0;
    trans.flags=0;
    trans.user=nullptr;
    trans.tx_buffer=nullptr;
    // extra byte for the dummy parameter
    trans.length = 8*(READBACK_SIZE*READBACK_SIZE*2+1);
    trans.rx_buffer = buffer;
    spi_device_polling_transmit(spi,&trans);
}
static void write_screen(spi_device_handle_t spi, uint8_t* buffer) {
    uint8_t data[4];
    lcd_cmd(spi,0x2a);
    data[0]=data[1]=0;
    data[2]=READBACK_SIZE>>8;
    data[3]=READBACK_SIZE&0xFF;
    lcd_data(spi,data,4);
    lcd_cmd(spi,0x2b);
    data[0]=data[1]=0;
    data[2]=READBACK_SIZE>>8;
    data[3]=READBACK_SIZE&0xFF;
    lcd_data(spi,data,4);
    lcd_cmd(spi,0x2c);
    // advance past the dummy parameter:
    lcd_data(spi,buffer+1,READBACK_SIZE*READBACK_SIZE*2);
}
// we need an extra byte for the dummy parameter
uint8_t buffer[READBACK_SIZE*READBACK_SIZE*2+1];
void app_main(void)
{
    spi_device_handle_t spi;
    const int rbs = READBACK_SIZE*READBACK_SIZE*2+9;
    const int pls = PARALLEL_LINES*2*320+8;
    spi_bus_config_t buscfg={
        .mosi_io_num=PIN_NUM_MOSI,
        .miso_io_num=PIN_NUM_MISO,
        .sclk_io_num=PIN_NUM_CLK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1,
        .max_transfer_sz=(rbs>pls)?rbs:pls,
        .flags = 0,
        .intr_flags =0
    };
    spi_device_interface_config_t devcfg={
        .command_bits=0,
        .address_bits=0,
        .dummy_bits=DUMMY_BITS,
        .mode=0,
        .duty_cycle_pos=0,
        .cs_ena_pretrans=0,
        .cs_ena_posttrans=0,
        .clock_speed_hz=CLOCK_SPEED,           //Clock out at 26 MHz
        .input_delay_ns = 0,
        .spics_io_num=PIN_NUM_CS,               //CS pin
#ifdef USE_NO_DUMMY_FLAG
        .flags =SPI_DEVICE_NO_DUMMY,
#else
        .flags =0,
#endif
        .queue_size=7,                          //We want to be able to queue 7 transactions at a time
        .pre_cb=NULL,  //Specify pre-transfer callback to handle D/C line
        .post_cb=NULL
    };
    //Initialize the SPI bus
    esp_err_t ret=spi_bus_initialize(LCD_HOST, &buscfg, DMA_CHAN);
    ESP_ERROR_CHECK(ret);
    //Attach the LCD to the SPI bus
    ret=spi_bus_add_device(LCD_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);
    //Initialize the LCD
    lcd_init(spi);
    
    while(1) {
        uint64_t total = 0;
        for(int i = 0;i<50;++i) {
            uint16_t color = rand()%65536;
            uint64_t start = esp_timer_get_time();
            fill_screen(spi,color);
            read_screen(spi,buffer);
            write_screen(spi,buffer);
            uint64_t end = esp_timer_get_time();
            total += (end-start);
            vTaskDelay(1000/portTICK_PERIOD_MS);
        }
        printf("Average frame write over 50 iterations is %llu microseconds per frame\r\n",total/50);
    }
}