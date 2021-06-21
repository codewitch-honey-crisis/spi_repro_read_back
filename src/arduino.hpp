#include <Arduino.h>
#include <SPI.h>
#ifdef ESP_WROVER_KIT
#define LCD_HOST    HSPI

#define PIN_NUM_MISO 25
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  19
#define PIN_NUM_CS   22

#define PIN_NUM_DC   21
#define PIN_NUM_RST  18
#define PIN_NUM_BCKL 5
#else
#define LCD_HOST    VSPI

#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   5

#define PIN_NUM_DC   2
#define PIN_NUM_RST  4
#define PIN_NUM_BCKL 15
#endif
#define CLOCK_SPEED 10*1000*1000

#define READBACK_SIZE 128

typedef struct {
    uint8_t cmd;
    uint8_t data[16];
    uint8_t databytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} lcd_init_cmd_t;

static const lcd_init_cmd_t ili_init_cmds[]={
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
SPIClass spi(LCD_HOST);
/* Send a command to the LCD. Uses spi_device_polling_transmit, which waits
 * until the transfer is complete.
 *
 * Since command transactions are usually small, they are handled in polling
 * mode for higher speed. The overhead of interrupt transactions is more than
 * just waiting for the transaction to complete.
 */
void lcd_cmd(uint8_t cmd)
{
    digitalWrite(PIN_NUM_CS,LOW);
    digitalWrite(PIN_NUM_DC,LOW);
    spi.beginTransaction(SPISettings(CLOCK_SPEED,MSBFIRST,SPI_MODE0));
    spi.transfer(cmd);
    spi.endTransaction();
    digitalWrite(PIN_NUM_CS,HIGH);
    digitalWrite(PIN_NUM_DC,HIGH);
}

/* Send data to the LCD. Uses spi_device_polling_transmit, which waits until the
 * transfer is complete.
 *
 * Since data transactions are usually small, they are handled in polling
 * mode for higher speed. The overhead of interrupt transactions is more than
 * just waiting for the transaction to complete.
 */
void lcd_data(const uint8_t *data, int len)
{
    digitalWrite(PIN_NUM_CS,LOW);
    spi.beginTransaction(SPISettings(CLOCK_SPEED,MSBFIRST,SPI_MODE0));
    spi.writeBytes(data,len);
    spi.endTransaction();
    digitalWrite(PIN_NUM_CS,HIGH);
    
}
void lcd_data(const uint8_t data) {
    digitalWrite(PIN_NUM_CS,LOW);
    spi.beginTransaction(SPISettings(CLOCK_SPEED,MSBFIRST,SPI_MODE0));
    spi.transfer(data);
    spi.endTransaction();
    digitalWrite(PIN_NUM_CS,HIGH);
}

static void fill_screen(uint16_t color)
{
    uint8_t data[4];
    lcd_cmd(0x2a);
    data[0]=data[1]=0;
    data[2]=320>>8;
    data[3]=320&0xFF;
    lcd_data(data,4);
    lcd_cmd(0x2b);
    data[0]=data[1]=0;
    data[2]=240>>8;
    data[3]=240&0xFF;
    lcd_data(data,4);
    lcd_cmd(0x2c);
    uint16_t fdata[320];
    for(int i = 0;i<320;++i) fdata[i]=color;
    for(int y = 0;y<240;++y) {
        lcd_data((uint8_t*)fdata,640);
    }
}
static void read_screen(uint8_t* buffer) {
    uint8_t data[4];
    lcd_cmd(0x2a);
    data[0]=data[1]=0;
    data[2]=READBACK_SIZE>>8;
    data[3]=READBACK_SIZE&0xFF;
    lcd_data(data,4);
    lcd_cmd(0x2b);
    data[0]=data[1]=0;
    data[2]=READBACK_SIZE>>8;
    data[3]=READBACK_SIZE&0xFF;
    lcd_data(data,4);
    lcd_cmd(0x2e);
    digitalWrite(PIN_NUM_CS,LOW);
    spi.beginTransaction(SPISettings(CLOCK_SPEED,MSBFIRST,SPI_MODE0));
    // dummy read - see datasheet
    spi.transferBytes(nullptr,data,1);
    spi.transferBytes(nullptr,buffer,READBACK_SIZE*READBACK_SIZE*2);
    spi.endTransaction();
    digitalWrite(PIN_NUM_CS,HIGH);
}
static void write_screen(uint8_t* buffer) {
    uint8_t data[4];
    lcd_cmd(0x2a);
    data[0]=data[1]=0;
    data[2]=READBACK_SIZE>>8;
    data[3]=READBACK_SIZE&0xFF;
    lcd_data(data,4);
    lcd_cmd(0x2b);
    data[0]=data[1]=0;
    data[2]=READBACK_SIZE>>8;
    data[3]=READBACK_SIZE&0xFF;
    lcd_data(data,4);
    lcd_cmd(0x2c);
    lcd_data(buffer,READBACK_SIZE*READBACK_SIZE*2);
}

//Initialize the display
void lcd_init()
{
    int cmd=0;
    const lcd_init_cmd_t* lcd_init_cmds;

    pinMode(PIN_NUM_CS, OUTPUT);
    //Initialize non-SPI GPIOs
    pinMode(PIN_NUM_DC, OUTPUT);
    pinMode(PIN_NUM_RST, OUTPUT);
    pinMode(PIN_NUM_BCKL, OUTPUT);
    
    //Reset the display
    digitalWrite(PIN_NUM_RST, LOW);
    delay(100);
    digitalWrite(PIN_NUM_RST, HIGH);
    delay(100);
    printf("LCD ILI9341 initialization.\n");
    lcd_init_cmds = ili_init_cmds;

    //Send all the commands
    while (lcd_init_cmds[cmd].databytes!=0xff) {
        lcd_cmd(lcd_init_cmds[cmd].cmd);
        lcd_data(lcd_init_cmds[cmd].data, lcd_init_cmds[cmd].databytes&0x1F);
        if (lcd_init_cmds[cmd].databytes&0x80) {
            delay(100);
        }
        cmd++;
    }

    ///Enable backlight
    digitalWrite(PIN_NUM_BCKL, HIGH);
}
uint8_t buffer[READBACK_SIZE*READBACK_SIZE*2];
void setup() {
    Serial.begin(115200);
    spi.begin(PIN_NUM_CLK,PIN_NUM_MISO,PIN_NUM_MOSI,PIN_NUM_CS);
    lcd_init();
}
void loop() {
    uint64_t total = 0;
    for(int i = 0;i<50;++i) {
        uint16_t color = rand()%65536;
        uint64_t start = esp_timer_get_time();
        fill_screen(color);
        
        read_screen(buffer);
        write_screen(buffer);
        uint64_t end = esp_timer_get_time();
        total += (end-start);
        delay(1000);
    }
    
}