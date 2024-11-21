#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_rom_sys.h"
#include "esp_log.h"

static const char *MY_TAG = "MAIN";

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"
#include "driver/dac.h"
#include "esp_avrc_api.h"

#include "esp_timer.h"

#include "AudioTools.h"
#include "BluetoothA2DPSink.h"


AnalogAudioStream out;
BluetoothA2DPSink a2dp_sink(out);

const gpio_num_t MELBUS_DATA = GPIO_NUM_21;     
const gpio_num_t MELBUS_CLOCKBIT = GPIO_NUM_19; 
const gpio_num_t MELBUS_BUSY = GPIO_NUM_18;     

volatile uint8_t melbus_ReceivedByte = 0;
uint8_t byteToSendINT = 0;
volatile uint8_t melbus_LastReadByte[8] = {0};
volatile uint8_t melbus_Bitposition = 7;
volatile uint8_t melbus_Bitposition_Send = 7;
volatile long Counter = 0;
uint8_t melbus_log[99];

volatile bool InitialSequence = false;
volatile bool byteIsRead = false;
volatile bool byteIsSend = false;
volatile bool Connected = false;
bool reqMasterFlag = false;
bool runMaster = false;


#define RESPONSE_ID 0xC5 // ID while responding to init requests (which will use base_id)
#define BASE_ID 0xC0     // ID when getting commands from HU
#define MASTER_ID 0xC7

#define CDC_RESPONSE_ID 0xEE // ID while responding to init requests (which will use base_id)
#define CDC_MASTER_ID 0xEF   // ID while requesting/beeing master
#define CDC_BASE_ID 0xE8     // ID when getting commands from HU
#define CDC_ALT_ID 0xE9      // Alternative ID when getting commands from HU

uint8_t textHeader[] = {0xFC, 0xC6, 0x73, 0x01};
uint8_t textRow = 2;
uint8_t customText[4][36] = {
    {""},
    {"visualapproach"},
    {""},
    {""}};

// HU asks for line 3 and 4 below at startup. They can be overwritten by customText if you change textRow to 3 or 4
uint8_t textLine[4][36] = {
    {"Line 1"},        // is overwritten by init-sequence ("Volvo!")
    {"Line 2"},        // is overwritten by customText[1][]
    {"3=cycle L col"}, // changes if pressing 3-button
    {"4=cycle R col"}  // changes if pressing 4-button
};

const uint8_t C1_Init_1[9] = {
    0x10, 0x00, 0xc3, 0x01,
    0x00, 0x81, 0x01, 0xff,
    0x00};
const uint8_t SO_C1_Init_1 = 9;

const uint8_t C1_Init_2[11] = {
    0x10, 0x01, 0x81,
    'V', 'o', 'l', 'v', 'o', '!', ' ', ' '};
const uint8_t SO_C1_Init_2 = 11;

const uint8_t C2_Init_1[] = {
    0x10, 0x01, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff};
const uint8_t SO_C2_Init_1 = 19;

const uint8_t C3_Init_0[30] = {
    0x10, 0x00, 0xfe, 0xff,
    0xff, 0xdf, 0x3f, 0x29,
    0x2c, 0xf0, 0xde, 0x2f,
    0x61, 0xf4, 0xf4, 0xdf,
    0xdd, 0xbf, 0xff, 0xbe,
    0xff, 0xff, 0x03, 0x00,
    0xe0, 0x05, 0x40, 0x00,
    0x00, 0x00};
const uint8_t SO_C3_Init_0 = 30;

const uint8_t C3_Init_1[30] = {
    0x10, 0x01, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff,
    0xff, 0xff};
const uint8_t SO_C3_Init_1 = 30;

const uint8_t C3_Init_2[30] = {
    0x10, 0x02, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff,
    0xff, 0xff};
const uint8_t SO_C3_Init_2 = 30;

// Defining the commands. First byte is the length of the command.
#define MRB_1 {3, 0x00, 0x1C, 0xEC} // Master Request Broadcast version 1
#define MRB_2 {3, 0x00, 0x1E, 0xEC} // Master Request Broadcast version 2 (maybe this is second init seq?)
#define MI {3, 0x07, 0x1A, 0xEE}    // Main init sequence
#define SI {3, 0x00, 0x1E, 0xED}    // Secondary init sequence (turn off ignition, then on)
// changed from 00 1D ED

#define C1_1 {5, 0xC1, 0x1B, 0x7F, 0x01, 0x08} // Respond with c1_init_1
#define C1_2 {5, 0xC1, 0x1D, 0x73, 0x01, 0x81} // Respond with c1_init_2 (text)

#define C2_0 {4, 0xC2, 0x1D, 0x73, 0x00} // Get next byte (nn) and respond with 10, 0, nn, 0,0 and 14 * 0x20 (possibly text)
#define C2_1 {4, 0xC2, 0x1D, 0x73, 0x01} // Same as above? Answer 19 bytes (unknown)

#define C3_0 {4, 0xC3, 0x1F, 0x7C, 0x00} // Respond with c1_init_2 (text)
#define C3_1 {4, 0xC3, 0x1F, 0x7C, 0x01} // Respond with c1_init_2 (text)
#define C3_2 {4, 0xC3, 0x1F, 0x7C, 0x02} // Respond with c1_init_2 (text)

#define C5_1 {3, 0xC5, 0x19, 0x73} // C5, 19, 73, xx, yy. Answer 0x10, xx, yy + free text. End with 00 00 and pad with spaces

#define CMD_1 {3, 0xC0, 0x1B, 0x76}                     // Followed by: [00, 92, FF], OR [01, 03 ,FF] OR [02, 05, FF]. Answer 0x10
#define PUP {4, 0xC0, 0x1C, 0x70, 0x02}                 // Wait 2 bytes and answer 0x90?
#define CMD_3 {5, 0xC0, 0x1D, 0x76, 0x80, 0x00}         // Answer: 0x10, 0x80, 0x92
#define PWR_OFF {6, 0xC0, 0x1C, 0x70, 0x00, 0x80, 0x01} // answer one byte
#define PWR_SBY {6, 0xC0, 0x1C, 0x70, 0x01, 0x80, 0x01} // answer one byte
#define IGN_OFF {3, 0x00, 0x18, 0x12}                   // this is the last message before HU goes to Nirvana

#define BTN {4, 0xC0, 0x1D, 0x77, 0x81}       // Read next byte which is the button #. Respond with 3 bytes
#define NXT {5, 0xC0, 0x1B, 0x71, 0x80, 0x00} // Answer 1 byte
#define PRV {5, 0xC0, 0x1B, 0x71, 0x00, 0x00} // Answer 1 byte
#define SCN {4, 0xC0, 0x1A, 0x74, 0x2A}       // Answer 1 byte

#define CDC_CIR {3, CDC_BASE_ID, 0x1E, 0xEF}             // Cartridge info request. Respond with 6 bytes
#define CDC_TIR {5, CDC_ALT_ID, 0x1B, 0xE0, 0x01, 0x08}  // track info req. resp 9 bytes
#define CDC_NXT {5, CDC_BASE_ID, 0x1B, 0x2D, 0x40, 0x01} // next track.
#define CDC_PRV {5, CDC_BASE_ID, 0x1B, 0x2D, 0x00, 0x01} // prev track
#define CDC_CHG {3, CDC_BASE_ID, 0x1A, 0x50}             // change cd
#define CDC_PUP {3, CDC_BASE_ID, 0x19, 0x2F}             // power up. resp ack (0x00).
#define CDC_PDN {3, CDC_BASE_ID, 0x19, 0x22}             // power down. ack (0x00)
#define CDC_FFW {3, CDC_BASE_ID, 0x19, 0x29}             // FFW. ack
#define CDC_FRW {3, CDC_BASE_ID, 0x19, 0x26}             // FRW. ack
#define CDC_SCN {3, CDC_BASE_ID, 0x19, 0x2E}             // scan mode. ack
#define CDC_RND {3, CDC_BASE_ID, 0x19, 0x52}             // random mode. ack
#define CDC_NU {3, CDC_BASE_ID, 0x1A, 0x50}              // not used
// #define CDC_MI {0x07, 0x1A, 0xEE},         //main init seq. wait for BASE_ID and respond with RESPONSE_ID.
// #define CDC_SI {0x00, 0x1C, 0xED},         //secondary init req. wait for BASE_ID and respond with RESPONSE_ID.
// #define CDC_MRB {0x00, 0x1C, 0xEC}         //master req broadcast. wait for MASTER_ID and respond with MASTER_ID.

// This list can be quite long. We have approx 700 us between the received bytes.
const uint8_t commands[][7] = {
    MRB_1,   // 0 now we are master and can send stuff (like text) to the display!
    MI,      // 1 main init
    SI,      // 2 sec init (00 1E ED respond 0xC5 !!)
    CMD_1,   // 3 follows: [0, 92, FF], OR [1,3 ,FF] OR [2, 5 FF]
    PUP,     // 4 wait 2 bytes and answer 0x90?
    MRB_2,   // 5 alternative master req bc
    CMD_3,   // 6 unknown. Answer: 0x10, 0x80, 0x92
    C1_1,    // 7 respond with c1_init_1
    C1_2,    // 8 respond with c1_init_2 (contains text)
    C3_0,    // 9 respond with c3_init_0
    C3_1,    // 10 respond with c3_init_1
    C3_2,    // 11 respond with c3_init_2
    C2_0,    // 12 get next byte (nn) and respond with 10, 0, nn, 0,0 and 14 of 0x20
    C2_1,    // 13
    C5_1,    // 14
    BTN,     // 15
    NXT,     // 16
    PRV,     // 17
    SCN,     // 18
    PWR_OFF, // 19
    PWR_SBY, // 20
    IGN_OFF, // 21
    CDC_CIR, // 22
    CDC_TIR, // 23
    CDC_NXT, // 24
    CDC_PRV, // 25
    CDC_CHG, // 26
    CDC_PUP, // 27
    CDC_PDN, // 28
    CDC_FFW, // 29
    CDC_FRW, // 30
    CDC_SCN, // 31
    CDC_RND, // 32
    CDC_NU   // 33
};

const uint8_t listLen = 34; // how many rows in the above array

// some CDC (CD-CHANGER) data
uint8_t track = 0x01; // Display show HEX value, not DEC. (A-F not "allowed")
uint8_t cd = 0x01;    // 1-10 is allowed (in HEX. 0A-0F and 1A-1F is not allowed)
//uint8_t trackInfo[] = {0x00, 0x02, 0x00, cd, 0x80, track, 0xC7, 0x0A, 0x02}; // 9 bytes
uint8_t trackInfo[] = {0x00, 0x02, 0x00, 0x00, 0x80, 0x99, 0x0C, 0xCC, 0xCC};
uint8_t startByte = 0x08; // on powerup - change trackInfo[1] & [8] to this
uint8_t stopByte = 0x02;  // same on powerdown
// uint8_t cartridgeInfo[] = {0x00, 0xFC, 0xFF, 0x4A, 0xFC, 0xFF};
uint8_t cartridgeInfo[] = {0x00, 0x00, 0x1F, 0x4A, 0x0, 0x1F};
// uint8_t cartridgeInfo[] = {0x00, 0x0F, 0xFF, 0x4A, 0xFC, 0xFF};
// uint8_t cartridgeInfo[] = {0x80, 0x00, 0x0F, 0x04, 0x00, 0x0F};

static long timeout = 10000000;       // should be around 10-20 secs
static long runOnce = 300000;         // counts down on every received message from HU. Triggers when it is passing 1.
static long runPeriodically = 100000; // same as runOnce but resets after each countdown.

// Interrupt CLOCK
void IRAM_ATTR MELBUS_CLOCK_INTERRUPT(void *arg)
{
    if (gpio_get_level(MELBUS_DATA))
    {
        melbus_ReceivedByte |= (1 << melbus_Bitposition);
    }
    else
    {
        melbus_ReceivedByte &= ~(1 << melbus_Bitposition);
    }

    // Checking for completion of byte reading
    if (melbus_Bitposition == 0)
    {
        // Shifting the array of read bytes
        for (int i = 7; i > 0; i--)
        {
            melbus_LastReadByte[i] = melbus_LastReadByte[i - 1];
        }
        melbus_LastReadByte[0] = melbus_ReceivedByte;
        byteIsRead = true;
        melbus_Bitposition = 7;
    }
    else
    {
        melbus_Bitposition--;
    }
}

// printing an array of bytes
void print_byte_array(const volatile uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++)
    {
        ESP_LOGI(MY_TAG, "Byte %d: 0x%02X", i, data[i]);
    }
}

// CD-CHGR initialization function
void melbus_Init_CDCHRG()
{
    gpio_intr_disable(MELBUS_CLOCKBIT);
    // Waiting until the BUSY line becomes HIGH
    while (!gpio_get_level(MELBUS_BUSY))
    {
    }
    esp_rom_delay_us(20);

    // Setting up the BUSY pin as an exit
    gpio_set_direction(MELBUS_BUSY, GPIO_MODE_OUTPUT);
    gpio_set_level(MELBUS_BUSY, 0);
    //esp_rom_delay_us(1200000);
    vTaskDelay(1200 / portTICK_PERIOD_MS); // 1,2 sec
    gpio_set_level(MELBUS_BUSY, 1);
    gpio_set_direction(MELBUS_BUSY, GPIO_MODE_INPUT);

    // Enable interrupt
    gpio_intr_enable(MELBUS_CLOCKBIT);
}

// The function of sending a byte to MELBUS
void SendByteToMelbus(uint8_t byteToSend)
{
    // disable interrupt
    gpio_intr_disable(MELBUS_CLOCKBIT);
    //  Configuring the DATA pin as an output
    gpio_set_direction(MELBUS_DATA, GPIO_MODE_OUTPUT);
    gpio_set_level(MELBUS_DATA, 1);
    //  Configuring the CLOCK pin as an input
    gpio_set_direction(MELBUS_CLOCKBIT, GPIO_MODE_INPUT);

    // Sending a byte
    for (int i = 7; i >= 0; i--)
    {
        while (gpio_get_level(MELBUS_CLOCKBIT) == 1)
        {
        } //  Waiting for CLK to become LOW

        gpio_set_level(MELBUS_DATA, byteToSend & (1 << i)); // Установить бит

        while (gpio_get_level(MELBUS_CLOCKBIT) == 0)
        {
        } // Waiting for CLK to become HIGH
    }

    esp_rom_delay_us(5);

    // Returning the DATA pin to the input mode
    gpio_set_level(MELBUS_DATA, 1);
    gpio_set_direction(MELBUS_DATA, GPIO_MODE_INPUT);
    gpio_set_direction(MELBUS_CLOCKBIT, GPIO_MODE_INPUT);
    gpio_intr_enable(MELBUS_CLOCKBIT);
}

void SendByteToMelbus2(uint8_t uint8_tToSend)
{
    esp_rom_delay_us(700);
    

    for (int i = 7; i >= 0; i--)
    {
        esp_rom_delay_us(7);
        // PORTD &= ~(1 << MELBUS_CLOCKBIT);  //clock -> low
        gpio_set_level(MELBUS_CLOCKBIT, 0);
        // If bit [i] is "1" - make datapin high
        if (uint8_tToSend & (1 << i))
        {
            // PORTD |= (1 << MELBUS_DATA);
            gpio_set_level(MELBUS_DATA, 1);
        }
        // if bit [i] is "0" - make datapin low
        else
        {
            // PORTD &= ~(1 << MELBUS_DATA);
            gpio_set_level(MELBUS_DATA, 0);
        }
        // wait for output to settle
        esp_rom_delay_us(5);
        // PORTD |= (1 << MELBUS_CLOCKBIT);   //clock -> high
        gpio_set_level(MELBUS_CLOCKBIT, 1);
        // wait for HU to read the bit
    }
    esp_rom_delay_us(20);
}

void SendText(uint8_t rowNum)
{
    gpio_intr_disable(MELBUS_CLOCKBIT);
    gpio_set_level(MELBUS_DATA, 1);                        // Устанавливаем пин DATA в HIGH
    gpio_set_direction(MELBUS_DATA, GPIO_MODE_OUTPUT);     // Устанавливаем пин DATA как выход
    gpio_set_level(MELBUS_CLOCKBIT, 1);                    // Устанавливаем пин CLOCK в HIGH
    gpio_set_direction(MELBUS_CLOCKBIT, GPIO_MODE_OUTPUT); // Устанавливаем пин CLOCK как выход
    // send header
    for (uint8_t b = 0; b < 4; b++)
    {
        // uint8_tToSend = textHeader[b];
        SendByteToMelbus2(textHeader[b]);
    }

    // send which row to show it on
    // uint8_tToSend = rowNum;
    SendByteToMelbus2(rowNum);

    // send text
    for (uint8_t b = 0; b < 36; b++)
    {
        SendByteToMelbus2(customText[rowNum - 1][b]);
    }

    gpio_set_direction(MELBUS_CLOCKBIT, GPIO_MODE_INPUT);
    gpio_set_level(MELBUS_DATA, 1);
    gpio_set_direction(MELBUS_DATA, GPIO_MODE_INPUT);
    gpio_intr_enable(MELBUS_CLOCKBIT);

    for (uint8_t b = 0; b < 36; b++)
    {
        Serial.print(char(customText[rowNum - 1][b]));
    }
    Serial.println("  END");
}

void reqMaster()
{
    gpio_set_direction(MELBUS_DATA, GPIO_MODE_OUTPUT); 
    gpio_set_level(MELBUS_DATA, 0);
    // esp_rom_delay_us(700);
    // esp_rom_delay_us(800);
    // esp_rom_delay_us(800);
    vTaskDelay(2.3/ portTICK_PERIOD_MS);
    gpio_set_level(MELBUS_DATA, 1);
    gpio_pullup_en(MELBUS_DATA);
    gpio_set_direction(MELBUS_DATA, GPIO_MODE_INPUT); 
}

void fixTrack()
{
    // cut out A-F in each nibble, and skip "00"
    uint8_t hn = track >> 4;
    uint8_t ln = track & 0xF;
    if (ln == 0xA)
    {
        ln = 0;
        hn += 1;
    }
    if (ln == 0xF)
    {
        ln = 9;
    }
    if (hn == 0xA)
    {
        hn = 0;
        ln = 1;
    }
    if ((hn == 0) && (ln == 0))
    {
        ln = 0x9;
        hn = 0x9;
    }
    track = (hn << 4) + ln;
}

void changeCD()
{
    while (!gpio_get_level(MELBUS_BUSY))
    {
        if (byteIsRead)
        {
            byteIsRead = false;
            switch (melbus_ReceivedByte)
            {
            // 0x81 to 0x86 corresponds to cd buttons 1 to 6 on the HU (650)
            case 0x81:
                cd = 1;
                track = 1;
                break;
            case 0x82:
                cd = 2;
                track = 1;
                break;
            case 0x83:
                cd = 3;
                track = 1;
                break;
            case 0x84:
                cd = 4;
                track = 1;
                break;
            case 0x85:
                cd = 5;
                track = 1;
                break;
            case 0x86:
                cd = 6;
                track = 1;
                break;
            case 0x41: // next cd
                cd++;
                track = 1;
                break;
            case 0x01: // prev cd
                cd--;
                track = 1;
                break;
            default:
                track = 1;
                break;
            }
        }
    }
    trackInfo[3] = cd;
    trackInfo[5] = track;
}

void SendTrackInfo()
{
    for (uint8_t i = 0; i < 9; i++)
    {
        SendByteToMelbus(trackInfo[i]);
    }
}

void SendCartridgeInfo()
{
    for (uint8_t i = 0; i < 6; i++)
    {
        SendByteToMelbus(cartridgeInfo[i]);
    }
}

void setup()
{
    esp_task_wdt_deinit();
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << MELBUS_DATA) | (1ULL << MELBUS_CLOCKBIT) | (1ULL << MELBUS_BUSY),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    // Binding the interrupt to the CLOCK pin
    gpio_set_intr_type(MELBUS_CLOCKBIT, GPIO_INTR_POSEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(MELBUS_CLOCKBIT, MELBUS_CLOCK_INTERRUPT, NULL);

    // disabling bluetooth logs
    //esp_log_level_set("BT_AV", ESP_LOG_NONE);
    // esp_log_level_set("BT_API", ESP_LOG_NONE);
    // esp_log_level_set("BTDM_INIT", ESP_LOG_NONE);
    // esp_log_level_set("BT_BTC", ESP_LOG_NONE);
    // esp_log_level_set("BT_APPL", ESP_LOG_NONE);
    // esp_log_level_set("BT_LOG", ESP_LOG_NONE);

}

void loop()
{
    setup();
    ESP_LOGI(MY_TAG, "Initializing communication with Volvo-Melbus:");

    // Initializing CD-CHGR
    melbus_Init_CDCHRG();
    while (1)
    {
        // static uint8_t lastByte = 0;       // used to copy volatile uint8_t to register variable. See below
        static long runOnce = 300000;         // counts down on every received message from HU. Triggers when it is passing 1.
        static long runPeriodically = 100000; // same as runOnce but resets after each countdown.
        static bool powerOn = true;
        static long HWTicks = 0;   // age since last BUSY switch
        static long ComTicks = 0;  // age since last received uint8_t
        static long ConnTicks = 0; // age since last message to SIRIUS SAT

        // static char text_array[17] = {0};

        // these variables are reset every loop
        // uint8_t byteCounter = 1;                 // keep track of how many uint8_ts is sent in current command
        // main init sequence is 61 uint8_ts long...
        bool BUSY = gpio_get_level(MELBUS_BUSY); // PIND & (1 << MELBUS_BUSY);

        HWTicks++;
        if (powerOn)
        {
            ComTicks++;
            ConnTicks++;
        }
        else
        {
            ComTicks = 1;
            ConnTicks = 1;
            //  to avoid a lot of serial.prints when its 0
        }

        while (!BUSY)
        {
            HWTicks = 0; // reset age

            // Transmission handling here...
            if (byteIsRead)
            {
                byteIsRead = false;
                ComTicks = 0; // reset age
                // melbus_log[byteCounter - 1] = melbus_ReceivedByte;
                // byteCounter++;
                uint8_t b1 = 0, b2 = 0, cnt = 0;

                // MI  0x07, 0x1A, 0xEE
                if (melbus_LastReadByte[2] == 0x07 && melbus_LastReadByte[1] == 0x1A && melbus_LastReadByte[0] == 0xEE)
                {
                    ESP_LOGI(MY_TAG, "MI");
                    ConnTicks = 0;
                    while (!gpio_get_level(MELBUS_BUSY))
                    {
                        if (byteIsRead)
                        {
                            byteIsRead = false;
                            // The ability to enable SAT
                            // if (melbus_ReceivedByte == BASE_ID)
                            // {
                            //     SendByteToMelbus(RESPONSE_ID);
                            // }
                            if (melbus_ReceivedByte == CDC_BASE_ID)
                            {
                                SendByteToMelbus(CDC_RESPONSE_ID);
                            }
                        }
                    }
                }

                // SI  0x00, 0x1E, 0xED
                else if (melbus_LastReadByte[2] == 0x00 && melbus_LastReadByte[1] == 0x1E && melbus_LastReadByte[0] == 0xED)
                {
                    ESP_LOGI(MY_TAG, "SI");
                    ConnTicks = 0;
                    while (!gpio_get_level(MELBUS_BUSY))
                    {
                        if (byteIsRead)
                        {
                            byteIsRead = false;
                            if (melbus_ReceivedByte == CDC_BASE_ID)
                            {
                                SendByteToMelbus(CDC_RESPONSE_ID);
                            }
                            if (melbus_ReceivedByte == BASE_ID)
                            {
                                SendByteToMelbus(RESPONSE_ID);
                            }
                        }
                    }
                }

                // MRB_1  0x00, 0x1C, 0xEC
                else if (melbus_LastReadByte[2] == 0x00 && melbus_LastReadByte[1] == 0x1C && melbus_LastReadByte[0] == 0xEC)
                {
                    ESP_LOGI(MY_TAG, "MRB_1");
                    ConnTicks = 0;
                    while (!gpio_get_level(MELBUS_BUSY))
                    {
                        if (byteIsRead)
                        {
                            byteIsRead = false;
                            if (melbus_ReceivedByte == MASTER_ID)
                            {
                                SendByteToMelbus(MASTER_ID);
                                SendText(textRow);
                                break;
                            }
                        }
                    }
                }

                // MRB_2  0x00, 0x1E, 0xEC
                else if (melbus_LastReadByte[2] == 0x00 && melbus_LastReadByte[1] == 0x1E && melbus_LastReadByte[0] == 0xEC)
                {
                    ESP_LOGI(MY_TAG, "MRB_2");
                    ConnTicks = 0;
                    while (!gpio_get_level(MELBUS_BUSY))
                    {
                        if (byteIsRead)
                        {
                            byteIsRead = false;
                            if (melbus_ReceivedByte == MASTER_ID)
                            {
                                SendByteToMelbus(MASTER_ID);
                                SendText(textRow);
                                break;
                            }
                        }
                    }
                }

                // C1_1  0xC1, 0x1B, 0x7F, 0x01, 0x08
                else if (melbus_LastReadByte[4] == 0xC1 && melbus_LastReadByte[3] == 0x1B && melbus_LastReadByte[2] == 0x7F && melbus_LastReadByte[1] == 0x01 && melbus_LastReadByte[0] == 0x08)
                {
                    ESP_LOGI(MY_TAG, "C1_1");
                    ConnTicks = 0;
                    for (uint8_t i = 0; i < SO_C1_Init_1; i++)
                    {
                        SendByteToMelbus(C1_Init_1[i]);
                    }
                }

                // C1_2  0xC1, 0x1D, 0x73, 0x01, 0x81
                else if (melbus_LastReadByte[4] == 0xC1 && melbus_LastReadByte[3] == 0x1D && melbus_LastReadByte[2] == 0x73 && melbus_LastReadByte[1] == 0x01 && melbus_LastReadByte[0] == 0x81)
                {
                    ESP_LOGI(MY_TAG, "C1_2");
                    ConnTicks = 0;
                    for (uint8_t i = 0; i < SO_C1_Init_2; i++)
                    {
                        SendByteToMelbus(C1_Init_2[i]);
                    }
                }

                // C2_0  0xC2, 0x1D, 0x73, 0x00
                else if (melbus_LastReadByte[3] == 0xC2 && melbus_LastReadByte[2] == 0x1D && melbus_LastReadByte[1] == 0x73 && melbus_LastReadByte[0] == 0x00)
                {
                    ESP_LOGI(MY_TAG, "C2_0");
                    ConnTicks = 0;
                    while (!gpio_get_level(MELBUS_BUSY))
                    {
                        if (byteIsRead)
                        {
                            byteIsRead = false;
                            SendByteToMelbus(0x10);
                            SendByteToMelbus(0x00);
                            SendByteToMelbus(melbus_ReceivedByte);
                            SendByteToMelbus(0x00);
                            SendByteToMelbus(0x00);
                            for (uint8_t b = 0; b < 14; b++)
                            {
                                SendByteToMelbus(0x20);
                            }
                            break;
                        }
                    }
                }

                // C2_1  0xC2, 0x1D, 0x73, 0x01
                else if (melbus_LastReadByte[3] == 0xC2 && melbus_LastReadByte[2] == 0x1D && melbus_LastReadByte[1] == 0x73 && melbus_LastReadByte[0] == 0x01)
                {
                    ESP_LOGI(MY_TAG, "C2_1");
                    ConnTicks = 0;
                    while (!gpio_get_level(MELBUS_BUSY))
                    {
                        if (byteIsRead)
                        {
                            byteIsRead = false;
                            SendByteToMelbus(0x10);
                            SendByteToMelbus(0x01);
                            SendByteToMelbus(melbus_ReceivedByte);
                            SendByteToMelbus(0x00);
                            SendByteToMelbus(0x00);
                            for (uint8_t b = 0; b < 14; b++)
                            {
                                SendByteToMelbus(0x20);
                            }
                            break;
                        }
                    }
                }

                // C3_0  0xC3, 0x1F, 0x7C, 0x00
                else if (melbus_LastReadByte[3] == 0xC3 && melbus_LastReadByte[2] == 0x1F && melbus_LastReadByte[1] == 0x7C && melbus_LastReadByte[0] == 0x00)
                {
                    ESP_LOGI(MY_TAG, "C3_0");
                    ConnTicks = 0;
                    for (uint8_t i = 0; i < SO_C3_Init_0; i++)
                    {
                        SendByteToMelbus(C3_Init_0[i]);
                    }
                }

                // C3_1  0xC3, 0x1F, 0x7C, 0x01
                else if (melbus_LastReadByte[3] == 0xC3 && melbus_LastReadByte[2] == 0x1F && melbus_LastReadByte[1] == 0x7C && melbus_LastReadByte[0] == 0x01)
                {
                    ESP_LOGI(MY_TAG, "C3_1");
                    ConnTicks = 0;
                    for (uint8_t i = 0; i < SO_C3_Init_1; i++)
                    {
                        SendByteToMelbus(C3_Init_1[i]);
                    }
                }

                // C3_2  0xC3, 0x1F, 0x7C, 0x02
                else if (melbus_LastReadByte[3] == 0xC3 && melbus_LastReadByte[2] == 0x1F && melbus_LastReadByte[1] == 0x7C && melbus_LastReadByte[0] == 0x02)
                {
                    ESP_LOGI(MY_TAG, "C3_2");
                    ConnTicks = 0;
                    for (uint8_t i = 0; i < SO_C3_Init_2; i++)
                    {
                        SendByteToMelbus(C3_Init_2[i]);
                    }
                }

                // C5_1  0xC5, 0x19, 0x73
                else if (melbus_LastReadByte[2] == 0xC5 && melbus_LastReadByte[1] == 0x19 && melbus_LastReadByte[0] == 0x73)
                {
                    ESP_LOGI(MY_TAG, "C5_1");
                    ConnTicks = 0;
                    while (!gpio_get_level(MELBUS_BUSY))
                    {
                        if (byteIsRead)
                        {
                            byteIsRead = false;
                            b1 = melbus_ReceivedByte;
                            break;
                        }
                    }
                    while (!gpio_get_level(MELBUS_BUSY))
                    {
                        if (byteIsRead)
                        {
                            byteIsRead = false;
                            b2 = melbus_ReceivedByte;
                            break;
                        }
                    }
                    SendByteToMelbus(0x10);
                    SendByteToMelbus(b1);
                    SendByteToMelbus(b2);
                    for (uint8_t b = 0; b < 36; b++)
                    {
                        SendByteToMelbus(textLine[b2 - 1][b]);
                    }
                }

                // CMD_1  0xC0, 0x1B, 0x76
                else if (melbus_LastReadByte[2] == 0xC0 && melbus_LastReadByte[1] == 0x1B && melbus_LastReadByte[0] == 0x76)
                {
                    ESP_LOGI(MY_TAG, "CMD_1");
                    ConnTicks = 0;
                    while (!gpio_get_level(MELBUS_BUSY))
                    {
                        if (byteIsRead)
                        {
                            byteIsRead = false;
                            cnt++;
                        }
                        if (cnt == 2)
                        {
                            SendByteToMelbus(0x10);
                            break;
                        }
                    }
                    powerOn = true;
                }

                // PUP  0xC0, 0x1C, 0x70, 0x02
                else if (melbus_LastReadByte[3] == 0xC0 && melbus_LastReadByte[2] == 0x1C && melbus_LastReadByte[1] == 0x70 && melbus_LastReadByte[0] == 0x02)
                {
                    ESP_LOGI(MY_TAG, "PUP");
                    ConnTicks = 0;
                    while (!gpio_get_level(MELBUS_BUSY))
                    {
                        if (byteIsRead)
                        {
                            byteIsRead = false;
                            cnt++;
                        }
                        if (cnt == 2)
                        {
                            SendByteToMelbus(0x90);
                            break;
                        }
                    }
                }

                // CMD_3  0xC0, 0x1D, 0x76, 0x80, 0x00
                else if (melbus_LastReadByte[4] == 0xC0 && melbus_LastReadByte[3] == 0x1D && melbus_LastReadByte[2] == 0x76 && melbus_LastReadByte[1] == 0x80 && melbus_LastReadByte[0] == 0x00)
                {
                    ESP_LOGI(MY_TAG, "CMD_3");
                    ConnTicks = 0;
                    SendByteToMelbus(0x10);
                    SendByteToMelbus(0x80);
                    SendByteToMelbus(0x92);
                }

                // PWR_OFF  0xC0, 0x1C, 0x70, 0x00, 0x80, 0x01
                else if (melbus_LastReadByte[5] == 0xC0 && melbus_LastReadByte[4] == 0x1C && melbus_LastReadByte[3] == 0x70 && melbus_LastReadByte[2] == 0x00 && melbus_LastReadByte[1] == 0x80 && melbus_LastReadByte[0] == 0x01)
                {
                    ESP_LOGI(MY_TAG, "PWR_OFF");
                    ConnTicks = 0;
                    SendByteToMelbus(0x00);
                    powerOn = false;
                }

                // PWR_SBY  0xC0, 0x1C, 0x70, 0x01, 0x80, 0x01
                else if (melbus_LastReadByte[5] == 0xC0 && melbus_LastReadByte[4] == 0x1C && melbus_LastReadByte[3] == 0x70 && melbus_LastReadByte[2] == 0x01 && melbus_LastReadByte[1] == 0x80 && melbus_LastReadByte[0] == 0x01)
                {
                    ESP_LOGI(MY_TAG, "PWR_SBY");
                    ConnTicks = 0;
                    SendByteToMelbus(0x00);
                    powerOn = false;
                }

                // IGN_OFF  0x00, 0x18, 0x12
                else if (melbus_LastReadByte[2] == 0x00 && melbus_LastReadByte[1] == 0x18 && melbus_LastReadByte[0] == 0x12)
                {
                    ESP_LOGI(MY_TAG, "IGN_OFF");
                    ConnTicks = 0;
                    powerOn = false;
                }

                // BTN  0xC0, 0x1D, 0x77, 0x81
                else if (melbus_LastReadByte[3] == 0xC0 && melbus_LastReadByte[2] == 0x1D && melbus_LastReadByte[1] == 0x77 && melbus_LastReadByte[0] == 0x81)
                {
                    ESP_LOGI(MY_TAG, "BTN");
                    ConnTicks = 0;
                    while (!gpio_get_level(MELBUS_BUSY))
                    {
                        if (byteIsRead)
                        {
                            byteIsRead = false;
                            b1 = melbus_ReceivedByte;
                            break;
                        }
                    }
                    SendByteToMelbus(255);
                    SendByteToMelbus(255);
                    SendByteToMelbus(255);
                    switch (b1)
                    {
                    // 0x1 to 0x6 corresponds to cd buttons 1 to 6 on the HU (650) (SAT 1)
                    // 7-13 on SAT 2, and 14-20 on SAT 3
                    // button 1 is always sent (once) when switching to SAT1.
                    case 0x1:
                        // toggleOutput(LEDMISC1); //turn on/off one output pin.
                        // Not used since it will be triggered by setting SAT1
                        break;
                    case 0x2:
                        a2dp_sink.next(); // unfortunately the steering wheel button equals btn #2
                        //esp_rom_delay_us(10);
                        vTaskDelay(1/ portTICK_PERIOD_MS);
                        break;
                    case 0x3:
                        break;
                    case 0x4:
                        break;
                    case 0x5:
                        break;
                    case 0x6: // unfortunately the steering wheel button equals btn #6
                        a2dp_sink.previous();
                        //esp_rom_delay_us(10);
                        vTaskDelay(1/ portTICK_PERIOD_MS);
                        break;
                    }
                }

                // NXT  0xC0, 0x1B, 0x71, 0x80, 0x00
                else if (melbus_LastReadByte[4] == 0xC0 && melbus_LastReadByte[3] == 0x1B && melbus_LastReadByte[2] == 0x71 && melbus_LastReadByte[1] == 0x80 && melbus_LastReadByte[0] == 0x00)
                {
                    ESP_LOGI(MY_TAG, "NXT");
                    ConnTicks = 0;
                    SendByteToMelbus(0x00);
                    a2dp_sink.next();
                    //esp_rom_delay_us(10);
                    vTaskDelay(1/ portTICK_PERIOD_MS);
                }

                // PRV  0xC0, 0x1B, 0x71, 0x00, 0x00
                else if (melbus_LastReadByte[4] == 0xC0 && melbus_LastReadByte[3] == 0x1B && melbus_LastReadByte[2] == 0x71 && melbus_LastReadByte[1] == 0x00 && melbus_LastReadByte[0] == 0x00)
                {
                    ESP_LOGI(MY_TAG, "PRV");
                    ConnTicks = 0;
                    SendByteToMelbus(0x00);
                    a2dp_sink.previous();
                    //esp_rom_delay_us(10);
                    vTaskDelay(1/ portTICK_PERIOD_MS);
                }

                // SCN  0xC0, 0x1A, 0x74, 0x2A
                else if (melbus_LastReadByte[3] == 0xC0 && melbus_LastReadByte[2] == 0x1A && melbus_LastReadByte[1] == 0x74 && melbus_LastReadByte[0] == 0x2A)
                {
                    ESP_LOGI(MY_TAG, "SCN");
                    ConnTicks = 0;
                    SendByteToMelbus(0x00);
                    a2dp_sink.play();
                    //esp_rom_delay_us(10);
                    vTaskDelay(1/ portTICK_PERIOD_MS);
                }

                // CDC_CIR  0xE8, 0x1E, 0xEF
                else if (melbus_LastReadByte[2] == 0xE8 && melbus_LastReadByte[1] == 0x1E && melbus_LastReadByte[0] == 0xEF)
                {
                    ESP_LOGI(MY_TAG, "CDC_CIR");
                    ConnTicks = 0;
                    SendCartridgeInfo();
                }

                // CDC_TIR  E9, 0x1B, 0xE0, 0x01, 0x08
                else if (melbus_LastReadByte[4] == 0xE9 && melbus_LastReadByte[3] == 0x1B && melbus_LastReadByte[2] == 0xE0 && melbus_LastReadByte[1] == 0x01 && melbus_LastReadByte[0] == 0x08)
                {
                    ESP_LOGI(MY_TAG, "CDC_TIR");
                    ConnTicks = 0;
                    SendTrackInfo();
                }

                // CDC_NXT  0xE8, 0x1B, 0x2D, 0x40, 0x01
                else if (melbus_LastReadByte[4] == 0xE8 && melbus_LastReadByte[3] == 0x1B && melbus_LastReadByte[2] == 0x2D && melbus_LastReadByte[1] == 0x40 && melbus_LastReadByte[0] == 0x01)
                {
                    ESP_LOGI(MY_TAG, "CDC_NXT");
                    ConnTicks = 0;
                    track++;
                    fixTrack();
                    trackInfo[5] = track;
                    a2dp_sink.next();
                    //esp_rom_delay_us(10);
                    vTaskDelay(1/ portTICK_PERIOD_MS);
                }

                // CDC_PRV  0xE8, 0x1B, 0x2D, 0x00, 0x01
                else if (melbus_LastReadByte[4] == 0xE8 && melbus_LastReadByte[3] == 0x1B && melbus_LastReadByte[2] == 0x2D && melbus_LastReadByte[1] == 0x00 && melbus_LastReadByte[0] == 0x01)
                {
                    ESP_LOGI(MY_TAG, "CDC_PRV");
                    ConnTicks = 0;
                    track--;
                    fixTrack();
                    trackInfo[5] = track;
                    a2dp_sink.previous();
                    //esp_rom_delay_us(10);
                    vTaskDelay(1/ portTICK_PERIOD_MS);
                }

                // CDC_CHG  0xE8, 0x1A, 0x50
                else if (melbus_LastReadByte[2] == 0xE8 && melbus_LastReadByte[1] == 0x1A && melbus_LastReadByte[0] == 0x50)
                {
                    ESP_LOGI(MY_TAG, "CDC_CHG");
                    ConnTicks = 0;
                    changeCD();
                }

                // CDC_PUP  0xE8, 0x19, 0x2F
                else if (melbus_LastReadByte[2] == 0xE8 && melbus_LastReadByte[1] == 0x19 && melbus_LastReadByte[0] == 0x2F)
                {
                    ESP_LOGI(MY_TAG, "CDC_PUP");
                    ConnTicks = 0;
                    SendByteToMelbus(0x00);
                    trackInfo[1] = startByte;
                    trackInfo[8] = startByte;
                }

                // CDC_PDN  0xE8, 0x19, 0x22
                else if (melbus_LastReadByte[2] == 0xE8 && melbus_LastReadByte[1] == 0x19 && melbus_LastReadByte[0] == 0x22)
                {
                    ESP_LOGI(MY_TAG, "CDC_PDN");
                    ConnTicks = 0;
                    SendByteToMelbus(0x00);
                    trackInfo[1] = stopByte;
                    trackInfo[8] = stopByte;
                }

                // CDC_FFW  0xE8, 0x19, 0x29
                else if (melbus_LastReadByte[2] == 0xE8 && melbus_LastReadByte[1] == 0x19 && melbus_LastReadByte[0] == 0x29)
                {
                    ESP_LOGI(MY_TAG, "CDC_FFW");
                    ConnTicks = 0;
                    SendByteToMelbus(0x00);
                }

                // CDC_FRW  0xE8, 0x19, 0x26
                else if (melbus_LastReadByte[2] == 0xE8 && melbus_LastReadByte[1] == 0x19 && melbus_LastReadByte[0] == 0x26)
                {
                    ESP_LOGI(MY_TAG, "CDC_FRW");
                    ConnTicks = 0;
                    SendByteToMelbus(0x00);
                }

                // CDC_SCN  0xE8, 0x19, 0x2E
                else if (melbus_LastReadByte[2] == 0xE8 && melbus_LastReadByte[1] == 0x19 && melbus_LastReadByte[0] == 0x2E)
                {

                    ESP_LOGI(MY_TAG, "CDC_SCN");
                    ConnTicks = 0;
                    SendByteToMelbus(0x00);
                }

                // CDC_RND  0xE8, 0x19, 0x52
                else if (melbus_LastReadByte[2] == 0xE8 && melbus_LastReadByte[1] == 0x19 && melbus_LastReadByte[0] == 0x52)
                {
                    ESP_LOGI(MY_TAG, "CDC_RND");
                    ConnTicks = 0;
                    SendByteToMelbus(0x00);
                    a2dp_sink.play();
                    //esp_rom_delay_us(10);
                    vTaskDelay(1/ portTICK_PERIOD_MS);
                }

                // CDC_NU  0xE8, 0x1A, 0x50
                else if (melbus_LastReadByte[2] == 0xE8 && melbus_LastReadByte[1] == 0x1A && melbus_LastReadByte[0] == 0x50)
                {
                    ESP_LOGI(MY_TAG, "CDC_NU");
                    ConnTicks = 0;
                }
            } // byteIsRead
            BUSY = gpio_get_level(MELBUS_BUSY);
        } // !busy
        vTaskDelay(1/ portTICK_PERIOD_MS);
        

        // runOnce is counting down to zero and stays there
        // after that, runPeriodically is counting down over and over...
        if (runOnce >= 1)
        {
            runOnce--;
        }
        else if (runPeriodically > 0)
            runPeriodically--;

        // check if BUSY-line is alive
        if (HWTicks > timeout)
        {
            ESP_LOGI(MY_TAG, "BUSY line problem");
            // print_byte_array(melbus_log, 99);
            HWTicks = 0;
            ComTicks = 0;  // to avoid several init requests at once
            ConnTicks = 0; // to avoid several init requests at once
            //  while (1);      //maybe do a reset here, after a delay.
            melbus_Init_CDCHRG(); // worth a try...
        }

        // check if we are receiving any data
        if (ComTicks > timeout)
        {
            ESP_LOGI(MY_TAG, "COM failure (check CLK line)");
            ComTicks = 0;
            ConnTicks = 0;        // to avoid several init requests at once
            melbus_Init_CDCHRG(); // what else to do...
        }

        // check if HU is talking to us specifically, otherwise force it.
        if (ConnTicks > timeout)
        {
            ESP_LOGI(MY_TAG, "Lost connection. Re-initializing");
            ConnTicks = 0;
            melbus_Init_CDCHRG();
        }

        // Reset stuff
        melbus_Bitposition = 7;

        // initiate MRB2 to send text (from one of the four textlines in customText[textRow][])
        if ((runOnce == 1) || reqMasterFlag)
        {
            esp_rom_delay_us(200);
            reqMaster();
            reqMasterFlag = false;
        }
    }
}

// Task of launching Bluetooth
void bluetoothTask(void *pvParameters) {
    
    a2dp_sink.start("SwedenMetal");

    while (true) {
        vTaskDelay(5/ portTICK_PERIOD_MS); // Delay so as not to overload the processor
    }
}

// Task of launching emulator CD-CHG
void mainTask(void *pvParameters) {
    loop();
}


// Main program
extern "C" void app_main(void)
{   
    // Create task of launching emulator CD-CHG on core 1
    xTaskCreatePinnedToCore(
        mainTask,  // Task function
        "MainTask", // Name function
        8192, // Stack size for the main program
        NULL,  // Parametrs of task
        10,    // Priority of the main task
        NULL, // Task handling
        1     // Running on core 0
    );

    // Create task of launching Bluetooth on core 0, it doesn't seem to have a special effect
    // xTaskCreatePinnedToCore(
    //     bluetoothTask,  // Task function
    //     "BluetoothTask", // Name function
    //     16384, // Stack size for the main program
    //     NULL,  // Parametrs of task
    //     1,    // Priority of the main task
    //     NULL, // Task handling
    //     0     // Running on core 0
    // );

    // comment this if task of launching bluetooth is enabled
    a2dp_sink.start("SwedenMetal");
    while(true){
        vTaskDelay(100/ portTICK_PERIOD_MS);
    }

    //comment this if task of launching CD-CHG is enabled
    //loop();
     
}


