// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

#include "driver/spi_master.h"
#include "soc/spi_reg.h"
#include "driver/hspi.h"
#include "soc/gpio_reg.h"
#include "esp_attr.h"

#include "soc/gpio_struct.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "camera.h"

#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/api.h"
#include "bitmap.h"

//#include "ov7670_ext.h"

/*
 This code displays some fancy graphics on the ILI9341-based 320x240 LCD on an ESP-WROVER_KIT board.
 It is not very fast, even when the SPI transfer itself happens at 8MHz and with DMA, because
 the rest of the code is not very optimized. Especially calculating the image line-by-line
 is inefficient; it would be quicker to send an entire screenful at once. This example does, however,
 demonstrate the use of both spi_device_transmit as well as spi_device_queue_trans/spi_device_get_trans_result
 as well as pre-transmit callbacks.

 Some info about the ILI9341: It has an C/D line, which is connected to a GPIO here. It expects this
 line to be low for a command and high for data. We use a pre-transmit callback here to control that
 line: every transaction has as the user-definable argument the needed state of the D/C line and just
 before the transaction is sent, the callback will set this line to the correct state.
*/

#define PIN_NUM_MISO 12
#define PIN_NUM_MOSI 13
#define PIN_NUM_CLK  14
#define PIN_NUM_CS   15

#define PIN_NUM_DC   16
#define PIN_NUM_RST  33
#define PIN_NUM_BCKL 17


/*
 The ILI9341 needs a bunch of command/argument values to be initialized. They are stored in this struct.
*/
typedef struct {
    uint8_t cmd;
    uint8_t data[16];
    uint8_t databytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} ili_init_cmd_t;

static spi_device_handle_t spi;

//Place data into DRAM. Constant data gets placed into DROM by default, which is not accessible by DMA.
DRAM_ATTR static const ili_init_cmd_t ili_init_cmds[]={
    {0xCF, {0x00, 0x83, 0X30}, 3},
    {0xED, {0x64, 0x03, 0X12, 0X81}, 4},
    {0xE8, {0x85, 0x01, 0x79}, 3},
    {0xCB, {0x39, 0x2C, 0x00, 0x34, 0x02}, 5},
    {0xF7, {0x20}, 1},
    {0xEA, {0x00, 0x00}, 2},
    {0xC0, {0x26}, 1},
    {0xC1, {0x11}, 1},
    {0xC5, {0x35, 0x3E}, 2},
    {0xC7, {0xBE}, 1},
    {0x36, {0x28}, 1},
    {0x3A, {0x55}, 1},
    {0xB1, {0x00, 0x1B}, 2},
    {0xF2, {0x08}, 1},
    {0x26, {0x01}, 1},
    {0xE0, {0x1F, 0x1A, 0x18, 0x0A, 0x0F, 0x06, 0x45, 0X87, 0x32, 0x0A, 0x07, 0x02, 0x07, 0x05, 0x00}, 15},
    {0XE1, {0x00, 0x25, 0x27, 0x05, 0x10, 0x09, 0x3A, 0x78, 0x4D, 0x05, 0x18, 0x0D, 0x38, 0x3A, 0x1F}, 15},
    {0x2A, {0x00, 0x00, 0x00, 0xEF}, 4},
    {0x2B, {0x00, 0x00, 0x01, 0x3f}, 4},
    {0x2C, {0}, 0},
    {0xB7, {0x07}, 1},
    {0xB6, {0x0A, 0x82, 0x27, 0x00}, 4},
    {0x11, {0}, 0x80},
    {0x29, {0}, 0x80},
    {0, {0}, 0xff},
};

//Send a command to the ILI9341. Uses spi_device_transmit, which waits until the transfer is complete.
void ili_cmd(spi_device_handle_t spi, const uint8_t cmd)
{
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=8;                     //Command is 8 bits
    t.tx_buffer=&cmd;               //The data is the cmd itself
    t.user=(void*)0;                //D/C needs to be set to 0
    ret=spi_device_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
}

//Send data to the ILI9341. Uses spi_device_transmit, which waits until the transfer is complete.
void ili_data(spi_device_handle_t spi, const uint8_t *data, int len)
{
    esp_err_t ret;
    spi_transaction_t t;
    if (len==0) return;             //no need to send anything
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=len*8;                 //Len is in bytes, transaction length is in bits.
    t.tx_buffer=data;               //Data
    t.user=(void*)1;                //D/C needs to be set to 1
    ret=spi_device_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
}

//This function is called (in irq context!) just before a transmission starts. It will
//set the D/C line to the value indicated in the user field.
void ili_spi_pre_transfer_callback(spi_transaction_t *t)
{
    int dc=(int)t->user;
    gpio_set_level(PIN_NUM_DC, dc);
}

//Initialize the display
void ili_init(spi_device_handle_t spi)
{
    int cmd=0;
    //Initialize non-SPI GPIOs
    gpio_set_direction(PIN_NUM_DC, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_RST, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_BCKL, GPIO_MODE_OUTPUT);
    //Reset the display
    gpio_set_level(PIN_NUM_RST, 0);
    vTaskDelay(100 / portTICK_RATE_MS);
    gpio_set_level(PIN_NUM_RST, 1);
    vTaskDelay(100 / portTICK_RATE_MS);
    //Send all the commands
    while (ili_init_cmds[cmd].databytes!=0xff) {
        ili_cmd(spi, ili_init_cmds[cmd].cmd);
        ili_data(spi, ili_init_cmds[cmd].data, ili_init_cmds[cmd].databytes&0x1F);
        if (ili_init_cmds[cmd].databytes&0x80) {
            vTaskDelay(100 / portTICK_RATE_MS);
        }
        cmd++;
    }
    ///Enable backlight
    gpio_set_level(PIN_NUM_BCKL, 1);
}


//To send a line we have to send a command, 2 data bytes, another command, 2 more data bytes and another command
//before sending the line data itself; a total of 6 transactions. (We can't put all of this in just one transaction
//because the D/C line needs to be toggled in the middle.)
//This routine queues these commands up so they get sent as quickly as possible.
static void send_line(spi_device_handle_t spi, int ypos, uint16_t *line)
{
    esp_err_t ret;
    int x;
    //Transaction descriptors. Declared static so they're not allocated on the stack; we need this memory even when this
    //function is finished because the SPI driver needs access to it even while we're already calculating the next line.
    static spi_transaction_t trans[6];

    //In theory, it's better to initialize trans and data only once and hang on to the initialized
    //variables. We allocate them on the stack, so we need to re-init them each call.
    for (x=0; x<6; x++) {
        memset(&trans[x], 0, sizeof(spi_transaction_t));
        if ((x&1)==0) {
            //Even transfers are commands
            trans[x].length=8;
            trans[x].user=(void*)0;
        } else {
            //Odd transfers are data
            trans[x].length=8*4;
            trans[x].user=(void*)1;
        }
        trans[x].flags=SPI_TRANS_USE_TXDATA;
    }
    trans[0].tx_data[0]=0x2A;           //Column Address Set
    trans[1].tx_data[0]=0;              //Start Col High
    trans[1].tx_data[1]=0;              //Start Col Low
    trans[1].tx_data[2]=(320)>>8;       //End Col High
    trans[1].tx_data[3]=(320)&0xff;     //End Col Low
    trans[2].tx_data[0]=0x2B;           //Page address set
    trans[3].tx_data[0]=ypos>>8;        //Start page high
    trans[3].tx_data[1]=ypos&0xff;      //start page low
    trans[3].tx_data[2]=(ypos+1)>>8;    //end page high
    trans[3].tx_data[3]=(ypos+1)&0xff;  //end page low
    trans[4].tx_data[0]=0x2C;           //memory write
    trans[5].tx_buffer=line;            //finally send the line data
    trans[5].length=320*2*8;            //Data length, in bits
    trans[5].flags=0; //undo SPI_TRANS_USE_TXDATA flag

    //Queue all transactions.
    for (x=0; x<6; x++) {
        ret=spi_device_queue_trans(spi, &trans[x], portMAX_DELAY);
        assert(ret==ESP_OK);
    }

    //When we are here, the SPI driver is busy (in the background) getting the transactions sent. That happens
    //mostly using DMA, so the CPU doesn't have much to do here. We're not going to wait for the transaction to
    //finish because we may as well spend the time calculating the next line. When that is done, we can call
    //send_line_finish, which will wait for the transfers to be done and check their status.
}


static void send_line_finish(spi_device_handle_t spi)
{
    spi_transaction_t *rtrans;
    esp_err_t ret;
    //Wait for all 6 transactions to be done and get back the results.
    for (int x=0; x<6; x++) {
        ret=spi_device_get_trans_result(spi, &rtrans, portMAX_DELAY);
        assert(ret==ESP_OK);
        //We could inspect rtrans now if we received any info back. The LCD is treated as write-only, though.
    }
}


//Simple routine to generate some patterns and send them to the LCD. Don't expect anything too
//impressive. Because the SPI driver handles transactions in the background, we can calculate the next line
//while the previous one is being sent.
/*
static void display_pretty_colors(spi_device_handle_t spi)
{
    uint16_t line[2][320];
    int x, y, frame=0;
    //Indexes of the line currently being sent to the LCD and the line we're calculating.
    int sending_line=-1;
    int calc_line=0;
    uint8_t* fb_ref = camera_get_fb();
    int width = camera_get_fb_width();
    int height =  camera_get_fb_height();
    while(1) {
        frame++;
        for (y=0; y<height; y++) {
            //Calculate a line.
            for (x=0; x<width; x++) {
                //line[calc_line][x]=((x<<3)^(y<<3)^(frame+x*y));

                line[calc_line][x]=fb_ref[(y*width)+x]+0;

            }
            //Finish up the sending process of the previous line, if any
            if (sending_line!=-1) send_line_finish(spi);
            //Swap sending_line and calc_line
            sending_line=calc_line;
            calc_line=(calc_line==1)?0:1;
            //Send the line we currently calculated.
            send_line(spi, y, line[sending_line]);
            //The line is queued up for sending now; the actual sending happens in the
            //background. We can go on to calculate the next line as long as we do not
            //touch line[sending_line]; the SPI sending process is still reading from that.
        }
    }
}
*/
/*
//Simple routine to generate some patterns and send them to the LCD. Don't expect anything too
//impressive. Because the SPI driver handles transactions in the background, we can calculate the next line
//while the previous one is being sent.
static void display_pretty_colors(spi_device_handle_t spi)
{
    uint16_t line[2][320];
    int x, y, frame=0;
    //Indexes of the line currently being sent to the LCD and the line we're calculating.
    int sending_line=-1;
    int calc_line=0;

    while(1) {
        frame++;
        for (y=0; y<240; y++) {
            //Calculate a line.
            for (x=0; x<320; x++) {
                line[calc_line][x]=((x<<3)^(y<<3)^(frame+x*y));
            }
            //Finish up the sending process of the previous line, if any
            if (sending_line!=-1) send_line_finish(spi);
            //Swap sending_line and calc_line
            sending_line=calc_line;
            calc_line=(calc_line==1)?0:1;
            //Send the line we currently calculated.
            send_line(spi, y, line[sending_line]);
            //The line is queued up for sending now; the actual sending happens in the
            //background. We can go on to calculate the next line as long as we do not
            //touch line[sending_line]; the SPI sending process is still reading from that.
        }
    }
}

*/

// camera code

static const char* TAG = "camera_demo";

int state = 0;
const static char http_hdr[] = "HTTP/1.1 200 OK\r\n";
const static char http_stream_hdr[] =
        "Content-type: multipart/x-mixed-replace; boundary=123456789000000000000987654321\r\n\r\n";
const static char http_jpg_hdr[] =
        "Content-type: image/jpg\r\n\r\n";
const static char http_pgm_hdr[] =
        "Content-type: image/x-portable-graymap\r\n\r\n";
const static char http_stream_boundary[] = "--123456789000000000000987654321\r\n";
const static char http_bitmap_hdr[] =
        "Content-type: image/bitmap\r\n\r\n";
const static char http_yuv422_hdr[] =
        "Content-Disposition: attachment; Content-type: application/octet-stream\r\n\r\n";


static EventGroupHandle_t wifi_event_group;
const int CONNECTED_BIT = BIT0;
static ip4_addr_t s_ip_addr;
static camera_pixelformat_t s_pixel_format;
//static camera_config_t config
static camera_config_t config = {
    .ledc_channel = LEDC_CHANNEL_0,
    .ledc_timer = LEDC_TIMER_0,
    .pin_d0 = CONFIG_D0,
    .pin_d1 = CONFIG_D1,
    .pin_d2 = CONFIG_D2,
    .pin_d3 = CONFIG_D3,
    .pin_d4 = CONFIG_D4,
    .pin_d5 = CONFIG_D5,
    .pin_d6 = CONFIG_D6,
    .pin_d7 = CONFIG_D7,
    .pin_xclk = CONFIG_XCLK,
    .pin_pclk = CONFIG_PCLK,
    .pin_vsync = CONFIG_VSYNC,
    .pin_href = CONFIG_HREF,
    .pin_sscb_sda = CONFIG_SDA,
    .pin_sscb_scl = CONFIG_SCL,
    .pin_reset = CONFIG_RESET,
    .xclk_freq_hz = 12000000,
    //.xclk_freq_hz = 12000000, //20000000, // 20 works well across all with HS sampling ?
    // above works well with freescale etc...
    //CONFIG_XCLK_FREQ,
    //.xclk_freq_hz = 20000000, //8000000, //16000000, //8000000,
};

static camera_model_t camera_model;

// CHECK OV760 PIXFORMAT HACK!!!
#define CAMERA_PIXEL_FORMAT CAMERA_PF_RGB565
//#define CAMERA_PIXEL_FORMAT CAMERA_PF_GRAYSCALE
//#define CAMERA_PIXEL_FORMAT CAMERA_PF_YUV422
//#define CAMERA_FRAME_SIZE CAMERA_FS_QQVGA
#define CAMERA_FRAME_SIZE CAMERA_FS_QVGA

static bool test_pattern_enabled = true;
static int tft_offset = 0;



static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch (event->event_id) {
        case SYSTEM_EVENT_STA_START:
            esp_wifi_connect();
            break;
        case SYSTEM_EVENT_STA_GOT_IP:
            xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
            s_ip_addr = event->event_info.got_ip.ip_info.ip;
            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            /* This is a workaround as ESP32 WiFi libs don't currently
             auto-reassociate. */
            esp_wifi_connect();
            xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
            break;
        default:
            break;
    }
    return ESP_OK;
}

static void initialise_wifi(void)
{
    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_WIFI_SSID,
            .password = CONFIG_WIFI_PASSWORD,
        },
    };
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );
    ESP_ERROR_CHECK( esp_wifi_set_ps(WIFI_PS_NONE) );
    ESP_LOGI(TAG, "Connecting to \"%s\"", wifi_config.sta.ssid);
    xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);
    ESP_LOGI(TAG, "Connected");
}

uint8_t getHexVal(char c)
{
   if(c >= '0' && c <= '9')
     return (uint8_t)(c - '0');
   else
     return (uint8_t)(c-'A'+10);
}

static void push_framebuffer_to_tft() {
  uint16_t line[2][320];
  int x, y, frame=0;
  //Indexes of the line currently being sent to the LCD and the line we're calculating.
  int sending_line=-1;
  int calc_line=0;

  uint8_t* fb = camera_get_fb();
// handle reset of re-init to change pxclk
  if (fb == NULL) return;

  // access the array as 16-bit chunks of raw rgb565 hopefully
  uint16_t* fb_ref = (uint16_t*)&fb;
  int ili_width = 320;
  int ili_height = 240;

  int width = camera_get_fb_width();
  int height =  camera_get_fb_height();
  int max_fb_pos = width * height;
  uint16_t pixel565 = 0;
  while(1) {
     frame++;
     for (y=0; y<ili_height; y++) {
        //Calculate a line.
        for (x=0; x<ili_width; x++) {
            // wrap pixels around...
            int current_fb_pos = ((y*width)+x+tft_offset) % max_fb_pos;
            //line[calc_line][x]=((x<<3)^(y<<3)^(frame+x*y));
            pixel565 = 0xAABC; //
            /*
            if (y < height) {
              if (x < width)
                if (camera_get_fb() != NULL)
                 pixel565 = fb_ref[(y*width)+x];
            }
            */
            // use new offset logic for 565 ....
            if (camera_get_fb() != NULL)
              //if (current_fb_pos < max_fb_pos)
                pixel565 = fb_ref[current_fb_pos];


            line[calc_line][x]= pixel565;
        }
        //Finish up the sending process of the previous line, if any
        if (sending_line!=-1) send_line_finish(spi);
        //Swap sending_line and calc_line
        sending_line=calc_line;
        calc_line=(calc_line==1)?0:1;
        //Send the line we currently calculated.
        send_line(spi, y, line[sending_line]);
        //The line is queued up for sending now; the actual sending happens in the
        //background. We can go on to calculate the next line as long as we do not
        //touch line[sending_line]; the SPI sending process is still reading from that.
      }
      vTaskDelay(30 / portTICK_RATE_MS);
 }

}


static void http_server_netconn_serve(struct netconn *conn)
{
    struct netbuf *inbuf;
    char *buf;
    u16_t buflen;
    err_t err;

    /* Read the data from the port, blocking if nothing yet there.
     We assume the request (the part we care about) is in one netbuf */
    err = netconn_recv(conn, &inbuf);

    if (err == ERR_OK) {
        netbuf_data(inbuf, (void**) &buf, &buflen);

        /* Is this an HTTP GET command? (only check the first 5 chars, since
         there are other formats for GET, and we're keeping it very simple )*/
        if (buflen >= 5 && buf[0] == 'G' && buf[1] == 'E' && buf[2] == 'T'
                && buf[3] == ' ' && buf[4] == '/') {
          /* Send the HTTP header
             * subtract 1 from the size, since we dont send the \0 in the string
             * NETCONN_NOCOPY: our data is const static, so no need to copy it
             */
          netconn_write(conn, http_hdr, sizeof(http_hdr) - 1,
                    NETCONN_NOCOPY);
            //check if a command has been requested.
          if (buf[5] == 'z') {
            ESP_LOGD(TAG, "Command mode.");

            bool reinit_reqd = true;
            if ((buf[6] == 'f')) {

              ESP_LOGD(TAG, "Setting OV7670 extra reg.");
              sensor_t* s_sensor = get_cam_sensor();
              if (s_sensor != NULL)
                //setup_ov7670_tft(s_sensor);
                /*
                err = camera_run();
              if (err != ESP_OK) {
                  ESP_LOGD(TAG, "Camera capture failed with error = %d", err);
              } else {
                  ESP_LOGD(TAG, "Done, rendering to TFT...");
                  //push_framebuffer_to_tft();
                  //char outp[] = "Pushed image to FB.\r\n\r\n";
                  //netconn_write(conn, outp, sizeof(outp) - 1,
                  //          NETCONN_NOCOPY);
                  // just let it be..
                  reinit_reqd = false;
              }
              */
              reinit_reqd = false;
            }
            if ((buf[6] == 'X')) {
              sensor_t* s_sensor = get_cam_sensor();
              if (s_sensor != NULL)
                //setup_ov7670_tft(s_sensor);
                // skip out of there...!!
                //return;
                //transformFramebuffer888to565();
                reinit_reqd = false;
            }
            if ((buf[6] == 'I')) {

                //push_framebuffer_to_tft();
                // skip out of there...!!
                //return;
                sensor_t* s_sensor = get_cam_sensor();
                if (s_sensor != NULL)
                //Ov7670(s_sensor);
                //init();

                reinit_reqd = false;
            }
            if ((buf[6] == 'O') && (buf[7] == '=')) {

                int n1 = buf[8] - '0';
                int n2 = buf[8] - '0';
                int n3 = buf[8] - '0';
                tft_offset = (n1 * 100)+(n2*10)+(n3*1);

              //offset = level
              //sensor_t* s_sensor = get_cam_sensor();
              //if (s_sensor != NULL)

              //Ov7670(s_sensor);

              //gamma_set();
              //AGC_set();
              //magic_set();

                // skip out of there...!!
                //return;
                reinit_reqd = false;
            }
            if ((buf[6] == 'T') && (buf[7] == '=')) {
                //int level = buf[8] - '0';


                sensor_t* s_sensor = get_cam_sensor();

                bool s_yuv_test_mode = false;
                bool s_yuv_reverse_bytes = false;
                bool s_raw_bytes_only = false;
                bool s_gbr_rgb_order = false;
                int s_highspeed_sampling_mode = 0;
                bool s_test_tft_filter = false;

                if (buf[8]=='Y') s_yuv_test_mode = true;
                if (buf[9]=='Y') s_yuv_reverse_bytes = true;
                if (buf[10]=='Y') s_raw_bytes_only = true;
                if (buf[11]=='Y') s_gbr_rgb_order = true;

                if (buf[12] == '1') s_highspeed_sampling_mode = 1;
                else if (buf[12] == '2') s_highspeed_sampling_mode = 2;
                if (buf[13] == 'Y') s_test_tft_filter = true;


                ESP_LOGD(TAG, "Set Test Bytes: %d,%d,%d,%d,%d,%d",s_yuv_test_mode,  s_yuv_reverse_bytes,
                                    s_raw_bytes_only,  s_gbr_rgb_order,  s_highspeed_sampling_mode, s_test_tft_filter);

                if (s_sensor != NULL) {

                   set_test_modes( s_yuv_test_mode,  s_yuv_reverse_bytes,
                                       s_raw_bytes_only,  s_gbr_rgb_order,  s_highspeed_sampling_mode, s_test_tft_filter);


                }
                reinit_reqd = true;
            }
            if ((buf[6] == 'S') && (buf[7] == '=')) {

               int level = buf[8] - '0';
                ESP_LOGD(TAG, "Set framesize: %x",level);
                if (level == 0 ) config.frame_size = FRAMESIZE_QCIF;
                if (level == 1 ) config.frame_size = FRAMESIZE_QVGA;
                if (level == 2 ) config.frame_size = FRAMESIZE_QQVGA;
                reinit_reqd = true;
            }
            if ((buf[6] == 'R') && (buf[7] == '=')) {


                //int level = buf[8] - '0';
                uint8_t reg = getHexVal(buf[9]) + (getHexVal(buf[8]) << 4);
                uint8_t regVal = getHexVal(buf[11]) + (getHexVal(buf[10]) << 4);

                ESP_LOGD(TAG, "Set Register: %x=%x",reg,regVal);
                sensor_t* s_sensor = get_cam_sensor();
                if (s_sensor != NULL) {
                  //s_sensor->sensor_write_reg(reg,regVal); //set_register(s_sensor, reg, regVal);
                  cam_set_sensor_reg( reg, regVal);
                }
                reinit_reqd = false;
            }
            if ((buf[6] == 'h') && (buf[7] == '=')) {
                int level = buf[8] - '0';
                ESP_LOGD(TAG, "Set hflip: %d",level);
                sensor_t* s_sensor = get_cam_sensor();
                if (s_sensor != NULL) {
                  s_sensor->set_vflip(s_sensor, level);
                }
                reinit_reqd = false;
            }
            if ((buf[6] == 'v') && (buf[7] == '=')) {
                int level = buf[8] - '0';
                ESP_LOGD(TAG, "Set vflip: %d",level);
                sensor_t* s_sensor = get_cam_sensor();
                if (s_sensor != NULL) {
                  //s_sensor->set_saturation(s_sensor, level);
                  s_sensor->set_hmirror(s_sensor,level);
                }
                reinit_reqd = false;
            }
            if ((buf[6] == 's') && (buf[7] == '=')) {
                int level = buf[8] - '0';
                ESP_LOGD(TAG, "Set Saturation: %d",level);
                sensor_t* s_sensor = get_cam_sensor();
                if (s_sensor != NULL) {
                  s_sensor->set_saturation(s_sensor, level);
                }
                reinit_reqd = false;
            }
            if ((buf[6] == 'c') && (buf[7] == '=')) {
                int level = buf[8] - '0';
                ESP_LOGD(TAG, "Set Contrast (1-8): %d",level);
                sensor_t* s_sensor = get_cam_sensor();
                if (s_sensor != NULL) {
                  s_sensor->set_contrast(s_sensor, level);
                }
                reinit_reqd = false;
            }
            if ((buf[6] == 'b') && (buf[7] == '=')) {
                int level = buf[8] - '0';
                ESP_LOGD(TAG, "Set brightness(1-8): %d",level);
                sensor_t* s_sensor = get_cam_sensor();
                if (s_sensor != NULL) {
                  s_sensor->set_brightness(s_sensor, level);
                }
                reinit_reqd = false;
            }
            if ((buf[6] == 'w') && (buf[7] == '=')) {
                int level = buf[8] - '0';
                ESP_LOGD(TAG, "Set Whitebal 1-0: %d",level);
                sensor_t* s_sensor = get_cam_sensor();
                if (s_sensor != NULL) {
                  s_sensor->set_whitebal(s_sensor, level);
                }
                reinit_reqd = false;
            }
            if ((buf[6] == 'g') && (buf[7] == '=')) {
                int level = buf[8] - '0';
                ESP_LOGD(TAG, "Set gainceiling 1-6: %d",level);
                sensor_t* s_sensor = get_cam_sensor();
                if (s_sensor != NULL) {
                  s_sensor->set_gainceiling(s_sensor, level);
                }
                reinit_reqd = false;
            }
            if ((buf[6] == 'e') && (buf[7] == '=')) {
                int level = buf[8] - '0';
                ESP_LOGD(TAG, "Set exposure_ctrl 1-0: %d",level);
                sensor_t* s_sensor = get_cam_sensor();
                if (s_sensor != NULL) {
                  s_sensor->set_exposure_ctrl(s_sensor, level);
                }
                reinit_reqd = false;
            }
            if ((buf[6] == 'z') && (buf[7] == '=')) {
                int level = buf[8] - '0';
                ESP_LOGD(TAG, "Set gain_ctrl 1-0: %d",level);
                sensor_t* s_sensor = get_cam_sensor();
                if (s_sensor != NULL) {
                  s_sensor->set_gain_ctrl(s_sensor, level);
                }
                reinit_reqd = false;
            }


            if (buf[6] == 'q') {

              ESP_LOGD(TAG, "Set Colorbar (Test Pattern) on / off");
              test_pattern_enabled = !test_pattern_enabled;

              sensor_t* s_sensor = get_cam_sensor();
              if (s_sensor != NULL) {
                s_sensor->set_colorbar(s_sensor, test_pattern_enabled);
              }
              reinit_reqd = false;

            }
            if (buf[6] == 'x') {
              int speed = 0;
              speed = (buf[7] - '0')*10 + (buf[8] - '0');
              ESP_LOGD(TAG, "Switch XCLCK to %dMHZ",speed);
              speed = speed * 1000000;
              config.xclk_freq_hz = speed;
              vTaskDelay(100 / portTICK_RATE_MS);
              reset_xclk(&config);
              vTaskDelay(100 / portTICK_RATE_MS);
              reinit_reqd = true;



            } else if (buf[6] == 'y') {
              ESP_LOGD(TAG, "Switch Pixelformat to YUV422.");
              s_pixel_format = CAMERA_PF_YUV422;
              // switch to YUV
            } else if (buf[6] == 'r') {
              if ((buf[7] == '5') && (buf[8] == '6')) {
                // rgb565
                ESP_LOGD(TAG, "Switch Pixelformat to RGB565.");
                s_pixel_format = CAMERA_PF_RGB565;
              }
              if ((buf[7] == '5') && (buf[8] == '5')) {
                // rgb555
                ESP_LOGD(TAG, "Switch Pixelformat to RGB555.");

                s_pixel_format = CAMERA_PF_RGB555;
              }
              if (buf[7] == '4') {
                // rgb444
                ESP_LOGD(TAG, "Switch Pixelformat to RGB444.");
                s_pixel_format = CAMERA_PF_RGB444;
              }
              // swithc to grayscale
            } else if (buf[6] == 'g') {
              ESP_LOGD(TAG, "Switch Pixelformat to GRAYSCALE.");
              s_pixel_format = CAMERA_PF_GRAYSCALE;
            } else {
              ESP_LOGD(TAG, "Unknown command");
            }

            if (reinit_reqd) {
              ESP_LOGD(TAG, "Reconfiguring camera...");
              esp_err_t err;
              vTaskDelay(100 / portTICK_RATE_MS);
              err = reset_pixformat();
              config.pixel_format = s_pixel_format;
              err = camera_init(&config);
              if (err != ESP_OK) {
                  ESP_LOGE(TAG, "Camera init failed with error 0x%x", err);
                  //return;
              }
              char cb[] = "Reinit Camera...\r\n\r\n";
              err = netconn_write(conn, cb, sizeof(cb) - 1,
                  NETCONN_NOCOPY);
              return;
              vTaskDelay(100 / portTICK_RATE_MS);
            }

            ESP_LOGI(TAG, "Free heap: %u", xPortGetFreeHeapSize());
            ESP_LOGI(TAG, "Camera ready");

           //check if a stream is requested.
          } else if (buf[5] == 's') {
                //Send mjpeg stream header
                err = netconn_write(conn, http_stream_hdr, sizeof(http_stream_hdr) - 1,
                    NETCONN_NOCOPY);
                ESP_LOGD(TAG, "Stream started.");

                //Run while everyhting is ok and connection open.
                while(err == ERR_OK) {
                    ESP_LOGD(TAG, "Capture frame");
                    err = camera_run();
                    if (err != ESP_OK) {
                        ESP_LOGD(TAG, "Camera capture failed with error = %d", err);
                    } else {
                        ESP_LOGD(TAG, "Done");
                        //Send jpeg header
                        if(s_pixel_format == CAMERA_PF_RGB565) {
                            err = netconn_write(conn, http_bitmap_hdr, sizeof(http_bitmap_hdr) - 1,
                                NETCONN_NOCOPY);
                            char *bmp = bmp_create_header(camera_get_fb_width(), camera_get_fb_height());
                            err = netconn_write(conn, bmp, sizeof(bitmap), NETCONN_NOCOPY);
                            free(bmp);
                        }
                        else
                            err = netconn_write(conn, http_jpg_hdr, sizeof(http_jpg_hdr) - 1,
                                NETCONN_NOCOPY);
                        //Send frame
                        err = netconn_write(conn, camera_get_fb(), camera_get_data_size(),
                            NETCONN_NOCOPY);
                        if(err == ERR_OK)
                        {
                            //Send boundary to next jpeg
                            err = netconn_write(conn, http_stream_boundary,
                                    sizeof(http_stream_boundary) -1, NETCONN_NOCOPY);
                        }
                    }
                }
                ESP_LOGD(TAG, "Stream ended.");
            } else {
                if (s_pixel_format == CAMERA_PF_JPEG) {
                    netconn_write(conn, http_jpg_hdr, sizeof(http_jpg_hdr) - 1, NETCONN_NOCOPY);
                } else if (s_pixel_format == CAMERA_PF_GRAYSCALE) {
                    netconn_write(conn, http_pgm_hdr, sizeof(http_pgm_hdr) - 1, NETCONN_NOCOPY);

                    if (memcmp(&buf[5], "pgm", 3) == 0) {
                        char pgm_header[32];
                        snprintf(pgm_header, sizeof(pgm_header), "P5 %d %d %d\n", camera_get_fb_width(), camera_get_fb_height(), 255);
                        netconn_write(conn, pgm_header, strlen(pgm_header), NETCONN_COPY);
                    }
                    else {
                      char outstr[120];
                      get_image_mime_info_str(outstr);
                      netconn_write(conn, outstr, sizeof(outstr) - 1, NETCONN_NOCOPY);
                      //netconn_write(conn, http_yuv422_hdr, sizeof(http_yuv422_hdr) - 1, NETCONN_NOCOPY);
                    }

                } else if (s_pixel_format == CAMERA_PF_RGB565) {

                    netconn_write(conn, http_bitmap_hdr, sizeof(http_bitmap_hdr) - 1, NETCONN_NOCOPY);
                    if (memcmp(&buf[5], "bmp", 3) == 0) {
                        char *bmp = bmp_create_header(camera_get_fb_width(), camera_get_fb_height());
                        err = netconn_write(conn, bmp, sizeof(bitmap), NETCONN_COPY);
                        free(bmp);
                    }
                    else {
                      char outstr[120];
                      get_image_mime_info_str(outstr);
                      netconn_write(conn, outstr, sizeof(outstr) - 1, NETCONN_NOCOPY);
                      //netconn_write(conn, http_yuv422_hdr, sizeof(http_yuv422_hdr) - 1, NETCONN_NOCOPY);
                    }

                } else if (s_pixel_format == CAMERA_PF_YUV422) {
                  if (memcmp(&buf[5], "bmp", 3) == 0) {
                      netconn_write(conn, http_bitmap_hdr, sizeof(http_bitmap_hdr) - 1, NETCONN_NOCOPY);
                      char *bmp = bmp_create_header(camera_get_fb_width(), camera_get_fb_height());
                      err = netconn_write(conn, bmp, sizeof(bitmap), NETCONN_COPY);
                      free(bmp);
                  } else {
                    char outstr[120];
                    get_image_mime_info_str(outstr);
                    netconn_write(conn, outstr, sizeof(outstr) - 1, NETCONN_NOCOPY);
                    //netconn_write(conn, http_yuv422_hdr, sizeof(http_yuv422_hdr) - 1, NETCONN_NOCOPY);
                  }

                } else {
                  char outstr[120];
                  get_image_mime_info_str(outstr);
                  netconn_write(conn, outstr, sizeof(outstr) - 1, NETCONN_NOCOPY);

                }


                ESP_LOGD(TAG, "Image requested.");
                err = camera_run();
                if (err != ESP_OK) {
                    ESP_LOGD(TAG, "Camera capture failed with error = %d", err);
                } else {
                    ESP_LOGD(TAG, "Done");
                    //Send jpeg
                    err = netconn_write(conn, camera_get_fb(), camera_get_data_size(),
                        NETCONN_NOCOPY);

                }
            }
        }
    }
    /* Close the connection (server closes in HTTP) */
    netconn_close(conn);

    /* Delete the buffer (netconn_recv gives us ownership,
     so we have to make sure to deallocate the buffer) */
    netbuf_delete(inbuf);
}

static void http_server(void *pvParameters)
{
    struct netconn *conn, *newconn;
    err_t err;
    conn = netconn_new(NETCONN_TCP);
    netconn_bind(conn, NULL, 80);
    netconn_listen(conn);
    do {
        err = netconn_accept(conn, &newconn);
        if (err == ERR_OK) {
            http_server_netconn_serve(newconn);
            netconn_delete(newconn);
        }
    } while (err == ERR_OK);
    netconn_close(conn);
    netconn_delete(conn);
}



/*
void lcd_debug(void *pvParameters)
{
	tft.begin();
	tft.fillScreen(ILI9341_GREEN);
	//setupUI();
//	vTaskDelete(NULL);
}
*/


void app_main()
{
    esp_log_level_set("wifi", ESP_LOG_WARN);
    esp_log_level_set("gpio", ESP_LOG_WARN);

    nvs_flash_init();

    //gpio_set_direction(PIN_NUM_MOSI, GPIO_MODE_OUTPUT);
    //gpio_set_direction(PIN_NUM_CLK, GPIO_MODE_OUTPUT);

/*
//CLK
gpio_pad_select_gpio(GPIO_NUM_14);
gpio_set_direction(GPIO_NUM_14, GPIO_MODE_INPUT);
PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTMS_U, 1); //gpio_matrix_in(GPIO_NUM_14, HSPICLK_IN_IDX,0);

//MISO
gpio_pad_select_gpio(GPIO_NUM_12);
gpio_set_direction(GPIO_NUM_12, GPIO_MODE_OUTPUT);
PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDI_U, 1); //gpio_matrix_out(GPIO_NUM_12, HSPIQ_OUT_IDX,0,0);

//MOSI
gpio_pad_select_gpio(GPIO_NUM_13);
gpio_set_direction(GPIO_NUM_13, GPIO_MODE_INPUT);
PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTCK_U, 1); //gpio_matrix_in(GPIO_NUM_13, HSPID_IN_IDX,0);

//CSS
gpio_pad_select_gpio(GPIO_NUM_15);
gpio_set_direction(GPIO_NUM_15, GPIO_MODE_INPUT);
PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDO_U, 1); //gpio_matrix_in(GPIO_NUM_15, HSPICS0_IN_IDX,0);
*/
    esp_err_t ret;
    spi_bus_config_t buscfg={
        .miso_io_num=PIN_NUM_MISO,
        .mosi_io_num=PIN_NUM_MOSI,
        .sclk_io_num=PIN_NUM_CLK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1
    };
    spi_device_interface_config_t devcfg={
        .clock_speed_hz=10000000,               //Clock out at 10 MHz
        .mode=0,                                //SPI mode 0
        .spics_io_num=PIN_NUM_CS,               //CS pin
        .queue_size=7,                          //We want to be able to queue 7 transactions at a time
        .pre_cb=ili_spi_pre_transfer_callback,  //Specify pre-transfer callback to handle D/C line
    };
    //Initialize the SPI bus
    ESP_LOGI(TAG, "Call spi_bus_initialize");
    ret=spi_bus_initialize(HSPI_HOST, &buscfg, 1);
    assert(ret==ESP_OK);
    //Attach the LCD to the SPI bus
    ESP_LOGI(TAG, "Call spi_bus_add_device");
    ret=spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
    assert(ret==ESP_OK);
    //Initialize the LCD
    ESP_LOGI(TAG, "Call ili_init");
    ili_init(spi);

//    ESP_LOGI(TAG, "Register SPI device with camera.c");
//    set_tft_spi_device(&spi);

    /*
    camera_config_t config = {
        .ledc_channel = LEDC_CHANNEL_0,
        .ledc_timer = LEDC_TIMER_0,
        .pin_d0 = CONFIG_D0,
        .pin_d1 = CONFIG_D1,
        .pin_d2 = CONFIG_D2,
        .pin_d3 = CONFIG_D3,
        .pin_d4 = CONFIG_D4,
        .pin_d5 = CONFIG_D5,
        .pin_d6 = CONFIG_D6,
        .pin_d7 = CONFIG_D7,
        .pin_xclk = CONFIG_XCLK,
        .pin_pclk = CONFIG_PCLK,
        .pin_vsync = CONFIG_VSYNC,
        .pin_href = CONFIG_HREF,
        .pin_sscb_sda = CONFIG_SDA,
        .pin_sscb_scl = CONFIG_SCL,
        .pin_reset = CONFIG_RESET,
        //.xclk_freq_hz = CONFIG_XCLK_FREQ,
        .xclk_freq_hz = 8000000, //16000000, //8000000,
    };
    */
    //camera_model_t camera_model;
    esp_err_t err = camera_probe(&config, &camera_model);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera probe failed with error 0x%x", err);
        return;
    }
    if (camera_model == CAMERA_OV7725) {
        ESP_LOGI(TAG, "Detected OV7725 camera, using grayscale bitmap format");
        s_pixel_format = CAMERA_PIXEL_FORMAT;
        config.frame_size = CAMERA_FRAME_SIZE;
    } else if (camera_model == CAMERA_OV7670) {
        ESP_LOGI(TAG, "Detected OV7670 camera, using grayscale bitmap format");
        s_pixel_format = CAMERA_PIXEL_FORMAT;
        config.frame_size = CAMERA_FRAME_SIZE;
    } else if (camera_model == CAMERA_OV2640) {
        ESP_LOGI(TAG, "Detected OV2640 camera, using JPEG format");
        s_pixel_format = CAMERA_PF_JPEG;
        config.frame_size = CAMERA_FS_VGA;
        config.jpeg_quality = 15;
    } else {
        ESP_LOGE(TAG, "Camera not supported");
        return;
    }
    config.pixel_format = s_pixel_format;
    err = camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed with error 0x%x", err);
        return;
    }
    initialise_wifi();
    ESP_LOGI(TAG, "Free heap: %u", xPortGetFreeHeapSize());
    ESP_LOGI(TAG, "Camera demo ready");
    if (s_pixel_format == CAMERA_PF_GRAYSCALE) {
        ESP_LOGI(TAG, "open http://" IPSTR "/get for a single grayscale bitmap (no headers)", IP2STR(&s_ip_addr));
        ESP_LOGI(TAG, "open http://" IPSTR "/pgm for a single image/x-portable-graymap image", IP2STR(&s_ip_addr));
    }
    if (s_pixel_format == CAMERA_PF_RGB565) {
        ESP_LOGI(TAG, "open http://" IPSTR "/bmp for single image/bitmap image", IP2STR(&s_ip_addr));
        ESP_LOGI(TAG, "open http://" IPSTR "/stream for multipart/x-mixed-replace stream of bitmaps", IP2STR(&s_ip_addr));
    }
    if (s_pixel_format == CAMERA_PF_JPEG) {
        ESP_LOGI(TAG, "open http://" IPSTR "/jpg for single image/jpg image", IP2STR(&s_ip_addr));
        ESP_LOGI(TAG, "open http://" IPSTR "/stream for multipart/x-mixed-replace stream of JPEGs", IP2STR(&s_ip_addr));
    }
    xTaskCreate(&http_server, "http_server", 4096, NULL, 5, NULL);

    push_framebuffer_to_tft();

    //Go do nice stuff.
    //display_pretty_colors(spi);

}
