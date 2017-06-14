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
#include "freertos/semphr.h"

#include "driver/gpio.h"

#include "esp_err.h"
#include "esp_log.h"
#include "camera.h"

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

#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/api.h"
#include "bitmap.h"

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
    //.xclk_freq_hz = CONFIG_XCLK_FREQ,
    .xclk_freq_hz = 20000000, //8000000, //16000000, //8000000,
};

static camera_model_t camera_model;

//#define CAMERA_PIXEL_FORMAT CAMERA_PF_RGB565
//#define CAMERA_PIXEL_FORMAT CAMERA_PF_GRAYSCALE
#define CAMERA_PIXEL_FORMAT CAMERA_PF_YUV422
#define CAMERA_FRAME_SIZE CAMERA_FS_QQVGA

static bool test_pattern_enabled = false;

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
            if ((buf[6] == 'T') && (buf[7] == '=')) {
                //int level = buf[8] - '0';


                sensor_t* s_sensor = get_cam_sensor();

                bool yuvTest = false;
                bool reverseBytesTest = false;
                if (buf[8]=='Y') yuvTest = true;
                if (buf[9]=='Y') reverseBytesTest = true;

                ESP_LOGD(TAG, "Set Test Bytes: %d,%d",yuvTest,reverseBytesTest);

                if (s_sensor != NULL) {
                  //s_sensor->sensor_write_reg(reg,regVal); //set_register(s_sensor, reg, regVal);
                  set_test_modes( yuvTest, reverseBytesTest);
                }
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
              reset_xclk(&config);

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
              }
              if (buf[7] == '4') {
                // rgb444
                ESP_LOGD(TAG, "Switch Pixelformat to RGB444.");

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
              err = reset_pixformat();
              config.pixel_format = s_pixel_format;
              err = camera_init(&config);
              if (err != ESP_OK) {
                  ESP_LOGE(TAG, "Camera init failed with error 0x%x", err);
                  return;
              }
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
                } else if (s_pixel_format == CAMERA_PF_RGB565) {
                    netconn_write(conn, http_bitmap_hdr, sizeof(http_bitmap_hdr) - 1, NETCONN_NOCOPY);
                    if (memcmp(&buf[5], "bmp", 3) == 0) {
                        char *bmp = bmp_create_header(camera_get_fb_width(), camera_get_fb_height());
                        err = netconn_write(conn, bmp, sizeof(bitmap), NETCONN_COPY);
                        free(bmp);
                    }
                } else if (s_pixel_format == CAMERA_PF_YUV422) {
                  if (memcmp(&buf[5], "bmp", 3) == 0) {
                      netconn_write(conn, http_bitmap_hdr, sizeof(http_bitmap_hdr) - 1, NETCONN_NOCOPY);
                      char *bmp = bmp_create_header(camera_get_fb_width(), camera_get_fb_height());
                      err = netconn_write(conn, bmp, sizeof(bitmap), NETCONN_COPY);
                      free(bmp);
                  } else {
                    char outstr[120];
                    get_image_info_str(outstr);
                    netconn_write(conn, outstr, sizeof(outstr) - 1, NETCONN_NOCOPY);
                    //netconn_write(conn, http_yuv422_hdr, sizeof(http_yuv422_hdr) - 1, NETCONN_NOCOPY);
                  }
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

void app_main()
{
    esp_log_level_set("wifi", ESP_LOG_WARN);
    esp_log_level_set("gpio", ESP_LOG_WARN);

    nvs_flash_init();
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
    xTaskCreate(&http_server, "http_server", 2048, NULL, 5, NULL);
}
