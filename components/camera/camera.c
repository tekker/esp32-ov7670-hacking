/*
 * Portions of this file come from OpenMV project (see sensor_* functions in the end of file)
 * Here is the copyright for these parts:
 * This file is part of the OpenMV project.
 * Copyright (c) 2013/2014 Ibrahim Abdelkader <i.abdalkader@gmail.com>
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 *
 * Rest of the functions are licensed under Apache license as found below:
 */

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
#include "time.h"
#include "sys/time.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "rom/lldesc.h"
#include "soc/soc.h"
#include "soc/gpio_sig_map.h"
#include "soc/i2s_reg.h"
#include "soc/i2s_struct.h"
#include "soc/io_mux_reg.h"
#include "driver/gpio.h"
#include "driver/periph_ctrl.h"
#include "esp_intr_alloc.h"
#include "esp_log.h"
#include "sensor.h"
#include "sccb.h"
#include "wiring.h"
#include "camera.h"
#include "camera_common.h"
#include "xclk.h"
#if CONFIG_OV2640_SUPPORT
#include "ov2640.h"
#endif
#if CONFIG_OV7725_SUPPORT
#include "ov7725.h"
#endif
#if CONFIG_OV7670_SUPPORT
#include "ov7670.h"
#endif

#define ENABLE_TEST_PATTERN CONFIG_ENABLE_TEST_PATTERN

#define REG_PID        0x0A
#define REG_VER        0x0B
#define REG_MIDH       0x1C
#define REG_MIDL       0x1D

static const char* TAG = "camera";

camera_state_t* s_state = NULL;

const int resolution[][2] = {
        { 40, 30 }, /* 40x30 */
        { 64, 32 }, /* 64x32 */
        { 64, 64 }, /* 64x64 */
        { 88, 72 }, /* QQCIF */
        { 160, 120 }, /* QQVGA */
        { 128, 160 }, /* QQVGA2*/
        { 176, 144 }, /* QCIF  */
        { 240, 160 }, /* HQVGA */
        { 320, 240 }, /* QVGA  */
        { 352, 288 }, /* CIF   */
        { 640, 480 }, /* VGA   */
        { 800, 600 }, /* SVGA  */
        { 1280, 1024 }, /* SXGA  */
        { 1600, 1200 }, /* UXGA  */
};

#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))

static void i2s_init();
static void i2s_run();
static void IRAM_ATTR gpio_isr(void* arg);
static void IRAM_ATTR i2s_isr(void* arg);
static esp_err_t dma_desc_init();
static void dma_desc_deinit();
static void dma_filter_task(void *pvParameters);

static void dma_filter_grayscale(const dma_elem_t* src, lldesc_t* dma_desc, uint8_t* dst);
static void dma_filter_grayscale_highspeed(const dma_elem_t* src, lldesc_t* dma_desc, uint8_t* dst);
static void dma_filter_jpeg(const dma_elem_t* src, lldesc_t* dma_desc, uint8_t* dst);
static void dma_filter_raw(const dma_elem_t* src, lldesc_t* dma_desc, uint8_t* dst);
static void rgbXXX_yuv_to_888(uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4, uint8_t* dst);
static void rgbXXX_yuv_to_565(uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4, uint8_t* dst);
static void dma_filter_rgbXXX(const dma_elem_t* src, lldesc_t* dma_desc, uint8_t* dst);



static void i2s_stop();

static bool yuv_test_mode = true;
static bool yuv_reverse_bytes = false;
static bool raw_bytes_only = true;
static bool gbr_rgb_order = false;
static int highspeed_sampling_mode = 2;
static bool test_tft_filter = true;

#define CAM_RES			QVGA		// カメラ解像度
#define CAM_WIDTH		320			// カメラ幅
#define CAM_HEIGHT	240			// カメラ高さ
#define CAM_DIV				3			// １画面分割数




void set_test_modes(bool s_yuv_test_mode, bool s_yuv_reverse_bytes,
                    bool s_raw_bytes_only, bool s_gbr_rgb_order, int s_highspeed_sampling_mode, bool s_test_tft_filter) {
  yuv_test_mode = s_yuv_test_mode;
  yuv_reverse_bytes = s_yuv_reverse_bytes;
  raw_bytes_only = s_raw_bytes_only;
  gbr_rgb_order = s_gbr_rgb_order;
  highspeed_sampling_mode = s_highspeed_sampling_mode;
  test_tft_filter = s_test_tft_filter;
}


static bool is_hs_mode()
{
    return s_state->config.xclk_freq_hz > 10000000;
}

static size_t i2s_bytes_per_sample(i2s_sampling_mode_t mode)
{
    switch(mode) {
        case SM_0A00_0B00:
            return 4;
        case SM_0A0B_0B0C:
            return 4;
        case SM_0A0B_0C0D:
            return 2;
        default:
            assert(0 && "invalid sampling mode");
            return 0;
    }
}

esp_err_t reset_xclk(camera_config_t* config) {
  s_state->config.xclk_freq_hz = config->xclk_freq_hz;
  ESP_LOGD(TAG, "Re-enable XCLCK");
  camera_enable_out_clock(s_state->config); // ensure timers are kept as before..
  // calling reset here will cause a FreeRTOS task related stack overflow due to the delay() that is called...
  //ESP_LOGD(TAG, "Doing SW reset of sensor");
  //s_state->sensor.reset(&s_state->sensor);
  return ESP_OK;
}

esp_err_t reset_pixformat() {

  ESP_LOGD(TAG, "Free frame buffer mem / reset RTOS tasks");

  // NOTE: Framebuffer stays!
  //if (s_state->fb != NULL) {
  //  free(s_state->fb);
  //}

  if (s_state->data_ready) {
      vQueueDelete(s_state->data_ready);
  }
  if (s_state->frame_ready) {
      vSemaphoreDelete(s_state->frame_ready);
  }
  if (s_state->dma_filter_task) {
      vTaskDelete(s_state->dma_filter_task);
  }
  dma_desc_deinit();
  return ESP_OK;
}

sensor_t* get_cam_sensor() {
   return &s_state->sensor;
}

int cam_set_sensor_reg(uint8_t reg, uint8_t regVal)
{
    return SCCB_Write(s_state->sensor.slv_addr, reg, regVal);
}

esp_err_t camera_probe(const camera_config_t* config, camera_model_t* out_camera_model)
{
    if (s_state != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    s_state = (camera_state_t*) calloc(sizeof(*s_state), 1);
    if (!s_state) {
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGD(TAG, "Enabling XCLK output");
    camera_enable_out_clock(config);

    ESP_LOGD(TAG, "Initializing SSCB");
    SCCB_Init(config->pin_sscb_sda, config->pin_sscb_scl);

    ESP_LOGD(TAG, "Resetting camera");
    gpio_config_t conf = { 0 };
    conf.pin_bit_mask = 1LL << config->pin_reset;
    conf.mode = GPIO_MODE_OUTPUT;
    gpio_config(&conf);

    gpio_pulldown_en(config->pin_reset); // ov7670 reqd
    gpio_set_level(config->pin_reset, 0);
    delay(10);
    gpio_pulldown_dis(config->pin_reset); // ov7670 reqd
    gpio_set_level(config->pin_reset, 1);
    delay(10);

    ESP_LOGD(TAG, "Searching for camera address");
    /* Probe the sensor */
    delay(10);
    uint8_t slv_addr = SCCB_Probe();
    if (slv_addr == 0) {
        *out_camera_model = CAMERA_NONE;
        return ESP_ERR_CAMERA_NOT_DETECTED;
    }
    s_state->sensor.slv_addr = slv_addr;
    ESP_LOGD(TAG, "Detected camera at address=0x%02x", slv_addr);
    sensor_id_t* id = &s_state->sensor.id;
    id->PID = SCCB_Read(slv_addr, REG_PID);
    id->VER = SCCB_Read(slv_addr, REG_VER);
    id->MIDL = SCCB_Read(slv_addr, REG_MIDL);
    id->MIDH = SCCB_Read(slv_addr, REG_MIDH);
    delay(10);
    ESP_LOGD(TAG, "Camera PID=0x%02x VER=0x%02x MIDL=0x%02x MIDH=0x%02x",
            id->PID, id->VER, id->MIDH, id->MIDL);

    switch (id->PID) {
#if CONFIG_OV2640_SUPPORT
        case OV2640_PID:
            *out_camera_model = CAMERA_OV2640;
            ov2640_init(&s_state->sensor);
            break;
#endif
#if CONFIG_OV7725_SUPPORT
        case OV7725_PID:
            *out_camera_model = CAMERA_OV7725;
            ov7725_init(&s_state->sensor);
            break;
#endif
#if CONFIG_OV7670_SUPPORT
        case OV7670_PID:
            *out_camera_model = CAMERA_OV7670;
            ov7670_init(&s_state->sensor);
            break;
#endif
        default:
            id->PID = 0;
            *out_camera_model = CAMERA_UNKNOWN;
            ESP_LOGD(TAG, "Detected camera not supported.");
            return ESP_ERR_CAMERA_NOT_SUPPORTED;
    }


    ESP_LOGD(TAG, "Doing SW reset of sensor");
    s_state->sensor.reset(&s_state->sensor);


    return ESP_OK;
}

int print_frame_data(char* outstr) {

  int cnt = 0;
  char pf[12]; //* pf = new char[12];

  switch(s_state->config.pixel_format) {
    case(PIXFORMAT_YUV422):
      sprintf(pf,"YUV422");
      break;
    case(PIXFORMAT_GRAYSCALE):
      sprintf(pf,"GRAYSCALE");
      break;
    case(PIXFORMAT_RGB565):
      sprintf(pf,"RGB565");
      break;
    case(PIXFORMAT_RGB555):
      sprintf(pf,"RGB555");
      break;
    case(PIXFORMAT_RGB444):
      sprintf(pf,"RGB444");
      break;
    default:
      sprintf(pf,"unknown");
      break;
  }

  cnt += sprintf(outstr+cnt,"img_%dx%d_%dbpp_%s",s_state->width, s_state->height, s_state->fb_bytes_per_pixel,pf);
  return cnt;

}

int get_image_mime_info_str(char* outstr) {

  int cnt = 0;
  char pf[12]; //* pf = new char[12];

  cnt += sprintf(outstr+cnt,"Content-Disposition: attachment;filename=\"");
  cnt += print_frame_data(outstr+cnt);
  //cnt += sprintf(outstr+cnt,"img_%dx%d_%dbpp_%s",s_state->width, s_state->height, s_state->fb_bytes_per_pixel,pf);
  cnt += sprintf(outstr+cnt,"\".img;Content-type: application/octet-stream\r\n\r\n");

  return cnt;
  // fname: s_state->width, s_state->height, s_state->fb_bytes_per_pixel, s_state->config.pixel_format

}



/*

camera_probe has called sensor.reset() for us...

sensor.set_brightness(0)\n"
"    sensor.set_saturation(0)\n"
"    sensor.set_gainceiling(8)\n"
"    sensor.set_contrast(2)\n"
"\n"
"    # Set sensor pixel format\n"
"    sensor.set_framesize(sensor.QVGA)\n"
"    sensor.set_pixformat(sensor.RGB565)\n"
"\n"
"    # Enable colorbar test mode\n"
"    sensor.set_colorbar(True)\n"
*/


esp_err_t camera_init(const camera_config_t* config)
{
    if (!s_state) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_state->sensor.id.PID == 0) {
        return ESP_ERR_CAMERA_NOT_SUPPORTED;
    }
    memcpy(&s_state->config, config, sizeof(*config));
    esp_err_t err = ESP_OK;
    framesize_t frame_size = (framesize_t) config->frame_size;
    pixformat_t pix_format = (pixformat_t) config->pixel_format;
    s_state->width = resolution[frame_size][0];
    s_state->height = resolution[frame_size][1];


    s_state->sensor.set_pixformat(&s_state->sensor, pix_format);

    ESP_LOGD(TAG, "Setting frame size to %dx%d", s_state->width, s_state->height);
    if (s_state->sensor.set_framesize(&s_state->sensor, frame_size) != 0) {
        ESP_LOGE(TAG, "Failed to set frame size");
        err = ESP_ERR_CAMERA_FAILED_TO_SET_FRAME_SIZE;
        goto fail;
    }

    if (pix_format == PIXFORMAT_YUV422) {
          s_state->sensor.set_brightness(&s_state->sensor,0);
          s_state->sensor.set_saturation(&s_state->sensor,0);
          s_state->sensor.set_gainceiling(&s_state->sensor,8);
          s_state->sensor.set_contrast(&s_state->sensor,2);
    }

    // check order here!??
    //s_state->sensor.set_pixformat(&s_state->sensor, pix_format);

    //if (yuv_test_mode) {
      ESP_LOGD(TAG, "yuv_test_mode set");
      s_state->sensor.set_colorbar(&s_state->sensor, yuv_test_mode);
      ESP_LOGD(TAG, "Test pattern enabled");
    //}


//#if ENABLE_TEST_PATTERN
    /* Test pattern may get handy
     if you are unable to get the live image right.
     Once test pattern is enable, sensor will output
     vertical shaded bars instead of live image.
     */
    //s_state->sensor.set_colorbar(&s_state->sensor, 1);
    //ESP_LOGD(TAG, "Test pattern enabled");
//#endif





    if (raw_bytes_only) {
      ESP_LOGD(TAG, "raw_bytes_only set - dma_filter_raw");
      ESP_LOGD(TAG, "Sending Raw Bytes from DMA to Framebuffer at %d HZ",s_state->config.xclk_freq_hz);

      s_state->fb_size = s_state->width * s_state->height * 2;
      s_state->in_bytes_per_pixel = 2;       // camera sends YUV422 (2 bytes)
      s_state->fb_bytes_per_pixel = 2;       // frame buffer stores YUYV
      s_state->dma_filter = &dma_filter_raw;

      if (highspeed_sampling_mode == 0) {
        ESP_LOGD(TAG, "Sampling mode SM_0A0B_0C0D (0)");
        s_state->sampling_mode = SM_0A0B_0C0D; // sampling mode for ov7670... works well for YUV
      } else if (highspeed_sampling_mode == 1) {
        ESP_LOGD(TAG, "Sampling mode SM_0A0B_0B0C (1)");
        s_state->sampling_mode = SM_0A0B_0B0C;
      } else if (highspeed_sampling_mode == 2) {
        ESP_LOGD(TAG, "Sampling mode SM_0A00_0B00 (2)");
        s_state->sampling_mode = SM_0A00_0B00; //highspeed
      }

    } else
    if (pix_format == PIXFORMAT_GRAYSCALE) {
      if ((s_state->sensor.id.PID != OV7725_PID) && (s_state->sensor.id.PID != OV7670_PID)) {
            ESP_LOGE(TAG, "Grayscale format is only supported for ov7225 and ov7670");
            err = ESP_ERR_NOT_SUPPORTED;
            goto fail;
        }
        ESP_LOGD(TAG, "PIXFORMAT_GRAYSCALE raw_bytes_only=false");
        if (test_tft_filter) {
          ESP_LOGD(TAG, "test_tft_filter set");
          s_state->fb_size = s_state->width * s_state->height * 2;
          s_state->fb_bytes_per_pixel = 2;
        }
        else {
          ESP_LOGD(TAG, "1 byte per pixel");
          s_state->fb_size = s_state->width * s_state->height;
          s_state->fb_bytes_per_pixel = 1;       // frame buffer stores Y8
        }
        if (is_hs_mode()) {
            ESP_LOGD(TAG, "Sampling mode SM_0A0B_0B0C (1)");
            s_state->sampling_mode = SM_0A0B_0B0C;
            s_state->dma_filter = &dma_filter_grayscale_highspeed;
        } else {
            ESP_LOGD(TAG, "Sampling mode SM_0A0B_0C0D (0)");
            s_state->sampling_mode = SM_0A0B_0C0D;
            s_state->dma_filter = &dma_filter_grayscale;
        }
        s_state->in_bytes_per_pixel = 2;       // camera sends YUYV


        // fname: s_state->width, s_state->height, s_state->fb_bytes_per_pixel, s_state->config.pixel_format

    }
    else if ((pix_format == PIXFORMAT_RGB565) || (pix_format == PIXFORMAT_YUV422)) {

        if ((s_state->sensor.id.PID != OV7725_PID) && (s_state->sensor.id.PID != OV7670_PID)) {
            ESP_LOGE(TAG, "RGB565 / YUV ... format is only supported for ov7225 and ov7670");
            err = ESP_ERR_NOT_SUPPORTED;
            goto fail;
        }

        if (test_tft_filter) {
          ESP_LOGD(TAG, "test_tft_filter - 2 byte per pixel");
          s_state->fb_size = s_state->width * s_state->height * 2;
          s_state->fb_bytes_per_pixel = 2;
        }
        else {
          ESP_LOGD(TAG, "3 byte per pixel");
          s_state->fb_size = s_state->width * s_state->height * 3;
          s_state->fb_bytes_per_pixel = 3;
        }

        if (is_hs_mode()) {
            ESP_LOGD(TAG, "Sampling mode SM_0A0B_0B0C (1)");
            s_state->sampling_mode = SM_0A0B_0B0C; // sampling mode for ov7670...
            s_state->dma_filter = &dma_filter_rgbXXX; //&dma_filter_rgb565_highspeed;
        } else {
            ESP_LOGD(TAG, "Sampling mode SM_0A00_0B00 (2)");
            s_state->sampling_mode = SM_0A00_0B00; // sampling mode for ov7670...
            s_state->dma_filter = &dma_filter_rgbXXX;
        }
        s_state->in_bytes_per_pixel = 2;       // camera sends RGB565 (2 bytes)


    } else if (pix_format == PIXFORMAT_JPEG) {
        if (s_state->sensor.id.PID != OV2640_PID) {
            ESP_LOGE(TAG, "JPEG format is only supported for ov2640");
            err = ESP_ERR_NOT_SUPPORTED;
            goto fail;
        }
        int qp = config->jpeg_quality;
        int compression_ratio_bound;
        if (qp >= 30) {
            compression_ratio_bound = 5;
        } else if (qp >= 10) {
            compression_ratio_bound = 10;
        } else {
            compression_ratio_bound = 20;
        }
        (*s_state->sensor.set_quality)(&s_state->sensor, qp);
        size_t equiv_line_count = s_state->height / compression_ratio_bound;
        s_state->fb_size = s_state->width * equiv_line_count * 2 /* bpp */;
        s_state->dma_filter = &dma_filter_jpeg;
        if (is_hs_mode()) {
            s_state->sampling_mode = SM_0A0B_0B0C;
        } else {
            s_state->sampling_mode = SM_0A00_0B00;
        }
        s_state->in_bytes_per_pixel = 2;
        s_state->fb_bytes_per_pixel = 2;
    } else {
        ESP_LOGE(TAG, "Requested format is not supported");
        err = ESP_ERR_NOT_SUPPORTED;
        goto fail;
    }

    ESP_LOGD(TAG, "in_bpp: %d, fb_bpp: %d, fb_size: %d, mode: %d, width: %d height: %d",
            s_state->in_bytes_per_pixel, s_state->fb_bytes_per_pixel,
            s_state->fb_size, s_state->sampling_mode,
            s_state->width, s_state->height);

    ESP_LOGD(TAG, "Allocating frame buffer (%d bytes)", s_state->fb_size);

    // TODO! TEKKER KLUDGE
    if (s_state->fb == NULL) {
      ESP_LOGD(TAG, "TEKKER KLUDGE - mAX framebuffer 2bpp");

      int max_fb_size = 320 * 240 * 2;
      //s_state->width * s_state->height * 2;
      s_state->fb = (uint8_t*) calloc(max_fb_size, 1);
      ESP_LOGD(TAG, "Allocated frame buffer (%d bytes)", max_fb_size);
    }
    if (s_state->fb == NULL) {
        ESP_LOGE(TAG, "Failed to allocate frame buffer");
        err = ESP_ERR_NO_MEM;
        goto fail;
    }

    ESP_LOGD(TAG, "Initializing I2S and DMA");
    i2s_init();
    err = dma_desc_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2S and DMA");
        goto fail;
    }

    s_state->data_ready = xQueueCreate(16, sizeof(size_t));
    s_state->frame_ready = xSemaphoreCreateBinary();
    if (s_state->data_ready == NULL || s_state->frame_ready == NULL) {
        ESP_LOGE(TAG, "Failed to create semaphores");
        err = ESP_ERR_NO_MEM;
        goto fail;
    }

    if (!xTaskCreatePinnedToCore(&dma_filter_task, "dma_filter", 4096, NULL, 10, &s_state->dma_filter_task, 1)) {
       ESP_LOGE(TAG, "Failed to create DMA filter task");
       err = ESP_ERR_NO_MEM;
       goto fail;
    }
/*
// TEKKER MOD!!!!
    if (test_tft_filter) {
      ESP_LOGI(TAG, "Creating DMA filter task TFT");
      if (!xTaskCreatePinnedToCore(&dma_filter_task_tft, "dma_filter", 4096, NULL, 10, &s_state->dma_filter_task, 1)) {
         ESP_LOGE(TAG, "Failed to create DMA filter task TFT");
         err = ESP_ERR_NO_MEM;
         goto fail;
      }
    } else {

     if (!xTaskCreatePinnedToCore(&dma_filter_task, "dma_filter", 4096, NULL, 10, &s_state->dma_filter_task, 1)) {
        ESP_LOGE(TAG, "Failed to create DMA filter task");
        err = ESP_ERR_NO_MEM;
        goto fail;
     }
    }
*/
    //delay(1);

    ESP_LOGD(TAG, "Initializing GPIO interrupts");
    gpio_set_intr_type(s_state->config.pin_vsync, GPIO_INTR_NEGEDGE);
    gpio_intr_enable(s_state->config.pin_vsync);

    if (&s_state->vsync_intr_handle == NULL) {
      ESP_LOGD(TAG, "Initializing GPIO ISR Register");
      err = gpio_isr_register(&gpio_isr, (void*) TAG,
              ESP_INTR_FLAG_INTRDISABLED | ESP_INTR_FLAG_IRAM,
              &s_state->vsync_intr_handle);
      if (err != ESP_OK) {
          ESP_LOGE(TAG, "gpio_isr_register failed (%x)", err);
          goto fail;
      }
    } else {
      ESP_LOGD(TAG, "Skipping GPIO ISR Register, already enabled...");
    }

    // skip at least one frame after changing camera settings
    while (gpio_get_level(s_state->config.pin_vsync) == 0) {
        ;
    }
    while (gpio_get_level(s_state->config.pin_vsync) != 0) {
        ;
    }
    while (gpio_get_level(s_state->config.pin_vsync) == 0) {
        ;
    }
    s_state->frame_count = 0;
    //ESP_LOGD(TAG, "Init done");


    return ESP_OK;

fail:

    if (s_state->fb != NULL)
      free(s_state->fb);

    if (s_state->data_ready) {
        vQueueDelete(s_state->data_ready);
    }
    if (s_state->frame_ready) {
        vSemaphoreDelete(s_state->frame_ready);
    }
    if (s_state->dma_filter_task) {
        vTaskDelete(s_state->dma_filter_task);
    }
    dma_desc_deinit();
    ESP_LOGE(TAG, "Init Failed");
    return err;
}

uint8_t* camera_get_fb()
{
    if (s_state == NULL) {
        return NULL;
    }
    return s_state->fb;
}

int camera_get_fb_width()
{
    if (s_state == NULL) {
        return 0;
    }
    return s_state->width;
}

int camera_get_fb_height()
{
    if (s_state == NULL) {
        return 0;
    }
    return s_state->height;
}

size_t camera_get_data_size()
{
    if (s_state == NULL) {
        return 0;
    }
    return s_state->data_size;
}

static char frame_info_str[40]; //

esp_err_t camera_run()
{
    if (s_state == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    struct timeval tv_start;
    gettimeofday(&tv_start, NULL);
#ifndef _NDEBUG
    memset(s_state->fb, 0, s_state->fb_size);
#endif // _NDEBUG
    i2s_run();

    // set

    ESP_LOGD(TAG, "Waiting for frame");

    xSemaphoreTake(s_state->frame_ready, portMAX_DELAY);
    struct timeval tv_end;
    gettimeofday(&tv_end, NULL);
    int time_ms = (tv_end.tv_sec - tv_start.tv_sec) * 1000 + (tv_end.tv_usec - tv_start.tv_usec) / 1000;

//    print_frame_data(frame_info_str);
//    ESP_LOGI(TAG, "Frame format %s : %d done in %d ms", frame_info_str, s_state->frame_count, time_ms);
    ESP_LOGI(TAG, "Frame %d done in %d ms", s_state->frame_count, time_ms);

    s_state->frame_count++;
    return ESP_OK;
}

static esp_err_t dma_desc_init()
{
    assert(s_state->width % 4 == 0);
    size_t line_size = s_state->width * s_state->in_bytes_per_pixel *
            i2s_bytes_per_sample(s_state->sampling_mode);
    ESP_LOGD(TAG, "Line width (for DMA): %d bytes", line_size);
    size_t dma_per_line = 1;
    size_t buf_size = line_size;
    while (buf_size >= 4096) {
        buf_size /= 2;
        dma_per_line *= 2;
    }
    size_t dma_desc_count = dma_per_line * 4;
    s_state->dma_buf_width = line_size;
    s_state->dma_per_line = dma_per_line;
    s_state->dma_desc_count = dma_desc_count;
    ESP_LOGD(TAG, "DMA buffer size: %d, DMA buffers per line: %d", buf_size, dma_per_line);
    ESP_LOGD(TAG, "DMA buffer count: %d", dma_desc_count);

    s_state->dma_buf = (dma_elem_t**) malloc(sizeof(dma_elem_t*) * dma_desc_count);
    if (s_state->dma_buf == NULL) {
        return ESP_ERR_NO_MEM;
    }
    s_state->dma_desc = (lldesc_t*) malloc(sizeof(lldesc_t) * dma_desc_count);
    if (s_state->dma_desc == NULL) {
        return ESP_ERR_NO_MEM;
    }
    size_t dma_sample_count = 0;
    for (int i = 0; i < dma_desc_count; ++i) {
        ESP_LOGD(TAG, "Allocating DMA buffer #%d, size=%d", i, buf_size);
        dma_elem_t* buf = (dma_elem_t*) malloc(buf_size);
        if (buf == NULL) {
            return ESP_ERR_NO_MEM;
        }
        s_state->dma_buf[i] = buf;
        ESP_LOGV(TAG, "dma_buf[%d]=%p", i, buf);

        lldesc_t* pd = &s_state->dma_desc[i];
        pd->length = buf_size;
        if (s_state->sampling_mode == SM_0A0B_0B0C &&
            (i + 1) % dma_per_line == 0) {
            pd->length -= 4;
        }
        dma_sample_count += pd->length / 4;
        pd->size = pd->length;
        pd->owner = 1;
        pd->sosf = 1;
        pd->buf = (uint8_t*) buf;
        pd->offset = 0;
        pd->empty = 0;
        pd->eof = 1;
        pd->qe.stqe_next = &s_state->dma_desc[(i + 1) % dma_desc_count];
    }
    s_state->dma_done = false;
    s_state->dma_sample_count = dma_sample_count;
    return ESP_OK;
}

static void dma_desc_deinit()
{
    if (s_state->dma_buf) {
        for (int i = 0; i < s_state->dma_desc_count; ++i) {
            free(s_state->dma_buf[i]);
        }
    }
    free(s_state->dma_buf);
    free(s_state->dma_desc);
}

static inline void i2s_conf_reset()
{
   // as per nkolban: https://github.com/igrr/esp32-cam-demo/issues/36
    const uint32_t lc_conf_reset_flags = I2S_IN_RST_M | I2S_AHBM_RST_M | I2S_AHBM_FIFO_RST_M;
    //const uint32_t lc_conf_reset_flags = I2S_IN_RST_S | I2S_AHBM_RST_S
    //        | I2S_AHBM_FIFO_RST_S;
    I2S0.lc_conf.val |= lc_conf_reset_flags;
    I2S0.lc_conf.val &= ~lc_conf_reset_flags;

    const uint32_t conf_reset_flags = I2S_RX_RESET_M | I2S_RX_FIFO_RESET_M
            | I2S_TX_RESET_M | I2S_TX_FIFO_RESET_M;
    I2S0.conf.val |= conf_reset_flags;
    I2S0.conf.val &= ~conf_reset_flags;
    while (I2S0.state.rx_fifo_reset_back) {
        ;
    }
}

static void i2s_init()
{
    camera_config_t* config = &s_state->config;

    // Configure input GPIOs
    gpio_num_t pins[] = {
            config->pin_d7,
            config->pin_d6,
            config->pin_d5,
            config->pin_d4,
            config->pin_d3,
            config->pin_d2,
            config->pin_d1,
            config->pin_d0,
            config->pin_vsync,
            config->pin_href,
            config->pin_pclk
    };
    gpio_config_t conf = {
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
    };
    for (int i = 0; i < sizeof(pins) / sizeof(gpio_num_t); ++i) {
        conf.pin_bit_mask = 1LL << pins[i];
        gpio_config(&conf);
    }

    // Route input GPIOs to I2S peripheral using GPIO matrix
    gpio_matrix_in(config->pin_d0, I2S0I_DATA_IN0_IDX, false);
    gpio_matrix_in(config->pin_d1, I2S0I_DATA_IN1_IDX, false);
    gpio_matrix_in(config->pin_d2, I2S0I_DATA_IN2_IDX, false);
    gpio_matrix_in(config->pin_d3, I2S0I_DATA_IN3_IDX, false);
    gpio_matrix_in(config->pin_d4, I2S0I_DATA_IN4_IDX, false);
    gpio_matrix_in(config->pin_d5, I2S0I_DATA_IN5_IDX, false);
    gpio_matrix_in(config->pin_d6, I2S0I_DATA_IN6_IDX, false);
    gpio_matrix_in(config->pin_d7, I2S0I_DATA_IN7_IDX, false);
    gpio_matrix_in(config->pin_vsync, I2S0I_V_SYNC_IDX, false);
    gpio_matrix_in(0x38, I2S0I_H_SYNC_IDX, false);
    gpio_matrix_in(config->pin_href, I2S0I_H_ENABLE_IDX, false);
    gpio_matrix_in(config->pin_pclk, I2S0I_WS_IN_IDX, false);

    // Enable and configure I2S peripheral
    periph_module_enable(PERIPH_I2S0_MODULE);
    // Toggle some reset bits in LC_CONF register
    // Toggle some reset bits in CONF register
    i2s_conf_reset();
    // Enable slave mode (sampling clock is external)
    I2S0.conf.rx_slave_mod = 1;
    // Enable parallel mode
    I2S0.conf2.lcd_en = 1;
    // Use HSYNC/VSYNC/HREF to control sampling
    I2S0.conf2.camera_en = 1;
    // Configure clock divider
    I2S0.clkm_conf.clkm_div_a = 1;
    I2S0.clkm_conf.clkm_div_b = 0;
    I2S0.clkm_conf.clkm_div_num = 2;
    // FIFO will sink data to DMA
    I2S0.fifo_conf.dscr_en = 1;
    // FIFO configuration
    I2S0.fifo_conf.rx_fifo_mod = s_state->sampling_mode;
    I2S0.fifo_conf.rx_fifo_mod_force_en = 1;
    I2S0.conf_chan.rx_chan_mod = 1;
    // Clear flags which are used in I2S serial mode
    I2S0.sample_rate_conf.rx_bits_mod = 0;
    I2S0.conf.rx_right_first = 0;
    I2S0.conf.rx_msb_right = 0;
    I2S0.conf.rx_msb_shift = 0;
    I2S0.conf.rx_mono = 0;
    I2S0.conf.rx_short_sync = 0;
    I2S0.timing.val = 0;

    // Allocate I2S interrupt, keep it disabled
    esp_intr_alloc(ETS_I2S0_INTR_SOURCE,
    ESP_INTR_FLAG_INTRDISABLED | ESP_INTR_FLAG_LEVEL1 | ESP_INTR_FLAG_IRAM,
            &i2s_isr, NULL, &s_state->i2s_intr_handle);
}


static void i2s_stop()
{
    esp_intr_disable(s_state->i2s_intr_handle);
    esp_intr_disable(s_state->vsync_intr_handle);
    i2s_conf_reset();
    I2S0.conf.rx_start = 0;
    size_t val = SIZE_MAX;
    BaseType_t higher_priority_task_woken;
    xQueueSendFromISR(s_state->data_ready, &val, &higher_priority_task_woken);
}

static void i2s_run()
{
#ifndef _NDEBUG
    for (int i = 0; i < s_state->dma_desc_count; ++i) {
        lldesc_t* d = &s_state->dma_desc[i];
        ESP_LOGV(TAG, "DMA desc %2d: %u %u %u %u %u %u %p %p",
                i, d->length, d->size, d->offset, d->eof, d->sosf, d->owner, d->buf, d->qe.stqe_next);
        memset(s_state->dma_buf[i], 0, d->length);
    }
#endif

    // wait for vsync
    ESP_LOGD(TAG, "Waiting for positive edge on VSYNC");
    while (gpio_get_level(s_state->config.pin_vsync) == 0) {
        ;
    }
    while (gpio_get_level(s_state->config.pin_vsync) != 0) {
        ;
    }
    ESP_LOGD(TAG, "Got VSYNC");

    s_state->dma_done = false;
    s_state->dma_desc_cur = 0;
    s_state->dma_received_count = 0;
    s_state->dma_filtered_count = 0;
    esp_intr_disable(s_state->i2s_intr_handle);
    i2s_conf_reset();

    I2S0.rx_eof_num = s_state->dma_sample_count;
    I2S0.in_link.addr = (uint32_t) &s_state->dma_desc[0];
    I2S0.in_link.start = 1;
    I2S0.int_clr.val = I2S0.int_raw.val;
    I2S0.int_ena.val = 0;
    I2S0.int_ena.in_done = 1;
    esp_intr_enable(s_state->i2s_intr_handle);

    if (s_state->config.pixel_format == CAMERA_PF_JPEG) {
        esp_intr_enable(s_state->vsync_intr_handle);
    }

    I2S0.conf.rx_start = 1;

}

static void IRAM_ATTR signal_dma_buf_received(bool* need_yield)
{
    size_t dma_desc_filled = s_state->dma_desc_cur;
    s_state->dma_desc_cur = (dma_desc_filled + 1) % s_state->dma_desc_count;
    s_state->dma_received_count++;
    BaseType_t higher_priority_task_woken;
    BaseType_t ret = xQueueSendFromISR(s_state->data_ready, &dma_desc_filled, &higher_priority_task_woken);
    if (ret != pdTRUE) {
        ESP_EARLY_LOGW(TAG, "queue send failed (%d), dma_received_count=%d", ret, s_state->dma_received_count);
    }
    *need_yield = (ret == pdTRUE && higher_priority_task_woken == pdTRUE);
}

static void IRAM_ATTR i2s_isr(void* arg)
{
    I2S0.int_clr.val = I2S0.int_raw.val;
    bool need_yield;
    signal_dma_buf_received(&need_yield);
    ESP_EARLY_LOGV(TAG, "isr, cnt=%d", s_state->dma_received_count);
    if (s_state->dma_received_count == s_state->height * s_state->dma_per_line) {
        i2s_stop();
    }
    if (need_yield) {
        portYIELD_FROM_ISR();
    }
}

static void IRAM_ATTR gpio_isr(void* arg)
{
    GPIO.status1_w1tc.val = GPIO.status1.val;
    GPIO.status_w1tc = GPIO.status;
    bool need_yield = false;
    ESP_EARLY_LOGV(TAG, "gpio isr, cnt=%d", s_state->dma_received_count);
    if (gpio_get_level(s_state->config.pin_vsync) == 0 &&
            s_state->dma_received_count > 0 &&
            !s_state->dma_done) {
        signal_dma_buf_received(&need_yield);
        i2s_stop();
    }
    if (need_yield) {
        portYIELD_FROM_ISR();
    }
}

static size_t get_fb_pos()
{
    return s_state->dma_filtered_count * s_state->width *
            s_state->fb_bytes_per_pixel / s_state->dma_per_line;
}



static void IRAM_ATTR dma_filter_task(void *pvParameters)
{
    while (true) {
        size_t buf_idx;
        xQueueReceive(s_state->data_ready, &buf_idx, portMAX_DELAY);
        if (buf_idx == SIZE_MAX) {
            s_state->data_size = get_fb_pos();
            xSemaphoreGive(s_state->frame_ready);
            continue;
        }

        uint8_t* pfb = s_state->fb + get_fb_pos();
        const dma_elem_t* buf = s_state->dma_buf[buf_idx];
        lldesc_t* desc = &s_state->dma_desc[buf_idx];
        ESP_LOGV(TAG, "dma_flt: pos=%d ", get_fb_pos());
        (*s_state->dma_filter)(buf, desc, pfb);
        s_state->dma_filtered_count++;
        ESP_LOGV(TAG, "dma_flt: flt_count=%d ", s_state->dma_filtered_count);
    }
}


/* convert a YUV set to a rgb set - thanks to MartinS and
   http://www.efg2.com/lab/Graphics/Colors/YUV.htm */
static void yuvtorgb(int Y, int U, int V, uint8_t *rgb)
{
	int r, g, b;
	static short L1[256], L2[256], L3[256], L4[256], L5[256];
	static int initialised;

	if (!initialised) {
		int i;
		initialised=1;
		for (i=0;i<256;i++) {
			L1[i] = 1.164*(i-16);
			L2[i] = 1.596*(i-128);
			L3[i] = -0.813*(i-128);
			L4[i] = 2.018*(i-128);
			L5[i] = -0.391*(i-128);
		}
	}
#if 0
	r = 1.164*(Y-16) + 1.596*(V-128);
	g = 1.164*(Y-16) - 0.813*(U-128) - 0.391*(V-128);
	b = 1.164*(Y-16) + 2.018*(U-128);
#endif

	r = L1[Y] + L2[V];
	g = L1[Y] + L3[U] + L5[V];
	b = L1[Y] + L4[U];

	if (r < 0) r = 0;
	if (g < 0) g = 0;
	if (b < 0) b = 0;
	if (r > 255) r = 255;
	if (g > 255) g = 255;
	if (b > 255) b = 255;

	rgb[0] = r;
	rgb[1] = g;
	rgb[2] = b;
}

/* convert yuv to rgb */
void yuv_convert(uint8_t *buf, uint8_t *rgb, int xsize, int ysize)
{
	int i;

	for (i=0;i<xsize*ysize;i+=2) {
		int Y1, Y2, U, V;

		Y1 = buf[2*i+0];
		Y2 = buf[2*i+2];
		U = buf[2*i+1];
		V = buf[2*i+3];

		yuvtorgb(Y1, U, V, &rgb[3*i]);
		yuvtorgb(Y2, U, V, &rgb[3*(i+1)]);
	}
}





static inline uint16_t rgb888torgb565(uint8_t red, uint8_t green, uint8_t blue)
{

    uint16_t b = (blue >> 3) & 0x1f;
    uint16_t g = ((green >> 2) & 0x3f) << 5;
    uint16_t r = ((red >> 3) & 0x1f) << 11;

    return (uint16_t) (r | g | b);
}

void transformFramebuffer888to565() {

  uint8_t* pfb = s_state->fb;
  uint16_t* x16_pt;

  for (int x = 0; x < s_state->fb_size; x+=3) {
    x16_pt = pfb;
    *x16_pt = rgb888torgb565(pfb[0],pfb[1],pfb[2]);
    pfb += 3;
    x16_pt += 2;
  }

}

/*
void YUVToRGB565(int width, int height, const unsigned char *src, unsigned short *dst)
{
  int line, col, linewidth;
  int y, u, v, yy, vr, ug, vg, ub;
  int r, g, b;
  const unsigned char *py, *pu, *pv;

  linewidth = width>>1;
  py = src;
  pu = py + (width * height);
  pv = pu + (width * height) / 4;

  y = *py++;
  yy = y <<8;
  u = *pu - 128;
  ug = 88 * u;
  ub = 454 * u;
  v = *pv - 128;
  vg = 183 * v;
  vr = 359 * v;

  for (line = 0; line < height; line++)
  {
    for (col = 0; col < width; col++)
    {
        r = (yy + vr) >> 8;
        g = (yy - ug - vg) >> 8;
        b = (yy + ub) >> 8;

        if (r < 0) r = 0;
        if (r > 255) r = 255;
        if (g < 0) g = 0;
        if (g > 255) g = 255;
        if (b < 0) b = 0;
        if (b > 255) b = 255;
        *dst++ = (((unsigned short )r>>3)<<11) | (((unsigned short)g>>2)<<5) | (((unsigned short )b>>3)<<0);

        y = *py++;
        yy = y << 8;
        if (col & 1)
        {
          pu++;
          pv++;

          u = *pu - 128;
          ug = 88 * u;
          ub = 454 * u;
          v = *pv - 128;
          vg = 183 * v;
          vr = 359 * v;
        }
    } // ..for col
    if ((line & 1) == 0)
    { // even line: rewind
      pu -= linewidth;
      pv -= linewidth;
    }
  } // ..for line
}
*/

static inline void rgbXXX_yuv_to_565(uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4, uint8_t* dst)
{
  int r1=0, g1=0, b1=0, r2=0, g2=0, b2=0;

  uint16_t index = 0;
  pixformat_t pix_format = (pixformat_t)s_state->config.pixel_format;
  if (pix_format == PIXFORMAT_YUV422) {
    b1 = d2 + 1.4075 * (d1 - 128);
    g1 = d2 - 0.3455 * (d1 - 128) - 0.7169 * (d3 - 128);
    r2 = d2 + 1.7790 * (d3 - 128);

    b2 = d4 + 1.4075 * (d1 - 128);
    g2 = d4 - 0.3455 * (d1 - 128) - 0.7169 * (d3 - 128);
    r2 = d4 + 1.7790 * (d3 - 128);

    b1 = min(max(b1, 0), 255);
    g1 = min(max(g1, 0), 255);
    r1 = min(max(r1, 0), 255);

    b2 = min(max(b2, 0), 255);
    g2 = min(max(g2, 0), 255);
    r2 = min(max(r2, 0), 255);
  } else if (pix_format == PIXFORMAT_RGB444) {
     b1 = (d1 & 0x0F) << 4;
     g1 = (d2 & 0xF0);
     r1 = (d2 & 0x0F) << 4;

     b2 = (d3 & 0x0F) << 4;
     g2 = (d4 & 0xF0);
     r2 = (d4 & 0x0F) << 4;
   } else if (pix_format == PIXFORMAT_RGB555) {
     b1 = (d1 & 0x1F) << 3;
     g1 = (((d1 & 0xE0) >> 2) | ((d2 & 0x03) << 6));
     r1 = (d2 & 0x7c) << 1;

     b2 = (d3 & 0x1F) << 3;
     g2 = (((d3 & 0xE0) >> 2) | ((d4 & 0x03) << 6));
     r2 = (d4 & 0x7c) << 1;
   } else if (pix_format == PIXFORMAT_RGB565) {
     b1 = (d1 & 0x1F) << 3;
     g1 = (((d1 & 0xE0) >> 3) | ((d2 & 0x07) << 5));
     r1 = (d2 & 0xF8);

     b1 = (d3 & 0x1F) << 3;
     g1 = (((d3 & 0xE0) >> 3) | ((d4 & 0x07) << 5));
     r1 = (d4 & 0xF8);
   }

   dst[0] = r1;
   dst[1] = g1;
   dst[2] = b1;

   dst[3] = r2;
   dst[4] = g2;
   dst[5] = b2;

   *dst = (((unsigned short )r1>>3)<<11) | (((unsigned short)g1>>2)<<5) | (((unsigned short )b1>>3)<<0);
   *(dst+2) = (((unsigned short )r2>>3)<<11) | (((unsigned short)g2>>2)<<5) | (((unsigned short )b2>>3)<<0);

}


static inline void rgbXXX_yuv_to_888(uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4, uint8_t* dst)
{
   int r1=0, g1=0, b1=0, r2=0, g2=0, b2=0;

   uint16_t index = 0;
   pixformat_t pix_format = (pixformat_t)s_state->config.pixel_format;
   if (pix_format == PIXFORMAT_YUV422) {
     b1 = d2 + 1.4075 * (d1 - 128);
     g1 = d2 - 0.3455 * (d1 - 128) - 0.7169 * (d3 - 128);
     r2 = d2 + 1.7790 * (d3 - 128);

     b2 = d4 + 1.4075 * (d1 - 128);
     g2 = d4 - 0.3455 * (d1 - 128) - 0.7169 * (d3 - 128);
     r2 = d4 + 1.7790 * (d3 - 128);

     b1 = min(max(b1, 0), 255);
     g1 = min(max(g1, 0), 255);
     r1 = min(max(r1, 0), 255);

     b2 = min(max(b2, 0), 255);
     g2 = min(max(g2, 0), 255);
     r2 = min(max(r2, 0), 255);
   } else if (pix_format == PIXFORMAT_RGB444) {
      b1 = (d1 & 0x0F) << 4;
      g1 = (d2 & 0xF0);
      r1 = (d2 & 0x0F) << 4;

      b2 = (d3 & 0x0F) << 4;
      g2 = (d4 & 0xF0);
      r2 = (d4 & 0x0F) << 4;
    } else if (pix_format == PIXFORMAT_RGB555) {
      b1 = (d1 & 0x1F) << 3;
      g1 = (((d1 & 0xE0) >> 2) | ((d2 & 0x03) << 6));
      r1 = (d2 & 0x7c) << 1;

      b2 = (d3 & 0x1F) << 3;
      g2 = (((d3 & 0xE0) >> 2) | ((d4 & 0x03) << 6));
      r2 = (d4 & 0x7c) << 1;
    } else if (pix_format == PIXFORMAT_RGB565) {
      b1 = (d1 & 0x1F) << 3;
      g1 = (((d1 & 0xE0) >> 3) | ((d2 & 0x07) << 5));
      r1 = (d2 & 0xF8);

      b1 = (d3 & 0x1F) << 3;
      g1 = (((d3 & 0xE0) >> 3) | ((d4 & 0x07) << 5));
      r1 = (d4 & 0xF8);
    }

    dst[0] = r1;
    dst[1] = g1;
    dst[2] = b1;

    dst[3] = r2;
    dst[4] = g2;
    dst[5] = b2;

}


static void IRAM_ATTR dma_filter_rgbXXX(const dma_elem_t* src, lldesc_t* dma_desc, uint8_t* dst)
{
  //assert(s_state->sampling_mode == SM_0A0B_0C0D);
  if (s_state->sampling_mode == SM_0A0B_0C0D) {
    size_t end = dma_desc->length / sizeof(dma_elem_t) / 4;
    for (size_t i = 0; i < end; ++i) {
      // manually unrolling 4 iterations of the loop here
      if (test_tft_filter) {
        rgbXXX_yuv_to_565(src[0].sample1, src[0].sample2, src[1].sample1, src[1].sample2, &dst[0]);
        rgbXXX_yuv_to_565(src[2].sample1, src[2].sample2,src[3].sample1, src[3].sample2, &dst[2]);
        src += 4;
        dst += 4;
      } else {
       rgbXXX_yuv_to_888(src[0].sample1, src[0].sample2, src[1].sample1, src[1].sample2, &dst[0]);
       rgbXXX_yuv_to_888(src[2].sample1, src[2].sample2,src[3].sample1, src[3].sample2, &dst[6]);
       src += 4;
       dst += 12;
      }
     }
   } else
   if (s_state->sampling_mode == SM_0A0B_0B0C ||
      s_state->sampling_mode == SM_0A00_0B00) {

    const int unroll = 2;         // manually unrolling 2 iterations of the loop
    const int samples_per_pixel = 2;
    const int bytes_per_pixel = 3;
    size_t end = dma_desc->length / sizeof(dma_elem_t) / unroll / samples_per_pixel;
    for (size_t i = 0; i < end; ++i) {
      if (test_tft_filter) {
        rgbXXX_yuv_to_565(src[0].sample1, src[1].sample1, src[2].sample1, src[3].sample1, &dst[0]);
        dst += 2 * unroll;
        src += samples_per_pixel * unroll;
      } else {
        rgbXXX_yuv_to_888(src[0].sample1, src[1].sample1, src[2].sample1, src[3].sample1, &dst[0]);
        dst += bytes_per_pixel * unroll;
        src += samples_per_pixel * unroll;
      }
    }
    if ((dma_desc->length & 0x7) != 0) {
      if (test_tft_filter) {
        rgbXXX_yuv_to_565(src[0].sample1, src[1].sample1, src[2].sample1, src[2].sample2, &dst[0]);
      } else {
        rgbXXX_yuv_to_888(src[0].sample1, src[1].sample1, src[2].sample1, src[2].sample2, &dst[0]);
      }
    }
  }

}

static void IRAM_ATTR dma_filter_grayscale(const dma_elem_t* src, lldesc_t* dma_desc, uint8_t* dst)
{
    assert(s_state->sampling_mode == SM_0A0B_0C0D);
    size_t end = dma_desc->length / sizeof(dma_elem_t) / 4;
    for (size_t i = 0; i < end; ++i) {
        // manually unrolling 4 iterations of the loop here
        dst[0] = src[0].sample1;
        dst[1] = src[1].sample1;
        dst[2] = src[2].sample1;
        dst[3] = src[3].sample1;
        src += 4;
        dst += 4;
    }
}



static void IRAM_ATTR dma_filter_grayscale_highspeed(const dma_elem_t* src, lldesc_t* dma_desc, uint8_t* dst)
{
    assert(s_state->sampling_mode == SM_0A0B_0B0C);
    size_t end = dma_desc->length / sizeof(dma_elem_t) / 8;
    for (size_t i = 0; i < end; ++i) {
        // manually unrolling 4 iterations of the loop here
        dst[0] = src[0].sample1;
        dst[1] = src[2].sample1;
        dst[2] = src[4].sample1;
        dst[3] = src[6].sample1;
        src += 8;
        dst += 4;
    }
    // the final sample of a line in SM_0A0B_0B0C sampling mode needs special handling
    if ((dma_desc->length & 0x7) != 0) {
        dst[0] = src[0].sample1;
        dst[1] = src[2].sample1;
    }
}

static void IRAM_ATTR dma_filter_jpeg(const dma_elem_t* src, lldesc_t* dma_desc, uint8_t* dst)
{
    assert(s_state->sampling_mode == SM_0A0B_0B0C ||
           s_state->sampling_mode == SM_0A00_0B00 );
    size_t end = dma_desc->length / sizeof(dma_elem_t) / 4;
    // manually unrolling 4 iterations of the loop here
    for (size_t i = 0; i < end; ++i) {
        dst[0] = src[0].sample1;
        dst[1] = src[1].sample1;
        dst[2] = src[2].sample1;
        dst[3] = src[3].sample1;
        src += 4;
        dst += 4;
    }
    // the final sample of a line in SM_0A0B_0B0C sampling mode needs special handling
    if ((dma_desc->length & 0x7) != 0) {
        dst[0] = src[0].sample1;
        dst[1] = src[1].sample1;
        dst[2] = src[2].sample1;
        dst[3] = src[2].sample2;
    }
}

// basically bytes in == bytes out in this mode
static void IRAM_ATTR dma_filter_raw(const dma_elem_t* src, lldesc_t* dma_desc, uint8_t* dst)
{

  if (s_state->sampling_mode == SM_0A0B_0C0D) {
  size_t end = dma_desc->length / sizeof(dma_elem_t) / 4;
  for (size_t i = 0; i < end; ++i) {
      // manually unrolling 4 iterations of the loop here
      dst[1] = src[0].sample1; // hmmm switch it on em for 00 0c0d?
      dst[0] = src[0].sample2;
      dst[3] = src[1].sample1;
      dst[2] = src[1].sample2;
      dst[5] = src[2].sample1;
      dst[4] = src[2].sample2;
      dst[7] = src[3].sample1;
      dst[6] = src[3].sample2;
      src += 4;
      dst += 8;
  }
} else {
  assert(s_state->sampling_mode == SM_0A0B_0B0C ||
         s_state->sampling_mode == SM_0A00_0B00);
  size_t end = dma_desc->length / sizeof(dma_elem_t) / 4;
  // manually unrolling 4 iterations of the loop here
  for (size_t i = 0; i < end; ++i) {
      dst[0] = src[0].sample1;
      dst[1] = src[1].sample1;
      dst[2] = src[2].sample1;
      dst[3] = src[3].sample1;
      src += 4;
      dst += 4;
  }
  // the final sample of a line in SM_0A0B_0B0C sampling mode needs special handling
  if ((dma_desc->length & 0x7) != 0) {
      dst[0] = src[0].sample1;
      dst[1] = src[1].sample1;
      dst[2] = src[2].sample1;
      dst[3] = src[2].sample2;
  }

}
}
