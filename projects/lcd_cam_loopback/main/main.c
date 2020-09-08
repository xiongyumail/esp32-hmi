#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event_loop.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "cam.h"
#include "ov2640.h"
#include "ov3660.h"
#include "sensor.h"
#include "sccb.h"
#include "lcd.h"
#include "jpeg.h"

static const char *TAG = "main";

#define JPEG_MODE 1
#define YUV_MODE 0
#define JPEG_ENCODE 0

#define CAM_WIDTH   (240)
#define CAM_HIGH    (240)

#define LCD_CLK   GPIO_NUM_12
#define LCD_MOSI  GPIO_NUM_22
#define LCD_DC    GPIO_NUM_2
#define LCD_CS    GPIO_NUM_15
#define LCD_RST   GPIO_NUM_26
#define LCD_BK    -1

#define CAM_XCLK  GPIO_NUM_4
#define CAM_PCLK  GPIO_NUM_25
#define CAM_VSYNC GPIO_NUM_5
#define CAM_HSYNC GPIO_NUM_27

#define CAM_D0    GPIO_NUM_34
#define CAM_D1    GPIO_NUM_13
#define CAM_D2    GPIO_NUM_14
#define CAM_D3    GPIO_NUM_35
#define CAM_D4    GPIO_NUM_39
#define CAM_D5    GPIO_NUM_21
#define CAM_D6    GPIO_NUM_19
#define CAM_D7    GPIO_NUM_36

#define CAM_SCL   GPIO_NUM_23
#define CAM_SDA   GPIO_NUM_18

#define CAM_PWD   GPIO_NUM_32
#define CAM_RST   GPIO_NUM_33

extern const uint8_t watch_map[];
extern const uint8_t wx_map[];

static void cam_task(void *arg)
{
    lcd_config_t lcd_config = {
        .clk_fre = 80 * 1000 * 1000,
        .pin_clk = LCD_CLK,
        .pin_mosi = LCD_MOSI,
        .pin_dc = LCD_DC,
        .pin_cs = LCD_CS,
        .pin_rst = LCD_RST,
        .pin_bk = LCD_BK,
        .max_buffer_size = 2 * 1024,
        .horizontal = 2, // 2: UP, 3： DOWN
        .dis_invert = true,
        .dis_bgr = true
    };

    lcd_init(&lcd_config);

    // lcd_set_index(0, 0, CAM_WIDTH - 1, CAM_HIGH - 1);
    // lcd_write_data(wx_map, CAM_WIDTH * CAM_HIGH * 2);

    // vTaskDelay(3000 / portTICK_RATE_MS);

    cam_config_t cam_config = {
        .width = 8,
        .fre = 5 * 1000 * 1000,
        .pin = {
            .xclk  = CAM_XCLK,
            .pclk  = CAM_PCLK,
            .vsync = CAM_VSYNC,
            .hsync = CAM_HSYNC,
            .data = {CAM_D0, CAM_D1, CAM_D2, CAM_D3, CAM_D4, CAM_D5, CAM_D6, CAM_D7},
        },
        .invert = {
            .xclk  = false,
            .pclk  = false,
            .vsync = true,
            .hsync = false,
            .data = {false, false, false, false, false, false, false, false},
        },
        .mode.jpeg = JPEG_MODE,
        .recv_size = 16 * 1024,
        // .recv_size = CAM_WIDTH * CAM_HIGH * 2,
        .max_dma_buffer_size = 8 * 1024,
        .frame_num = 2, 
        .frame_caps = MALLOC_CAP_SPIRAM,
        .task_stack = 1024,
        .task_pri = configMAX_PRIORITIES
    };

#if JPEG_ENCODE
    uint8_t *jpeg_buf = (uint8_t *)heap_caps_malloc(CAM_WIDTH * CAM_HIGH * sizeof(uint16_t), MALLOC_CAP_SPIRAM);
#endif

    cam_init(&cam_config);

    if (CAM_PWD != -1 && CAM_RST != -1) {
        gpio_config_t io_conf;
        io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
        io_conf.mode = GPIO_MODE_OUTPUT;
        io_conf.pin_bit_mask = (1ULL << CAM_PWD) | (1ULL << CAM_RST);
        io_conf.pull_down_en = 0;
        io_conf.pull_up_en = 0;
        gpio_config(&io_conf);

        gpio_set_level(CAM_PWD, 0);
        gpio_set_level(CAM_RST, 1);
    }

    sensor_t sensor;
    SCCB_Init(CAM_SDA, CAM_SCL);
    sensor.slv_addr = SCCB_Probe();
    ESP_LOGI(TAG, "sensor_id: 0x%x\n", sensor.slv_addr);
    if (sensor.slv_addr == 0x30) { // OV2640
        if (OV2640_Init(0, 1) != 0) {
            goto fail;
        }
        if (cam_config.mode.jpeg) {
            OV2640_JPEG_Mode();
        } else {
#if YUV_MODE
            OV2640_YUV_Mode();
#else
            OV2640_RGB565_Mode(false);	//RGB565模式
#endif
        }
        
        OV2640_ImageSize_Set(800, 600);
        OV2640_ImageWin_Set(0, 0, 800, 600);
        OV2640_OutSize_Set(CAM_WIDTH, CAM_HIGH); 
    } else if (sensor.slv_addr == 0x3C) { // OV3660
        ov3660_init(&sensor);
        sensor.init_status(&sensor);
        if (sensor.reset(&sensor) != 0) {
            goto fail;
        }
        if (cam_config.mode.jpeg) {
            sensor.set_pixformat(&sensor, PIXFORMAT_JPEG);
        } else {
#if YUV_MODE
            sensor.set_pixformat(&sensor, PIXFORMAT_YUV422);
#else
            sensor.set_pixformat(&sensor, PIXFORMAT_RGB565);
#endif 
        }
        // totalX 变小，帧率提高
        // totalY 变小，帧率提高vsync 变短
        sensor.set_res_raw(&sensor, 0, 0, 2079, 1547, 8, 2, 1920, 800, CAM_WIDTH, CAM_HIGH, true, true);
        sensor.set_vflip(&sensor, 1);
        sensor.set_hmirror(&sensor, 1);
        sensor.set_pll(&sensor, false, 15, 1, 0, false, 0, true, 5); // 12fps
    } else {
        ESP_LOGE(TAG, "sensor is temporarily not supported\n");
        goto fail;
    }

    ESP_LOGI(TAG, "camera init done\n");
    vTaskDelay(1000 / portTICK_RATE_MS); // need delay for stable
    cam_start();
    uint32_t ticks_now = 0, ticks_last = 0;
    struct timeval now;
    while (1) {
        gettimeofday(&now, NULL);
        ticks_last = now.tv_sec * 1000 + now.tv_usec / 1000;
        uint8_t *cam_buf = NULL;
        size_t recv_len = cam_take(&cam_buf);
#if JPEG_MODE
        printf("total_len: %d\n", recv_len);
        // ets_printf("start: ");
        // for (int x = 0; x < 10; x++) {
        //     ets_printf("%d ", cam_buf[x]);
        // }
        // ets_printf("\nend: ");
        // for (int x = 10; x > 0; x--) {
        //     ets_printf("%d ", cam_buf[recv_len - x]);
        // }
        // ets_printf("\n");

        // int w, h;
        // uint8_t *img = jpeg_decode(cam_buf, &w, &h);
        // if (img) {
        //     ESP_LOGI(TAG, "jpeg: w: %d, h: %d\n", w, h);
        //     lcd_set_index(0, 0, w - 1, h - 1);
        //     lcd_write_data(img, w * h * sizeof(uint16_t));
        //     free(img);
        // }
#else
#if JPEG_ENCODE
        int jpeg_len = jpeg_encode(YUV_MODE ? ENCODE_YUV_MODE: ENCODE_RGB16_MODE, cam_buf, CAM_WIDTH, CAM_HIGH, jpeg_buf, CAM_WIDTH * CAM_HIGH * sizeof(uint16_t));

        printf("jpeg_len: %d\n", jpeg_len);
        for (int x = 0; x < 10; x++) {
            ets_printf("%d ", jpeg_buf[x]);
        }
        ets_printf("\n");

        int w, h;
        uint8_t *img = jpeg_decode(jpeg_buf, &w, &h);
        if (img) {
            ESP_LOGI(TAG, "jpeg: w: %d, h: %d\n", w, h);
            lcd_set_index(0, 0, w - 1, h - 1);
            lcd_write_data(img, w * h * sizeof(uint16_t));
            free(img);
        }
#else
        lcd_set_index(0, 0, CAM_WIDTH - 1, CAM_HIGH - 1);
        lcd_write_data(cam_buf, CAM_WIDTH * CAM_HIGH * 2);
#endif
#endif
        cam_give(cam_buf);   
        // 使用逻辑分析仪观察帧率
        // gpio_set_level(LCD_BK, 1);
        // gpio_set_level(LCD_BK, 0);  
        gettimeofday(&now, NULL);
        ticks_now = now.tv_sec * 1000 + now.tv_usec / 1000;
        printf("fps: %.2f\n", 1000.0 / (int)(ticks_now - ticks_last));
    }

fail:
    cam_deinit();
    vTaskDelete(NULL);
}

void app_main() 
{
    xTaskCreate(cam_task, "cam_task", 4096, NULL, 5, NULL);
}