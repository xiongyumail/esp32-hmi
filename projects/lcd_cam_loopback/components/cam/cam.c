#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s.h"
#include "esp_system.h"
#include "esp_log.h"
#include "soc/i2s_struct.h"
#include "soc/apb_ctrl_reg.h"
#include "esp32/rom/lldesc.h"
#include "esp32/rom/cache.h"
#include "soc/dport_access.h"
#include "soc/dport_reg.h"
#include "driver/ledc.h"
#include "cam.h"

static const char *TAG = "cam";

#define CAM_DMA_MAX_SIZE     (4095)

typedef enum {
    CAM_IN_SUC_EOF_EVENT = 0,
    CAM_VSYNC_EVENT
} cam_event_t;

typedef struct {
    uint8_t *frame_buffer;
    size_t len;
} frame_buffer_event_t;

typedef struct {
    uint32_t dma_buffer_size;
    uint32_t dma_half_buffer_size;
    uint32_t dma_node_buffer_size;
    uint32_t dma_node_num;
    uint32_t dma_half_node_num;
    uint32_t dma_buffer_num;
    lldesc_t *dma;
    uint8_t *dma_buffer;
    uint8_t *frame_buffer;
    uint8_t *frame_en;
    uint32_t frame_num;
    uint32_t recv_size;
    uint8_t jpeg_mode;
    uint8_t vsync_pin;
    uint8_t vsync_invert;
    SemaphoreHandle_t vsync_sem;
    SemaphoreHandle_t dma_sem;
    QueueHandle_t frame_buffer_queue;
    TaskHandle_t task_handle;
    intr_handle_t cam_intr_handle;
} cam_obj_t;

static cam_obj_t *cam_obj = NULL;

static void IRAM_ATTR cam_isr(void *arg)
{
    BaseType_t HPTaskAwoken = pdFALSE;
    typeof(I2S0.int_st) status = I2S0.int_st;
    if (status.val == 0) {
        return;
    }
    I2S0.int_clr.val = status.val;
    if (status.in_suc_eof) {
        xSemaphoreGiveFromISR(cam_obj->dma_sem, &HPTaskAwoken);
    }

    if(HPTaskAwoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

#include "hal/gpio_ll.h"
static void IRAM_ATTR cam_vsync_isr(void *arg)
{
    BaseType_t HPTaskAwoken = pdFALSE;
    // filter
    ets_delay_us(1);
    if (gpio_ll_get_level(&GPIO, cam_obj->vsync_pin) == !cam_obj->vsync_invert) {
        xSemaphoreGiveFromISR(cam_obj->vsync_sem, &HPTaskAwoken);
    }

    if(HPTaskAwoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static void cam_vsync_intr_enable(uint8_t en)
{
    if (en) {
        gpio_intr_enable(cam_obj->vsync_pin);
    } else {
        gpio_intr_disable(cam_obj->vsync_pin);
    }
}

static void cam_dma_stop(void)
{
    if (I2S0.int_ena.in_suc_eof == 1) {
        I2S0.conf.rx_start = 0;
        I2S0.int_ena.in_suc_eof = 0;
        I2S0.int_clr.in_suc_eof = 1;
        I2S0.in_link.stop = 1;
    }
}

static void cam_dma_start(void)
{
    if (I2S0.int_ena.in_suc_eof == 0) {
        I2S0.conf.rx_start = 0;
        I2S0.int_clr.in_suc_eof = 1;
        I2S0.int_ena.in_suc_eof = 1;
        I2S0.conf.rx_reset = 1;
        I2S0.conf.rx_reset = 0;
        I2S0.conf.rx_fifo_reset = 1;
        I2S0.conf.rx_fifo_reset = 0;
        I2S0.lc_conf.in_rst = 1;
        I2S0.lc_conf.in_rst = 0;
        I2S0.lc_conf.ahbm_fifo_rst = 1;
        I2S0.lc_conf.ahbm_fifo_rst = 0;
        I2S0.lc_conf.ahbm_rst = 1;
        I2S0.lc_conf.ahbm_rst = 0;
        I2S0.in_link.start = 1;
        I2S0.conf.rx_start = 1;
    }
}

void cam_stop(void)
{
    cam_vsync_intr_enable(0);
    cam_dma_stop();
}

void cam_start(void)
{
    cam_vsync_intr_enable(1);
}

typedef enum {
    CAM_STATE_IDLE = 0,
    CAM_STATE_READ_BUF = 1,
} cam_state_t;

uint8_t jpeg_buf[10 * 1024];

#if 1
//Copy fram from DMA buffer to fram buffer
static void cam_task(void *arg)
{
    int len = 0;
    int dma_cnt = 0;
    int frame_cnt = 0;
    uint8_t char0 = 0;
    uint8_t char1 = 0;
    uint8_t *in_buf = NULL;
    uint8_t *out_buf = NULL;
    int state = CAM_STATE_IDLE;
    frame_buffer_event_t frame_buffer_event = {0};
    cam_dma_start();
    while (1) {
            xSemaphoreTake(cam_obj->dma_sem, portMAX_DELAY);
            memcpy(jpeg_buf, &cam_obj->dma_buffer[(dma_cnt % 2) * cam_obj->dma_half_buffer_size], cam_obj->dma_half_buffer_size);
            in_buf = jpeg_buf;
            // in_buf = &cam_obj->dma_buffer[(dma_cnt % 2) * cam_obj->dma_half_buffer_size];
            for (int x = -2; x < (int)(cam_obj->dma_half_buffer_size - 2); x+=2) {
                if (x == -2) {
                    char0 = char1;
                    char1 = in_buf[3];
                } else {
                    if (x % 4 == 0) {
                        uint8_t temp = in_buf[x + 1];
                        in_buf[x + 1] = in_buf[x + 3];
                        in_buf[x + 3] = temp;
                    }
                    char0 = in_buf[x + 1];
                    char1 = in_buf[x + 3];
                }
                switch (state) {
                    case 0: {
                        if (char0 == 255) {
                            if (char1 == 216) {
                                for (int y = 0; y < cam_obj->frame_num; y++) {
                                    if (cam_obj->frame_en[y]) {
                                        state = 1;
                                        frame_cnt = y;
                                        len = 0;
                                        out_buf = &cam_obj->frame_buffer[frame_cnt * cam_obj->recv_size];
                                        out_buf[len++] = char0;
                                    }
                                }
                            }
                        }
                    }
                    break;

                    case 1: {
                        if (len >= cam_obj->recv_size - 1) {
                            state = 0;
                            break;
                        }
                        out_buf[len++] = char0;
                        if (char0 == 255) {
                            if (char1 == 217) {
                                state = 0;
                                out_buf[len++] = char1;
                                frame_buffer_event.frame_buffer = &cam_obj->frame_buffer[frame_cnt * cam_obj->recv_size];
                                frame_buffer_event.len = len;
                                if (xQueueSend(cam_obj->frame_buffer_queue, (void *)&frame_buffer_event, 0) == pdTRUE) {
                                    cam_obj->frame_en[frame_cnt] = 0;
                                }
                            } else if (char1 == 216) {// strange buffer
                                len = 0;
                                out_buf[len++] = char0;
                            } 
                        }
                    }
                    break;
                }
            }
            dma_cnt++;
            dma_cnt = dma_cnt % cam_obj->dma_buffer_num;
    }
}
#else
//Copy fram from DMA buffer to fram buffer
static void cam_task(void *arg)
{
    int dma_cnt = 0;
    int frame_cnt = 0;
    uint8_t *in_buf = NULL;
    uint8_t *out_buf = NULL;
    int state = CAM_STATE_IDLE;
    cam_event_t cam_event = {0};
    frame_buffer_event_t frame_buffer_event = {0};
    xQueueReset(cam_obj->event_queue);
    xSemaphoreTake(cam_obj->vsync_sem, 0);
    xSemaphoreTake(cam_obj->dma_sem, 0);
    while (1) {
        switch (state) {
            case CAM_STATE_IDLE: {
                for (int x = 0; x < cam_obj->frame_num; x++) {
                    if (cam_obj->frame_en[x]) {
                        frame_cnt = x;
                        xSemaphoreTake(cam_obj->vsync_sem, portMAX_DELAY);
                        cam_dma_start(); 
                        state = CAM_STATE_READ_BUF;
                        break;
                    }
                }
                dma_cnt = 0;
            }
            break;

            case CAM_STATE_READ_BUF: {
                xSemaphoreTake(cam_obj->dma_sem, portMAX_DELAY);
                if (dma_cnt == 0) {
                    cam_vsync_intr_enable(1); // 需要cam真正start接收到第一个buf数据再打开vsync中断
                }
                out_buf = &cam_obj->frame_buffer[frame_cnt * cam_obj->recv_size + dma_cnt * (cam_obj->dma_half_buffer_size >> 1)];
                in_buf = &cam_obj->dma_buffer[(dma_cnt % 2) * cam_obj->dma_half_buffer_size];
                for (int x = 0; x < cam_obj->dma_half_buffer_size; x += 4) {
                    out_buf[(x >> 1)] = in_buf[x + 3];
                    out_buf[(x >> 1) + 1] = in_buf[x + 1];
                }

                if (dma_cnt == cam_obj->dma_buffer_num - 1) {
                    frame_buffer_event.frame_buffer = &cam_obj->frame_buffer[frame_cnt * cam_obj->recv_size];
                    frame_buffer_event.len = (dma_cnt + 1) * (cam_obj->dma_half_buffer_size >> 1);
                    if (xQueueSend(cam_obj->frame_buffer_queue, (void *)&frame_buffer_event, 0) == pdTRUE) {
                        cam_obj->frame_en[frame_cnt] = 0;
                    }
                    state = CAM_STATE_IDLE;
                } else {
                    dma_cnt++;
                }
            }
            break;
        }
    }
}
#endif

size_t cam_take(uint8_t **buffer_p)
{
    frame_buffer_event_t frame_buffer_event;
    xQueueReceive(cam_obj->frame_buffer_queue, (void *)&frame_buffer_event, portMAX_DELAY);
    *buffer_p = frame_buffer_event.frame_buffer;
    return frame_buffer_event.len;
}

void cam_give(uint8_t *buffer)
{
    int frame_cnt = (buffer - &cam_obj->frame_buffer[0]) / cam_obj->recv_size;
    if (frame_cnt < cam_obj->frame_num) {
        cam_obj->frame_en[frame_cnt] = 1;
    }
}

static void cam_config(cam_config_t *config)
{
    //Enable I2S periph
    periph_module_enable(PERIPH_I2S0_MODULE);

    // 配置时钟
    I2S0.clkm_conf.clkm_div_num = 2;
    I2S0.clkm_conf.clkm_div_b = 0;
    I2S0.clkm_conf.clkm_div_a = 0;
    I2S0.clkm_conf.clk_en = 1;

    // 配置采样率
    I2S0.sample_rate_conf.rx_bck_div_num = 1;
    I2S0.sample_rate_conf.rx_bits_mod = (config->width == 8) ? 0 : 1;

    // 配置数据格式
    I2S0.conf.rx_start = 0;
    I2S0.conf.rx_reset = 1;
    I2S0.conf.rx_reset = 0;
    I2S0.conf.rx_fifo_reset = 1;
    I2S0.conf.rx_fifo_reset = 0;
    I2S0.conf.rx_slave_mod = 1;
    I2S0.conf.rx_right_first = 0;
    I2S0.conf.rx_msb_right = 0;
    I2S0.conf.rx_short_sync = 0;
    I2S0.conf.rx_mono = 0;
    I2S0.conf.rx_msb_shift = 0;

    I2S0.conf1.rx_pcm_bypass = 1;

    I2S0.conf2.lcd_en = 1;
    I2S0.conf2.camera_en = 1;

    I2S0.conf_chan.rx_chan_mod = 1;

    I2S0.fifo_conf.rx_fifo_mod_force_en = 1;
    I2S0.fifo_conf.rx_data_num = 32;
    I2S0.fifo_conf.rx_fifo_mod = 1;
    I2S0.fifo_conf.dscr_en = 1;

    I2S0.lc_conf.in_rst  = 1;
    I2S0.lc_conf.in_rst  = 0;
    I2S0.lc_conf.ahbm_fifo_rst = 1;
    I2S0.lc_conf.ahbm_fifo_rst = 0;
    I2S0.lc_conf.ahbm_rst = 1;
    I2S0.lc_conf.ahbm_rst = 0;
    I2S0.lc_conf.check_owner = 0;

    I2S0.int_ena.val = 0;
    I2S0.int_clr.val = ~0;
    I2S0.conf.rx_start = 1;
}

static void cam_set_pin(cam_config_t *config)
{
    gpio_config_t io_conf = {0};
    io_conf.intr_type = config->invert.vsync ? GPIO_PIN_INTR_NEGEDGE : GPIO_PIN_INTR_POSEDGE;
    io_conf.pin_bit_mask = 1 << config->pin.vsync; 
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    io_conf.pull_down_en = 0;
    gpio_config(&io_conf);
    gpio_install_isr_service(ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_IRAM);
    gpio_isr_handler_add(config->pin.vsync, cam_vsync_isr, NULL);
    gpio_intr_disable(config->pin.vsync);

    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[config->pin.pclk], PIN_FUNC_GPIO);
    gpio_set_direction(config->pin.pclk, GPIO_MODE_INPUT);
    gpio_set_pull_mode(config->pin.pclk, GPIO_FLOATING);
    gpio_matrix_in(config->pin.pclk, I2S0I_WS_IN_IDX, config->invert.pclk);

    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[config->pin.vsync], PIN_FUNC_GPIO);
    gpio_set_direction(config->pin.vsync, GPIO_MODE_INPUT);
    gpio_set_pull_mode(config->pin.vsync, GPIO_FLOATING);
    gpio_matrix_in(config->pin.vsync, I2S0I_V_SYNC_IDX, !config->invert.vsync);

    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[config->pin.hsync], PIN_FUNC_GPIO);
    gpio_set_direction(config->pin.hsync, GPIO_MODE_INPUT);
    gpio_set_pull_mode(config->pin.hsync, GPIO_FLOATING);
    gpio_matrix_in(config->pin.hsync, I2S0I_H_SYNC_IDX, config->invert.hsync);

    for(int i = 0; i < config->width; i++) {
        PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[config->pin.data[i]], PIN_FUNC_GPIO);
        gpio_set_direction(config->pin.data[i], GPIO_MODE_INPUT);
        gpio_set_pull_mode(config->pin.data[i], GPIO_FLOATING);
        // 高位对齐，IN16总是最高位
        // fifo按bit来访问数据，rx_bits_mod为8时，数据需要按8位对齐
        gpio_matrix_in(config->pin.data[i], I2S0I_DATA_IN0_IDX + (16 - config->width) + i, config->invert.data[i]);
    }

    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_1_BIT,
        .freq_hz = config->fre,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_1
    };
    ledc_timer_config(&ledc_timer);
    ledc_channel_config_t ledc_channel = {
        .channel    = LEDC_CHANNEL_2,
        .duty       = 1,
        .gpio_num   = config->pin.xclk,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel  = LEDC_TIMER_1,
        .hpoint     = 0
    };
    ledc_channel_config(&ledc_channel);

    gpio_matrix_in(0x38, I2S0I_H_ENABLE_IDX, false);
    ESP_LOGI(TAG, "cam_xclk_pin setup\n");
}

void cam_dma_config(cam_config_t *config) 
{
    int dma_cnt = 0;
    uint32_t recv_size = cam_obj->recv_size * 2; // ESP32 4byte data-> 2byte valid data 
    for (dma_cnt = 0;;dma_cnt++) { // 寻找可以整除的buffer大小
        if (recv_size % (config->max_dma_buffer_size - dma_cnt) == 0) {
            break;
        }
    }
    cam_obj->dma_buffer_size = config->max_dma_buffer_size - dma_cnt;

    cam_obj->dma_half_buffer_size = cam_obj->dma_buffer_size / 2;
    for (dma_cnt = 0;;dma_cnt++) { // 寻找可以整除的dma大小
        if ((cam_obj->dma_half_buffer_size) % (CAM_DMA_MAX_SIZE - dma_cnt) == 0) {
            break;
        }
    }
    cam_obj->dma_node_buffer_size = CAM_DMA_MAX_SIZE - dma_cnt;

    cam_obj->dma_node_num = (cam_obj->dma_buffer_size) / cam_obj->dma_node_buffer_size; // DMA节点个数
    cam_obj->dma_half_node_num = cam_obj->dma_node_num / 2;
    cam_obj->dma_buffer_num = recv_size / cam_obj->dma_half_buffer_size; // 产生中断拷贝的次数, 乒乓拷贝

    ESP_LOGI(TAG, "dma_buffer_size: %d, dma_node_buffer_size: %d, dma_node_num: %d, dma_buffer_num: %d\n", cam_obj->dma_buffer_size, cam_obj->dma_node_buffer_size, cam_obj->dma_node_num, cam_obj->dma_buffer_num);

    cam_obj->dma    = (lldesc_t *)heap_caps_malloc(cam_obj->dma_node_num * sizeof(lldesc_t), MALLOC_CAP_DMA);
    cam_obj->dma_buffer = (uint8_t *)heap_caps_malloc(cam_obj->dma_buffer_size * sizeof(uint8_t), MALLOC_CAP_DMA);

    for (int x = 0; x < cam_obj->dma_node_num; x++) {
        cam_obj->dma[x].size = cam_obj->dma_node_buffer_size;
        cam_obj->dma[x].length = cam_obj->dma_node_buffer_size;
        cam_obj->dma[x].eof = 0;
        cam_obj->dma[x].owner = 1;
        cam_obj->dma[x].buf = (cam_obj->dma_buffer + cam_obj->dma_node_buffer_size * x);
        cam_obj->dma[x].empty = &cam_obj->dma[(x + 1) % cam_obj->dma_node_num];
    }

    I2S0.in_link.addr = ((uint32_t)&cam_obj->dma[0]) & 0xfffff;
    I2S0.rx_eof_num = cam_obj->dma_half_buffer_size / 4; // 乒乓操作, ESP32 4Byte
}

void cam_deinit()
{
    if (!cam_obj) {
        return;
    }
    cam_stop();
    esp_intr_free(cam_obj->cam_intr_handle);
    vTaskDelete(cam_obj->task_handle);
    vQueueDelete(cam_obj->frame_buffer_queue);
    free(cam_obj->dma);
    free(cam_obj->dma_buffer);
    free(cam_obj->frame_buffer);
    free(cam_obj->frame_en);
    free(cam_obj);
}

int cam_init(const cam_config_t *config)
{
    cam_obj = (cam_obj_t *)heap_caps_calloc(1, sizeof(cam_obj_t), MALLOC_CAP_DMA);
    if (!cam_obj) {
        ESP_LOGI(TAG, "camera object malloc error\n");
        return -1;
    }
    cam_obj->jpeg_mode = config->mode.jpeg;
    cam_obj->frame_num = config->frame_num;
    cam_obj->recv_size = config->recv_size;
    cam_obj->vsync_pin = config->pin.vsync;
    cam_obj->vsync_invert = config->invert.vsync;
    cam_set_pin(config);
    cam_config(config);
    cam_dma_config(config);

    cam_obj->vsync_sem = xSemaphoreCreateBinary();
    cam_obj->dma_sem = xSemaphoreCreateBinary();
    cam_obj->frame_buffer_queue = xQueueCreate(cam_obj->frame_num, sizeof(frame_buffer_event_t));
    cam_obj->frame_buffer = (uint8_t *)heap_caps_malloc(cam_obj->frame_num * cam_obj->recv_size * sizeof(uint8_t), config->frame_caps);
    cam_obj->frame_en = (uint8_t *)heap_caps_malloc(cam_obj->frame_num * sizeof(uint8_t), MALLOC_CAP_DEFAULT);

    for (int x = 0; x < cam_obj->frame_num; x++) {
        cam_obj->frame_en[x] = 1;
    }
    esp_intr_alloc(ETS_I2S0_INTR_SOURCE, ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_IRAM, cam_isr, NULL, &cam_obj->cam_intr_handle);
    xTaskCreate(cam_task, "cam_task", config->task_stack, NULL, config->task_pri, &cam_obj->task_handle);
    return 0;
}