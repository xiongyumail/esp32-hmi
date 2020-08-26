#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"
#include "esp32/rom/lldesc.h"
#include "soc/dport_access.h"
#include "soc/dport_reg.h"
#include "esp_log.h"
#include "lcd.h"

static const char *TAG = "lcd";

#define LCD_DMA_MAX_SIZE     (4095)

typedef struct {
    uint32_t buffer_size;
    uint32_t half_buffer_size;
    uint32_t node_cnt;
    uint32_t half_node_cnt;
    uint32_t dma_size;
    uint8_t horizontal;
    uint8_t dc_state;
    int8_t pin_dc;
    int8_t pin_cs;
    int8_t pin_rst;
    int8_t pin_bk;
    lldesc_t *dma;
    uint8_t *buffer;
    QueueHandle_t event_queue;
} lcd_obj_t;

static lcd_obj_t *lcd_obj = NULL;

void inline lcd_set_rst(uint8_t state)
{
    if (lcd_obj->pin_rst < 0) {
        return;
    }
    gpio_set_level(lcd_obj->pin_rst, state);
}

void inline lcd_set_dc(uint8_t state)
{
    gpio_set_level(lcd_obj->pin_dc, state);
}

void inline lcd_set_cs(uint8_t state)
{
    gpio_set_level(lcd_obj->pin_cs, state);
}

void inline lcd_set_blk(uint8_t state)
{
    if (lcd_obj->pin_bk < 0) {
        return;
    }
    gpio_set_level(lcd_obj->pin_bk, state);
}

static void IRAM_ATTR lcd_isr(void *arg)
{
    int event;
    BaseType_t HPTaskAwoken = pdFALSE;
    typeof(SPI3.dma_int_st) int_st = SPI3.dma_int_st;
    SPI3.dma_int_clr.val = int_st.val;
    // ets_printf("intr: 0x%x\n", int_st);

    if (int_st.out_eof) {
        xQueueSendFromISR(lcd_obj->event_queue, &int_st.val, &HPTaskAwoken);
    }

    if (HPTaskAwoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static void spi_write_data(uint8_t *data, size_t len)
{
    int event  = 0;
    int x = 0, cnt = 0, size = 0;
    int end_pos = 0;
    lcd_set_dc(lcd_obj->dc_state);
    // 生成一段数据DMA链表
    for (x = 0; x < lcd_obj->node_cnt; x++) {
        lcd_obj->dma[x].size = lcd_obj->dma_size;
        lcd_obj->dma[x].length = lcd_obj->dma_size;
        lcd_obj->dma[x].buf = (lcd_obj->buffer + lcd_obj->dma_size * x);
        lcd_obj->dma[x].eof = !((x + 1) % lcd_obj->half_node_cnt);
        lcd_obj->dma[x].empty = &lcd_obj->dma[(x + 1) % lcd_obj->node_cnt];
    }
    lcd_obj->dma[lcd_obj->half_node_cnt - 1].empty = NULL;
    lcd_obj->dma[lcd_obj->node_cnt - 1].empty = NULL;
    cnt = len / lcd_obj->half_buffer_size;
    // 启动信号
    xQueueSend(lcd_obj->event_queue, &event, 0);
    // 处理完整一段数据， 乒乓操作
    for (x = 0; x < cnt; x++) {
        memcpy(lcd_obj->dma[(x % 2) * lcd_obj->half_node_cnt].buf, data, lcd_obj->half_buffer_size);
        data += lcd_obj->half_buffer_size;
        xQueueReceive(lcd_obj->event_queue, (void *)&event, portMAX_DELAY);
        SPI3.mosi_dlen.usr_mosi_dbitlen = lcd_obj->half_buffer_size * 8 - 1;
        SPI3.dma_out_link.addr = ((uint32_t)&lcd_obj->dma[(x % 2) * lcd_obj->half_node_cnt]) & 0xfffff;
        SPI3.dma_out_link.start = 1;
        ets_delay_us(1);
        SPI3.cmd.usr = 1;
    }
    cnt = len % lcd_obj->half_buffer_size;
    // 处理剩余非完整段数据
    if (cnt) {
        memcpy(lcd_obj->dma[(x % 2) * lcd_obj->half_node_cnt].buf, data, cnt);
        // 处理数据长度为 lcd_obj->dma_size 的整数倍情况
        if (cnt % lcd_obj->dma_size) {
            end_pos = (x % 2) * lcd_obj->half_node_cnt + cnt / lcd_obj->dma_size;
            size = cnt % lcd_obj->dma_size;
        } else {
            end_pos = (x % 2) * lcd_obj->half_node_cnt + cnt / lcd_obj->dma_size - 1;
            size = lcd_obj->dma_size;
        }
        // 处理尾节点，使其成为 DMA 尾
        lcd_obj->dma[end_pos].size = size;
        lcd_obj->dma[end_pos].length = size;
        lcd_obj->dma[end_pos].eof = 1;
        lcd_obj->dma[end_pos].empty = NULL;
        xQueueReceive(lcd_obj->event_queue, (void *)&event, portMAX_DELAY);
        SPI3.mosi_dlen.usr_mosi_dbitlen = cnt * 8 - 1;
        SPI3.dma_out_link.addr = ((uint32_t)&lcd_obj->dma[(x % 2) * lcd_obj->half_node_cnt]) & 0xfffff;
        SPI3.dma_out_link.start = 1;
        ets_delay_us(1);
        SPI3.cmd.usr = 1;
    }
    xQueueReceive(lcd_obj->event_queue, (void *)&event, portMAX_DELAY);
}

static void lcd_delay_ms(uint32_t time)
{
    vTaskDelay(time / portTICK_RATE_MS);
}

static void lcd_write_cmd(uint8_t data)
{
    lcd_obj->dc_state = 0;
    spi_write_data(&data, 1);
}

static void lcd_write_byte(uint8_t data)
{
    lcd_obj->dc_state = 1;
    spi_write_data(&data, 1);
}

void lcd_write_data(uint8_t *data, size_t len)
{
    if (len <= 0) {
        return;
    }
    lcd_obj->dc_state = 1;
    spi_write_data(data, len);
}

void lcd_rst()
{
    lcd_set_rst(0);
    lcd_delay_ms(100);
    lcd_set_rst(1);
    lcd_delay_ms(100);
}

static void lcd_gc9a01_config(lcd_config_t *config)
{
    lcd_set_cs(0);

    uint8_t bgr = (config->dis_bgr ? 0x08 : 0x0);

    lcd_write_cmd(0xEF);
    lcd_write_cmd(0xEB);
    lcd_write_byte(0x14); 

    lcd_write_cmd(0xFE);             
    lcd_write_cmd(0xEF); 

    lcd_write_cmd(0xEB);    
    lcd_write_byte(0x14); 

    lcd_write_cmd(0x84);            
    lcd_write_byte(0x40); 

    lcd_write_cmd(0x85);            
    lcd_write_byte(0xFF); 

    lcd_write_cmd(0x86);            
    lcd_write_byte(0xFF); 

    lcd_write_cmd(0x87);            
    lcd_write_byte(0xFF);

    lcd_write_cmd(0x88);            
    lcd_write_byte(0x0A);

    lcd_write_cmd(0x89);            
    lcd_write_byte(0x21); 

    lcd_write_cmd(0x8A);            
    lcd_write_byte(0x00); 

    lcd_write_cmd(0x8B);            
    lcd_write_byte(0x80); 

    lcd_write_cmd(0x8C);            
    lcd_write_byte(0x01); 

    lcd_write_cmd(0x8D);            
    lcd_write_byte(0x01); 

    lcd_write_cmd(0x8E);            
    lcd_write_byte(0xFF); 

    lcd_write_cmd(0x8F);            
    lcd_write_byte(0xFF); 


    lcd_write_cmd(0xB6);
    lcd_write_byte(0x00);
    lcd_write_byte(0x20);

    lcd_write_cmd(0x36);
    switch (config->horizontal) {
        case 0: {
            lcd_write_byte(0x00 | bgr);
        }
        break;

        case 1: {
            lcd_write_byte(0xC0 | bgr);
        }
        break;

        case 2: {
            lcd_write_byte(0x60 | bgr);
        }
        break;

        case 3: {
            lcd_write_byte(0xA0 | bgr);
        }
        break;

        default: {
            lcd_write_byte(0x00 | bgr);
        }
        break;
    }

    lcd_write_cmd(0x3A);            
    lcd_write_byte(0x05); 


    lcd_write_cmd(0x90);            
    lcd_write_byte(0x08);
    lcd_write_byte(0x08);
    lcd_write_byte(0x08);
    lcd_write_byte(0x08); 

    lcd_write_cmd(0xBD);            
    lcd_write_byte(0x06);

    lcd_write_cmd(0xBC);            
    lcd_write_byte(0x00);    

    lcd_write_cmd(0xFF);            
    lcd_write_byte(0x60);
    lcd_write_byte(0x01);
    lcd_write_byte(0x04);

    lcd_write_cmd(0xC3);            
    lcd_write_byte(0x13);
    lcd_write_cmd(0xC4);            
    lcd_write_byte(0x13);

    lcd_write_cmd(0xC9);            
    lcd_write_byte(0x22);

    lcd_write_cmd(0xBE);            
    lcd_write_byte(0x11); 

    lcd_write_cmd(0xE1);            
    lcd_write_byte(0x10);
    lcd_write_byte(0x0E);

    lcd_write_cmd(0xDF);            
    lcd_write_byte(0x21);
    lcd_write_byte(0x0c);
    lcd_write_byte(0x02);

    lcd_write_cmd(0xF0);   
    lcd_write_byte(0x45);
    lcd_write_byte(0x09);
    lcd_write_byte(0x08);
    lcd_write_byte(0x08);
    lcd_write_byte(0x26);
    lcd_write_byte(0x2A);

    lcd_write_cmd(0xF1);    
    lcd_write_byte(0x43);
    lcd_write_byte(0x70);
    lcd_write_byte(0x72);
    lcd_write_byte(0x36);
    lcd_write_byte(0x37);  
    lcd_write_byte(0x6F);


    lcd_write_cmd(0xF2);   
    lcd_write_byte(0x45);
    lcd_write_byte(0x09);
    lcd_write_byte(0x08);
    lcd_write_byte(0x08);
    lcd_write_byte(0x26);
    lcd_write_byte(0x2A);

    lcd_write_cmd(0xF3);   
    lcd_write_byte(0x43);
    lcd_write_byte(0x70);
    lcd_write_byte(0x72);
    lcd_write_byte(0x36);
    lcd_write_byte(0x37); 
    lcd_write_byte(0x6F);

    lcd_write_cmd(0xED);    
    lcd_write_byte(0x1B); 
    lcd_write_byte(0x0B); 

    lcd_write_cmd(0xAE);            
    lcd_write_byte(0x77);

    lcd_write_cmd(0xCD);            
    lcd_write_byte(0x63);        


    lcd_write_cmd(0x70);            
    lcd_write_byte(0x07);
    lcd_write_byte(0x07);
    lcd_write_byte(0x04);
    lcd_write_byte(0x0E); 
    lcd_write_byte(0x0F); 
    lcd_write_byte(0x09);
    lcd_write_byte(0x07);
    lcd_write_byte(0x08);
    lcd_write_byte(0x03);

    lcd_write_cmd(0xE8);            
    lcd_write_byte(0x34);

    lcd_write_cmd(0x62);            
    lcd_write_byte(0x18);
    lcd_write_byte(0x0D);
    lcd_write_byte(0x71);
    lcd_write_byte(0xED);
    lcd_write_byte(0x70); 
    lcd_write_byte(0x70);
    lcd_write_byte(0x18);
    lcd_write_byte(0x0F);
    lcd_write_byte(0x71);
    lcd_write_byte(0xEF);
    lcd_write_byte(0x70); 
    lcd_write_byte(0x70);

    lcd_write_cmd(0x63);            
    lcd_write_byte(0x18);
    lcd_write_byte(0x11);
    lcd_write_byte(0x71);
    lcd_write_byte(0xF1);
    lcd_write_byte(0x70); 
    lcd_write_byte(0x70);
    lcd_write_byte(0x18);
    lcd_write_byte(0x13);
    lcd_write_byte(0x71);
    lcd_write_byte(0xF3);
    lcd_write_byte(0x70); 
    lcd_write_byte(0x70);

    lcd_write_cmd(0x64);            
    lcd_write_byte(0x28);
    lcd_write_byte(0x29);
    lcd_write_byte(0xF1);
    lcd_write_byte(0x01);
    lcd_write_byte(0xF1);
    lcd_write_byte(0x00);
    lcd_write_byte(0x07);

    lcd_write_cmd(0x66);            
    lcd_write_byte(0x3C);
    lcd_write_byte(0x00);
    lcd_write_byte(0xCD);
    lcd_write_byte(0x67);
    lcd_write_byte(0x45);
    lcd_write_byte(0x45);
    lcd_write_byte(0x10);
    lcd_write_byte(0x00);
    lcd_write_byte(0x00);
    lcd_write_byte(0x00);

    lcd_write_cmd(0x67);            
    lcd_write_byte(0x00);
    lcd_write_byte(0x3C);
    lcd_write_byte(0x00);
    lcd_write_byte(0x00);
    lcd_write_byte(0x00);
    lcd_write_byte(0x01);
    lcd_write_byte(0x54);
    lcd_write_byte(0x10);
    lcd_write_byte(0x32);
    lcd_write_byte(0x98);

    lcd_write_cmd(0x74);            
    lcd_write_byte(0x10);    
    lcd_write_byte(0x85);    
    lcd_write_byte(0x80);
    lcd_write_byte(0x00); 
    lcd_write_byte(0x00); 
    lcd_write_byte(0x4E);
    lcd_write_byte(0x00);                    

    lcd_write_cmd(0x98);            
    lcd_write_byte(0x3e);
    lcd_write_byte(0x07);

    lcd_write_cmd(0x35);    
    lcd_write_cmd(config->dis_invert ? 0x21 : 0x20);

    lcd_write_cmd(0x11);
    lcd_delay_ms(120);
    lcd_write_cmd(0x29);
    lcd_delay_ms(20);
}

static void lcd_config(lcd_config_t *config)
{

    DPORT_REG_CLR_BIT(DPORT_PERIP_CLK_EN_REG, DPORT_SPI3_CLK_EN);
    DPORT_REG_SET_BIT(DPORT_PERIP_CLK_EN_REG, DPORT_SPI3_CLK_EN);
    DPORT_REG_SET_BIT(DPORT_PERIP_RST_EN_REG, DPORT_SPI3_RST);
    DPORT_REG_CLR_BIT(DPORT_PERIP_RST_EN_REG, DPORT_SPI3_RST);
    DPORT_REG_CLR_BIT(DPORT_PERIP_CLK_EN_REG, DPORT_SPI_DMA_CLK_EN);
    DPORT_REG_SET_BIT(DPORT_PERIP_CLK_EN_REG, DPORT_SPI_DMA_CLK_EN);
    DPORT_REG_SET_BIT(DPORT_PERIP_RST_EN_REG, DPORT_SPI_DMA_RST);
    DPORT_REG_CLR_BIT(DPORT_PERIP_RST_EN_REG, DPORT_SPI_DMA_RST);
    DPORT_SET_PERI_REG_BITS(DPORT_SPI_DMA_CHAN_SEL_REG, DPORT_SPI3_DMA_CHAN_SEL, 1, DPORT_SPI3_DMA_CHAN_SEL_S); // ESP32 SPI DMA Channel config, channel 0, SPI3

    int div = 2;
    if (config->clk_fre == 80000000) {
        SPI3.clock.clk_equ_sysclk = 1;
    } else {
        SPI3.clock.clk_equ_sysclk = 0;
        div = 80000000 / config->clk_fre;
    }
    SPI3.clock.clkdiv_pre = 1 - 1;
    SPI3.clock.clkcnt_n = div - 1;
    SPI3.clock.clkcnt_l = div - 1;
    SPI3.clock.clkcnt_h = ((div >> 1) - 1);
    
    SPI3.pin.ck_dis = 0;

    SPI3.user1.val = 0;
    SPI3.slave.val = 0;
    SPI3.pin.ck_idle_edge = 0;
    SPI3.user.ck_out_edge = 0;
    SPI3.ctrl.wr_bit_order = 0;
    SPI3.ctrl.rd_bit_order = 0;
    SPI3.user.val = 0;
    SPI3.user.cs_setup = 1;
    SPI3.user.cs_hold = 1;
    SPI3.user.usr_mosi = 1;
    SPI3.user.usr_mosi_highpart = 0;

    SPI3.dma_conf.val = 0;
    SPI3.dma_conf.out_rst = 1;
    SPI3.dma_conf.out_rst = 0;
    SPI3.dma_conf.ahbm_fifo_rst = 1;
    SPI3.dma_conf.ahbm_fifo_rst = 0;
    SPI3.dma_conf.ahbm_rst = 1;
    SPI3.dma_conf.ahbm_rst = 0;
    SPI3.dma_conf.out_eof_mode = 1;
    SPI3.cmd.usr = 0;

    SPI3.dma_int_clr.val = ~0;
    SPI3.dma_int_ena.out_eof = 1;

    intr_handle_t intr_handle = NULL;
    esp_intr_alloc(ETS_SPI3_DMA_INTR_SOURCE, 0, lcd_isr, NULL, &intr_handle);
    esp_intr_enable(intr_handle);
}

static void lcd_set_pin(lcd_config_t *config)
{
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[config->pin_clk], PIN_FUNC_GPIO);
    gpio_set_direction(config->pin_clk, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(config->pin_clk, GPIO_FLOATING);
    gpio_matrix_out(config->pin_clk, VSPICLK_OUT_IDX, 0, 0);

    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[config->pin_mosi], PIN_FUNC_GPIO);
    gpio_set_direction(config->pin_mosi, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(config->pin_mosi, GPIO_FLOATING);
    gpio_matrix_out(config->pin_mosi, VSPID_OUT_IDX, 0, 0);

    //Initialize non-SPI GPIOs
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << config->pin_dc) | (1ULL << config->pin_cs);
    io_conf.pin_bit_mask |= (config->pin_rst < 0) ? 0ULL : (1ULL << config->pin_rst);
    io_conf.pin_bit_mask |= (config->pin_bk < 0) ? 0ULL : (1ULL << config->pin_bk);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
}

void lcd_dma_config(lcd_config_t *config) 
{
    int cnt = 0;
    if (config->max_buffer_size >= LCD_DMA_MAX_SIZE * 2) {
        lcd_obj->dma_size = LCD_DMA_MAX_SIZE;
        for (cnt = 0;;cnt++) { // 寻找可以整除dma_size的buffer大小
            if ((config->max_buffer_size - cnt) % (lcd_obj->dma_size * 2) == 0) {
                break;
            }
        }
        lcd_obj->buffer_size = config->max_buffer_size - cnt;
    } else {
        lcd_obj->dma_size = config->max_buffer_size / 2;
        lcd_obj->buffer_size = lcd_obj->dma_size * 2;
    }
    
    lcd_obj->half_buffer_size = lcd_obj->buffer_size / 2;

    lcd_obj->node_cnt = (lcd_obj->buffer_size) / lcd_obj->dma_size; // DMA节点个数
    lcd_obj->half_node_cnt = lcd_obj->node_cnt / 2;

    ESP_LOGI(TAG, "lcd_buffer_size: %d, lcd_dma_size: %d, lcd_dma_node_cnt: %d\n", lcd_obj->buffer_size, lcd_obj->dma_size, lcd_obj->node_cnt);

    lcd_obj->dma    = (lldesc_t *)heap_caps_malloc(lcd_obj->node_cnt * sizeof(lldesc_t), MALLOC_CAP_DMA);
    lcd_obj->buffer = (uint8_t *)heap_caps_malloc(lcd_obj->buffer_size * sizeof(uint8_t), MALLOC_CAP_DMA);
}

int lcd_init(lcd_config_t *config)
{
    lcd_obj = (lcd_obj_t *)heap_caps_calloc(1, sizeof(lcd_obj_t), MALLOC_CAP_DMA);
    if (!lcd_obj) {
        ESP_LOGI(TAG, "lcd object malloc error\n");
        return -1;
    }

    lcd_set_pin(config);
    lcd_config(config);
    lcd_dma_config(config);

    lcd_obj->event_queue = xQueueCreate(1, sizeof(int));
    
    lcd_obj->buffer_size = config->max_buffer_size;

    lcd_obj->pin_dc = config->pin_dc;
    lcd_obj->pin_cs = config->pin_cs;
    lcd_obj->pin_rst = config->pin_rst;
    lcd_obj->pin_bk = config->pin_bk;
    lcd_set_cs(1);

    lcd_rst();//lcd_rst before LCD Init.
    lcd_delay_ms(100);
    lcd_gc9a01_config(config);

    lcd_set_blk(0);
    ESP_LOGI(TAG, "lcd init ok\n");

    return 0;
}

void lcd_set_index(uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end)
{
    uint16_t start_pos, end_pos;
    lcd_write_cmd(0x2a);    // CASET (2Ah): Column Address Set 
    // Must write byte than byte
    start_pos = x_start;
    end_pos = x_end;
    lcd_write_byte(start_pos >> 8);
    lcd_write_byte(start_pos & 0xFF);
    lcd_write_byte(end_pos >> 8);
    lcd_write_byte(end_pos & 0xFF);

    lcd_write_cmd(0x2b);    // RASET (2Bh): Row Address Set
    start_pos = y_start;
    end_pos = y_end;
    lcd_write_byte(start_pos >> 8);
    lcd_write_byte(start_pos & 0xFF);
    lcd_write_byte(end_pos >> 8);
    lcd_write_byte(end_pos & 0xFF); 
    lcd_write_cmd(0x2c);    // RAMWR (2Ch): Memory Write 
}