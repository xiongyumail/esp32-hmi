#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#define CAM_DATA_WIDTH (16)

typedef struct {
    uint8_t  width;
    uint32_t fre;
    struct {
        int8_t xclk;
        int8_t pclk;
        int8_t vsync;
        int8_t hsync;
        int8_t data[CAM_DATA_WIDTH];
    } pin;
    struct {
        bool xclk;
        bool pclk;
        bool vsync;
        bool hsync;
        bool data[CAM_DATA_WIDTH];
    } invert;
    union {
        struct {
            uint32_t jpeg:   1; 
        };
        uint32_t val;
    } mode;
    uint32_t max_dma_buffer_size; // DMA used
    uint32_t recv_size;
    uint32_t frame_num;
    uint32_t frame_caps;
    uint32_t task_stack;
    uint8_t  task_pri;
} cam_config_t;

void cam_start(void);
void cam_stop(void);
size_t cam_take(uint8_t **buffer_p);
void cam_give(uint8_t *buffer);
void cam_deinit();
int cam_init(const cam_config_t *config);

#ifdef __cplusplus
}
#endif

