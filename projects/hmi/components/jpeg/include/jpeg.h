#pragma once
#include <stdint.h>
#include "esp_err.h"
#include "tjpgd.h"
#include "jpegenc.h"

#define JPEG_WORK_BUF_SIZE 3100

uint8_t *jpeg_decode(uint8_t *jpeg, int *w, int* h);

size_t jpeg_encode(uint8_t *img, int w, int h, uint8_t *jpeg, size_t max_size);