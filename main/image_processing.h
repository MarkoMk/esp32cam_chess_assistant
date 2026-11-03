#ifndef IMAGE_PROCESSING_H
#define IMAGE_PROCESSING_H

#include "esp_camera.h"
#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

esp_err_t extract_y_channel(const camera_fb_t *fb, uint8_t *grayscale_buffer);

#endif