#include "image_processing.h"
#include "esp_err.h"
#include "esp_log.h"
#include "sensor.h"

#include <cstddef>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define IMG_WIDTH 160
#define IMG_HEIGHT 120
#define IMG_PIXELS (IMG_WIDTH * IMG_HEIGHT)

static const char *TAG = "TEST";

esp_err_t extract_y_channel(const camera_fb_t *fb, uint8_t *grayscale_buffer)
{   
    
    if (fb == NULL){
        ESP_LOGE(TAG, "Framebuffer pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    if (fb->format != PIXFORMAT_YUV422) {
        ESP_LOGE(TAG, "Unexpected pixel format: %d", fb->format);
        return ESP_ERR_INVALID_ARG;
    }

    if (fb != NULL) {

        const uint8_t *src = fb->buf;

        for (int i = 0; i < fb->len / 2; i++) {
            if (grayscale_buffer != NULL) {
                grayscale_buffer[i] = src[i * 2]; 
            }
        }
    } else {
        ESP_LOGE(TAG, "POINTER NOT FOUND");
    }
}