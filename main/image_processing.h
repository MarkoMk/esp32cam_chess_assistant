#ifndef IMAGE_PROCESSING_H
#define IMAGE_PROCESSING_H

#include "esp_camera.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct 
{
    uint8_t *data;

    int width;
    int height;
    int channels;
} processed_image_t;

bool image_process(
    camera_fb_t *frame,
    processed_image_t *output
);

#endif