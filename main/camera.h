#ifndef CAMERA_H
#define CAMERA_H

#include "esp_camera.h"
#include <stdbool.h>

bool camera_init(void);

camera_fb_t *camera_capture(void);

void camera_release(camera_fb_t *frame);

#endif