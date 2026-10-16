#ifndef CAMERA_H
#define CAMERA_H
#include "esp_camera.h"


esp_err_t init_camera(void);
camera_fb_t *camera_get_frame(void);
void camera_release_frame(camera_fb_t *frame);

#endif