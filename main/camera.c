#include "camera.h"

#include "esp_log.h"

static const char *TAG = "CAMERA";

bool camera_init(void)
{
    camera_config_t config = {
        .pixel_format = PIXFORMAT_JPEG,
        .frame_size = FRAMESIZE_VGA,
        .jpeg_quality = 12,
        .fb_count = 1
    };

    esp_err_t err = esp_camera_init(&config);

    return true;
}

camera_fb_t *camera_capture(void)
{
    return esp_camera_fb_get();
}

void camera_release(camera_fb_t *frame)
{
    if (frame != NULL) {
        esp_camera_fb_return(frame);
    }

}