#include "image_processing.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "sensor.h"

#include <cstddef>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define IMG_WIDTH 160
#define IMG_HEIGHT 120
#define IMG_PIXELS (IMG_WIDTH * IMG_HEIGHT)
#define HARRIS_K_DIVISOR 25
#define HARRIS_THRESHOLD_DIVISOR 10
#define MAX_CORNERS 256

typedef struct {
    int x;
    int y;
    int64_t response;
}harris_corner_t;

static const char *TAG = "IMAGE_PROCESSING";

static uint8_t *gaussian_buffer = NULL;
static int16_t *ix_buffer = NULL;
static int16_t *iy_buffer = NULL;
static int64_t *harris_response_buffer = NULL;
static uint8_t *candidate_buffer = NULL;
static uint8_t *nms_buffer = NULL;
static harris_corner_t corners[MAX_CORNERS];
static int corner_count = 0;

static esp_err_t allocate_work_buffers(void)
{
    if (gaussian_buffer != NULL) {
        return ESP_OK;
    }

    gaussian_buffer = heap_caps_malloc(IMG_PIXELS * sizeof(uint8_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    ix_buffer = heap_caps_malloc(IMG_PIXELS * sizeof( int16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    iy_buffer = heap_caps_malloc(IMG_PIXELS * sizeof(int16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    harris_response_buffer = heap_caps_malloc(IMG_PIXELS * sizeof(int64_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    candidate_buffer = heap_caps_malloc(IMG_PIXELS * sizeof(uint8_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    nms_buffer = heap_caps_malloc(IMG_PIXELS * sizeof(uint8_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

    if (gaussian_buffer == NULL || ix_buffer == NULL || iy_buffer == NULL || harris_response_buffer == NULL || candidate_buffer == NULL || nms_buffer == NULL) {
        
        ESP_LOGE(TAG, "Failed to allocate image-processing buffers in PSRAM");

        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Image-processing buffers allocated");

    return ESP_OK;
}

esp_err_t extract_y_channel(const camera_fb_t *fb, uint8_t *grayscale_buffer)
{   
    
    if (fb == NULL){
        ESP_LOGE(TAG, "Framebuffer pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    if (grayscale_buffer == NULL) {
        ESP_LOGE(TAG, "Grayscale buffer is NULL");
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

    return ESP_OK;
}

static void gaussian_3x3(const uint8_t *grayscale_buffer, uint8_t *gaussian_buffer)
{
    for (int x = 0; x < IMG_WIDTH; x++) {
        gaussian_buffer[x] = grayscale_buffer[x];
        gaussian_buffer[(IMG_HEIGHT - 1) * IMG_WIDTH + x] = grayscale_buffer[(IMG_HEIGHT - 1) * IMG_WIDTH + x];
    }

    for (int y = 0; y < IMG_HEIGHT; y++) {
        gaussian_buffer[y * IMG_WIDTH] = grayscale_buffer[y * IMG_WIDTH];
        gaussian_buffer[y * IMG_WIDTH + IMG_WIDTH - 1] = grayscale_buffer[y * IMG_WIDTH + IMG_WIDTH - 1];
    }

    for (int y = 1; y < IMG_HEIGHT - 1; y++) {
        for (int x = 1; x < IMG_WIDTH - 1; x++) {
            int idx = y * IMG_WIDTH + x;

            int sum = grayscale_buffer[idx - IMG_WIDTH - 1] + (grayscale_buffer[idx - IMG_WIDTH] << 1) + grayscale_buffer[idx - IMG_WIDTH + 1]
            + (grayscale_buffer[idx - 1] << 1) + (grayscale_buffer[idx] << 2) + (grayscale_buffer[idx + 1] << 1)
            + grayscale_buffer[idx + IMG_WIDTH - 1] + (grayscale_buffer[idx + IMG_WIDTH] << 1) + grayscale_buffer[idx + IMG_WIDTH + 1];

            gaussian_buffer[idx] = (uint8_t)(sum >> 4);
        }
    }
}

static void sobel_filter(const uint8_t *gaussian_buffer, int16_t *ix_buffer, int16_t *iy_buffer)
{
    for (int y = 1; y < IMG_HEIGHT - 1; y++) {
        for (int x = 1; x < IMG_WIDTH - 1; x++) {
            
            int idx = y * IMG_WIDTH + x;

            ix_buffer[idx] = - gaussian_buffer[idx - IMG_WIDTH - 1] + gaussian_buffer[idx - IMG_WIDTH + 1]
            - (gaussian_buffer[idx - 1] << 1) + (gaussian_buffer[idx + 1] << 1)
            - gaussian_buffer[idx + IMG_WIDTH - 1] + gaussian_buffer[idx + IMG_WIDTH + 1];

            iy_buffer[idx] = -gaussian_buffer[idx - IMG_WIDTH - 1] - (gaussian_buffer[idx - IMG_WIDTH] << 1) - gaussian_buffer[idx - IMG_WIDTH + 1]
            +gaussian_buffer[idx + IMG_WIDTH - 1] + (gaussian_buffer[idx + IMG_WIDTH] << 1) + gaussian_buffer[idx + IMG_WIDTH + 1];
        }
    }
}

static void harris_structure_tensor(const int16_t *ix_buffer, const int16_t *iy_buffer, int32_t *sum_xx, int32_t *sum_yy, int32_t *sum_xy, int x, int y)
{   
    *sum_xx = 0;
    *sum_yy = 0;
    *sum_xy = 0;

    for (int dy = -1; dy <= 1; dy++) {
        for (int dx = -1; dx<= 1; dx++) {

            int idx = (y + dy) * IMG_WIDTH + (x + dx);

            int16_t ix = ix_buffer[idx];
            int16_t iy = iy_buffer[idx];

            *sum_xx += ix * ix;
            *sum_yy += iy * iy;
            *sum_xy += ix * iy;
        }
    }
}

static int64_t harris_response(int32_t sum_xx, int32_t sum_yy, int32_t sum_xy)
{
    int64_t determinant = (int64_t)sum_xx * sum_yy - (int64_t)sum_xy * sum_xy;
    int64_t trace = (int64_t)sum_xx + sum_yy;

    int64_t response = determinant - ((trace * trace) / HARRIS_K_DIVISOR);

    return response;
}

static void calculate_harris_response_buffer(const int16_t *ix_buffer, const int16_t *iy_buffer)
{
    for (int y = 2; y < IMG_HEIGHT - 2; y++) {
        for (int x = 2; x < IMG_WIDTH - 2; x ++) {
            
            int32_t sum_xx;
            int32_t sum_yy;
            int32_t sum_xy;

            harris_structure_tensor(ix_buffer, iy_buffer, &sum_xx, &sum_yy, &sum_xy, x, y);
            int64_t response = harris_response(sum_xx, sum_yy, sum_xy);
            int idx = y * IMG_WIDTH + x;

            harris_response_buffer[idx] = response;
        }
    }
}

static void harris_threshold(const int64_t *harris_response_buffer, uint8_t *candidate_buffer)
{
    int64_t max_response = 0;

    for (int y = 2; y < IMG_HEIGHT - 2; y++) {
        for (int x = 2; x < IMG_WIDTH - 2; x++) {
            
            int idx = y * IMG_WIDTH + x;
            int64_t response = harris_response_buffer[idx];

            if (response > max_response) {
                max_response = response;
            }
        }
    }

    if (max_response <= 0) {
        memset(candidate_buffer, 0, IMG_PIXELS);
        return;
    }

    int64_t threshold = max_response / HARRIS_THRESHOLD_DIVISOR;

    for (int i = 0; i < IMG_WIDTH * IMG_HEIGHT; i++) {
        candidate_buffer[i] = 0;
    }

    for (int y = 2; y < IMG_HEIGHT - 2; y++) {
        for (int x = 2; x < IMG_WIDTH - 2; x++) {
            
            int idx = y * IMG_WIDTH + x;

            if (harris_response_buffer[idx] >= threshold) {
                candidate_buffer[idx] = 1;
            }
        }
    }
}

static void harris_nms(const int64_t *harris_response_buffer, const uint8_t *candidate_buffer, uint8_t *nms_buffer)
{
    for (int i = 0; i < IMG_WIDTH * IMG_HEIGHT; i++) {
        nms_buffer[i] = 0;
    }

    for (int y = 2; y < IMG_HEIGHT - 2; y++) {
        for (int x = 2; x < IMG_WIDTH - 2; x++) {
            
            int idx = y * IMG_WIDTH + x;

            if (candidate_buffer[idx] == 0) {
                continue;
            }

            int64_t current_response = harris_response_buffer[idx];
            int is_maximum = 1;

            for (int dy = -1; dy <= 1; dy++) {
                for (int dx = -1; dx <= 1; dx++) {
                    
                    if (dx == 0 && dy == 0) {
                        continue;
                    }
                    int neighbor_idx = (y + dy) * IMG_WIDTH + (x + dx);

                    if (candidate_buffer[neighbor_idx] == 0) {
                        continue;
                    }

                    int64_t neighbor_response = harris_response_buffer[neighbor_idx];

                    if (neighbor_response > current_response) {
                        is_maximum = 0;
                    }

                    if (neighbor_response == current_response && neighbor_idx < idx) {
                        is_maximum = 0;
                    }
                }
            }

            if (is_maximum) {
                nms_buffer[idx] = 1;
            }
        }
    }
}

static int extract_harris_corners(const uint8_t *nms_buffer, const int64_t *harris_response_buffer, harris_corner_t *corners, int max_corners)
{
    int corner_count = 0;

    for (int y = 2; y < IMG_HEIGHT - 2; y++){
        for (int x = 2; x < IMG_WIDTH - 2; x++) {
            
            int idx = y * IMG_WIDTH + x;

            if (nms_buffer[idx] == 0) {
                continue;
            }

            if (corner_count >= max_corners) {
                return corner_count;
            }

            corners[corner_count].x = x;
            corners[corner_count].y = y;
            corners[corner_count].response = harris_response_buffer[idx];

            corner_count++;
        }
    }

    return corner_count;
}

int process_harris_corners(const uint8_t *grayscale_buffer)
{
    if (grayscale_buffer == NULL) {
        return 0;
    }

    if (allocate_work_buffers() != ESP_OK) {
        return -1;
    }

    gaussian_3x3(grayscale_buffer, gaussian_buffer);
    sobel_filter(gaussian_buffer, ix_buffer, iy_buffer);
    calculate_harris_response_buffer(ix_buffer, iy_buffer);
    harris_threshold(harris_response_buffer, candidate_buffer);
    harris_nms(harris_response_buffer, candidate_buffer, nms_buffer);
    corner_count = extract_harris_corners(nms_buffer, harris_response_buffer, corners, MAX_CORNERS);

    return corner_count;
}