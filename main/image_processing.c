#include "image_processing.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "sensor.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define IMG_WIDTH                   160
#define IMG_HEIGHT                  120
#define IMG_PIXELS                  (IMG_WIDTH * IMG_HEIGHT)

#define WARP_WIDTH                  96
#define WARP_HEIGHT                 96
#define WARP_PIXELS                 (WARP_WIDTH * WARP_HEIGHT)

#define CHESS_SQUARES               8
#define SQUARE_SIZE                 12

#define HARRIS_K_DIVISOR            25
#define HARRIS_THRESHOLD_DIVISOR    10
#define MAX_CORNERS                 256

#define INNER_GRID_SIZE             7
#define FULL_GRID_SIZE              9

#define ANGLE_BINS                  36

#define PI_F                        3.14159265358979323846f

#define MIN_PAIR_DISTANCE_SQ        16.0f
#define MAX_PAIR_DISTANCE_SQ        2500.0f

#define MIN_AXIS_SEPARATION         0.70f
#define AXIS_AVERAGE_WINDOW         0.18f

#define GRID_CLUSTER_TOLERANCE      0.75f

#define MIN_GRID_CORRESPONDENCES    16
#define MIN_GRID_ROWS               5
#define MIN_GRID_COLS               5

#define MIN_BOARD_AREA              500.0f

static const char *TAG = "IMAGE_PROCESSING";

typedef struct {
    int x;
    int y;
    int64_t response;
}harris_corner_t;

typedef struct {
    float x;
    float y;
} point2f_t;

typedef struct {
    float ux;
    float uy;

    float vx;
    float vy;
} grid_axes_t;

typedef struct {
    bool valid;

    point2f_t point;

    int64_t response;
} grid_cell_t;

static uint8_t *gaussian_buffer = NULL;

static int16_t *ix_buffer = NULL;
static int16_t *iy_buffer = NULL;

static int64_t *harris_response_buffer = NULL;

static uint8_t *candidate_buffer = NULL;
static uint8_t *nms_buffer = NULL;

static harris_corner_t corners[MAX_CORNERS];
static int corner_count = 0;

static float projection_u[MAX_CORNERS];
static float projection_v[MAX_CORNERS];

static float sort_buffer[MAX_CORNERS];

static void free_work_buffers(void)
{
    if (gaussian_buffer != NULL) {
        heap_caps_free(gaussian_buffer);
        gaussian_buffer = NULL;
    }

    if (ix_buffer != NULL) {
        heap_caps_free(ix_buffer);
        ix_buffer = NULL;
    }

    if (iy_buffer != NULL) {
        heap_caps_free(iy_buffer);
        iy_buffer = NULL;
    }

    if (harris_response_buffer != NULL) {
        heap_caps_free(harris_response_buffer);
        harris_response_buffer = NULL;
    }

    if (candidate_buffer != NULL) {
        heap_caps_free(candidate_buffer);
        candidate_buffer = NULL;
    }

    if (nms_buffer != NULL) {
        heap_caps_free(nms_buffer);
        nms_buffer = NULL;
    }
}

static esp_err_t allocate_work_buffers(void)
{

    if (gaussian_buffer != NULL || ix_buffer != NULL || iy_buffer != NULL || harris_response_buffer != NULL || candidate_buffer != NULL || nms_buffer != NULL) {
        
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

        free_work_buffers();
        
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Image processing buffers allocated in PSRAM");

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
    int count = 0;

    for (int y = 2; y < IMG_HEIGHT - 2; y++){
        for (int x = 2; x < IMG_WIDTH - 2; x++) {
            
            int idx = y * IMG_WIDTH + x;

            if (nms_buffer[idx] == 0) {
                continue;
            }

            if (count >= max_corners) {
                return count;
            }

            corners[count].x = x;
            corners[count].y = y;
            corners[count].response = harris_response_buffer[idx];

            count++;
        }
    }

    return count;
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

static float orientation_difference(float a, float b)
{
    float difference = fabsf(a - b);

    if (difference > PI_F) {
        
        difference = fmodf(difference, PI_F);
    }

    if (difference > PI_F * 0.5f) {
        
        difference = PI_F - difference;
    }

    return fabsf(difference);
}

static bool estimate_grid_axes(const harris_corner_t *input_corners, int count, grid_axes_t *axes)
{
    if (input_corners == NULL || axes == NULL || count < 8) {
        return false;
    }

    int histogram[ANGLE_BINS];

    memset(histogram, 0, sizeof(histogram));

    for (int i = 0; i < count; i++) {
        for (int j = i + 1; j < count; j++) {
            
            float dx = (float)(input_corners[j].x - input_corners[i].x);
            float dy = (float)(input_corners[j].y - input_corners[i].y);

            float distance_sq = dx * dx + dy * dy;

            if (distance_sq < MIN_PAIR_DISTANCE_SQ || distance_sq > MAX_PAIR_DISTANCE_SQ) {
                continue;
            }

            float angle = atan2f(dy, dx);

            if (angle < 0.0f) {
                angle += PI_F;
            }

            if (angle >= PI_F) {
                angle -= PI_F;
            }
            
            int bin = (int)(angle * ANGLE_BINS / PI_F);

            if (bin < 0) {
                bin = 0;
            }

            if (bin >= ANGLE_BINS) {
                bin = ANGLE_BINS -1;
            }

            histogram[bin]++;
        }
    }

    int first_bin = -1;
    int first_count = 0;

    for (int i = 0; i < ANGLE_BINS; i++) {
        if (histogram[i] > first_count) {
            
            first_count = histogram[i];
            first_bin = i;
        }
    }

    if (first_bin < 0 || first_count == 0) {
        return false;
    }

    float first_angle = ((float)first_bin + 0.5f) * PI_F / ANGLE_BINS;

    int second_bin = -1;
    int second_count = 0;

    for (int i = 0; i < ANGLE_BINS; i++) {
        
        float angle = ((float)i + 0.5f) * PI_F / ANGLE_BINS;

        float difference = orientation_difference(angle, first_angle);

        if (difference < MIN_AXIS_SEPARATION) {
            continue;
        }

        if (histogram[i] > second_count) {
            
            second_count = histogram[i];

            second_bin = i;
        }
    }

    if (second_bin < 0 || second_count == 0) {
        return false;
    }

    float second_angle = ((float)second_bin + 0.5f) * PI_F / ANGLE_BINS;

    float first_ref_x = 0.0f;
    float first_ref_y = 0.0f;

    float second_ref_x = 0.0f;
    float second_ref_y = 0.0f;

    int first_samples = 0;
    int second_samples = 0;

    float first_base_x = cosf(first_angle);
    float first_base_y = sinf(first_angle);

    float second_base_x = cosf(second_angle);
    float second_base_y = sinf(second_angle);
    
    for (int i = 0; i < count; i++) {
        for (int j = i + 1; j < count; j++) {
            
            float dx = (float)(input_corners[j].x - input_corners[i].x);
            float dy = (float)(input_corners[j].y - input_corners[i].y);

            float distance_sq = dx * dx + dy * dy;

            if (distance_sq < MIN_PAIR_DISTANCE_SQ || distance_sq > MAX_PAIR_DISTANCE_SQ) {
                continue;
            }

            float length = sqrtf(distance_sq);

            if (length <= 0.0f) {
                continue;
            }

            float nx = dx / length;
            float ny = dy / length;

            float angle = atan2f(dy, dx);

            if (angle < 0.0f) {
                angle += PI_F;
            }

            if (orientation_difference(angle, first_angle) <= AXIS_AVERAGE_WINDOW) {
                
                if (nx * first_base_x + ny * first_base_y < 0.0f){

                    nx = -nx;
                    ny = -ny;
                }

                first_ref_x += nx;
                first_ref_y += ny;

                first_samples++;
            }

            if (orientation_difference(angle, second_angle) <= AXIS_AVERAGE_WINDOW) {
                
                if (nx * second_base_x + ny * second_base_y < 0.0f) {
                    
                    nx = -nx;
                    ny = -ny;
                }

                second_ref_x += nx;
                second_ref_y += ny;

                second_samples++;
            }
        }
    }

    if (first_samples == 0 || second_samples == 0) {
        return false;
    }

    float first_length = sqrtf(first_ref_x * first_ref_x + first_ref_y * first_ref_y);
    float second_length = sqrtf(second_ref_x * second_ref_x + second_ref_y *second_ref_y);

    if (first_length < 0.001f || second_length < 0.001f) {
        return false;
    }

    axes->ux = first_ref_x / first_length;
    axes->uy = first_ref_y / first_length;
    axes->vx = second_ref_x / second_length;
    axes->vy = second_ref_y / second_length;

    return true;
}

static void sort_float_values(float *values, int count)
{
    for (int i = 1; i < count; i++) {
        
        float key = values[i];

        int j = i - 1;

        while (j >= 0 && values[j] > key) {
            
            values[j + 1] = values[j];

            j--;
        }

        values[j + 1] = key;
    }
}

static bool cluster_seven_lines(const float *values, int count, float centers[INNER_GRID_SIZE])
{
    if (values == NULL || centers == NULL || count < INNER_GRID_SIZE) {
        return false;
    }

    for (int i = 0; i < count; i++) {
        
        sort_buffer[i] = values[i];
    }

    sort_float_values(sort_buffer, count);

    int trim = 0;

    if (count >= 40) {
        
        trim = count / 20;
    }

    float low = sort_buffer[trim];

    float high = sort_buffer[count - 1 - trim];

    if (high - low < 6.0f) {
        return false;
    }

    for (int i = 0; i < INNER_GRID_SIZE; i++) {
        
        centers[i] = low + (high - low) * (float)i / (float)(INNER_GRID_SIZE - 1);
    }

    for (int iteration = 0; iteration < 16; iteration++) {
        
        float sums[INNER_GRID_SIZE] = {0};
        int counts[INNER_GRID_SIZE] = {0};

        for (int i = 0; i < count; i++) {
            
            int best = 0;
            float best_distance = fabsf(values[i] - centers[0]);

            for (int k = 1; k < INNER_GRID_SIZE; k++) {
                
                float distance = fabsf(values[i] - centers[k]);

                if (distance < best_distance) {
                    
                    best_distance = distance;
                    best = k;
                }
            }

            sums[best] += values[i];

            counts[best]++;
        }

        for (int k = 0; k < INNER_GRID_SIZE; k++) {
            
            if (counts[k] > 0) {
                
                centers[k] = sums[k] / counts[k];
            }
        }
    }
    
    sort_float_values(centers, INNER_GRID_SIZE);

    for (int i = 0; i < INNER_GRID_SIZE - 1; i++) {
        
        if (centers[i + 1] - centers[i] < 2.0f) {
            
            return false;
        }
    }

    return true;
}

static int nearest_center(float value, const float centers[INNER_GRID_SIZE], float *distance_out)
{
    int best = 0;

    float best_distance = fabsf(value - centers[0]);

    for (int i = 1; i < INNER_GRID_SIZE; i++) {
        
        float distance = fabsf(value - centers[i]);

        if (distance < best_distance) {
            
            best_distance = distance;

            best = i;
        }
    }

    if (distance_out != NULL) {
        
        *distance_out = best_distance;
    }

    return best;
}

static int build_internal_grid(const harris_corner_t *input_corners, int count, const grid_axes_t *axes, grid_cell_t cells[INNER_GRID_SIZE][INNER_GRID_SIZE])
{
    memset(cells, 0, sizeof(grid_cell_t) * INNER_GRID_SIZE * INNER_GRID_SIZE);

    for (int i = 0; i < count; i++) {
        
        projection_u[i] = input_corners[i].x * axes->ux + input_corners[i].y * axes->uy;
        projection_v[i] = input_corners[i].x * axes->vx + input_corners[i].y * axes->vy;
    }

    float centers_u[INNER_GRID_SIZE];
    float centers_v[INNER_GRID_SIZE];

    if (!cluster_seven_lines(projection_u, count, centers_u)) {
        return 0;
    }

    if (!cluster_seven_lines(projection_v, count, centers_v)) {
        return 0;
    }

    float spacing_u = 0.0f;
    float spacing_v = 0.0f;

    for (int i = 0; i < INNER_GRID_SIZE - 1; i++) {
        
        spacing_u += centers_u[i + 1] - centers_u[i];
        spacing_v += centers_v[i + 1] - centers_v[i];
    }

    spacing_u /= (INNER_GRID_SIZE - 1);
    spacing_v /= (INNER_GRID_SIZE - 1);

    if (spacing_u <= 0.0f || spacing_v <= 0.0f) {
        return 0;
    }

    for (int i = 0; i < count; i++) {
        
        float distance_u;
        float distance_v;

        int column = nearest_center(projection_u[i], centers_u, &distance_u);
        int row = nearest_center(projection_v[i], centers_v, &distance_v);

        if (distance_u > spacing_u * GRID_CLUSTER_TOLERANCE || distance_v > spacing_v * GRID_CLUSTER_TOLERANCE) {
            continue;
        }

        grid_cell_t *cell = &cells[row][column];

        if (!cell->valid || input_corners[i].response > cell->response) {
            
            cell->valid = true;

            cell->point.x = (float)input_corners[i].x;
            cell->point.y = (float)input_corners[i].y;

            cell->response = input_corners[i].response;
        }
    }

    int valid_count = 0;

    int valid_rows = 0;
    int valid_columns = 0;

    for (int row = 0; row < INNER_GRID_SIZE; row++) {
        
        bool row_has_point = false;

        for (int column = 0; column < INNER_GRID_SIZE; column++) {
            
            if (cells[row][column].valid) {
                
                valid_count++;

                row_has_point = true;
            }
        }

        if (row_has_point) {
            valid_rows++;
        }
    }

    for (int column = 0; column < INNER_GRID_SIZE; column++) {
        
        bool column_has_point = false;

        for (int row = 0; row < INNER_GRID_SIZE; row++) {
            
            if (cells[row][column].valid) {
                
                column_has_point = true;

                break;
            }
        }

        if (column_has_point) {
            valid_columns++;
        }
    }
    
    if (valid_count < MIN_GRID_CORRESPONDENCES || valid_rows < MIN_GRID_ROWS || valid_columns < MIN_GRID_COLS) {
        
        return 0;
    }

    return valid_count;
}

static bool solve_8x8(float matrix[8][8], float vector[8], float solution[8])
{
    for (int column = 0; column < 8; column++) {
        
        int pivot = column;

        float pivot_value = fabsf(matrix[column][column]);

        for (int row = column + 1; row < 8; row++) {
            
            float value = fabsf(matrix[row][column]);

            if (value > pivot_value) {
                
                pivot = row;
                pivot_value = value;
            }
        }

        if (pivot_value < 1e-6f) {
            
            return false;
        }

        if (pivot != column) {
            
            for (int k = 0; k < 8; k++) {
                
                float temp = matrix[column][k];
                matrix[column][k] = matrix[pivot][k];

                matrix[pivot][k] = temp;
            }

            float temp = vector[column];
            vector[column] = vector[pivot];
            
            vector[pivot] = temp;
        }

        float divisor = matrix[column][column];

        for (int k = column; k < 8; k++) {
            
            matrix[column][k] /= divisor;
        }

        vector[column] /= divisor;

        for (int row = 0; row < 8; row++) {
            
            if (row == column) {
                
                continue;
            }

            float factor = matrix[row][column];

            for (int k = column; k < 8; k++) {
                
                matrix[row][k] -= factor * matrix[column][k];
            }

            vector[row] -= factor * vector[column];
        }
    }

    for (int i = 0; i < 8; i++) {
        
        solution[i] = vector[i];
    }

    return true;
}

static void accumulate_equation(const float row[8], float target, float ata[8][8], float atb[8])
{

    for (int i = 0; i < 8; i++) {    
        
        atb[i] += row[i] * target;

        for (int j = 0; j < 8; j++) {
            
            ata[i][j] += row[i] * row[j];
        }
    }
}

static bool fit_grid_homography(const grid_cell_t cells[INNER_GRID_SIZE][INNER_GRID_SIZE], float h[8])
{
    float ata[8][8] = {{0}};
    float atb[8] = {0};

    int used_points = 0;

    for (int row = 0; row < INNER_GRID_SIZE; row++) {
        for (int column = 0; column < INNER_GRID_SIZE; column++) {

            if (!cells[row][column].valid) {
                continue;
            }

            float X = (float)(column + 1);
            float Y = (float)(row + 1);

            float x = cells[row][column].point.x;
            float y = cells[row][column].point.y;

            float equation_x[8] = {
                X,
                Y,
                1.0f,

                0.0f,
                0.0f,
                0.0f,

                -x * X,
                -x * Y
            };

            float equation_y[8] = {
                0.0f,
                0.0f,
                0.0f,

                X,
                Y,
                1.0f,

                -y * X,
                -y * Y
            };

            accumulate_equation(equation_x, x, ata, atb);

            accumulate_equation(equation_y, y, ata, atb);

            used_points++;
        }
    }

    if (used_points < MIN_GRID_CORRESPONDENCES) {
        return false;
    }

    return solve_8x8(ata, atb, h);
}

static bool apply_homography(const float h[8], float X, float Y, point2f_t *output)
{

    float denominator = h[6] * X + h[7] * Y + 1.0f;

    if (fabsf(denominator) < 1e-6f) {
        
        return false;
    }

    output->x = (h[0] * X + h[1] * Y + h[2]) / denominator;
    output->y = (h[3] * X + h[4] * Y + h[5]) / denominator;

    if (!isfinite(output->x) || !isfinite(output->y)) {
        
        return false;
    }

    return true;
}

static bool reconstruct_full_grid(const float h[8], point2f_t grid[FULL_GRID_SIZE][FULL_GRID_SIZE])
{
    for (int row = 0; row < FULL_GRID_SIZE; row++) {
        for (int column = 0; column < FULL_GRID_SIZE; column++) {
            
            if (!apply_homography(h, (float)column, (float)row, &grid[row][column])) {
                
                return false;
            }


            if (grid[row][column].x < -IMG_WIDTH * 0.25f || grid[row][column].x > IMG_WIDTH * 1.25f || grid[row][column].y < -IMG_HEIGHT * 0.25f || grid[row][column].y > IMG_HEIGHT * 1.25f) {
        
                return false;
            }
        }

    }

    point2f_t p0 = grid[0][0];
    point2f_t p1 = grid[0][8];
    point2f_t p2 = grid[8][8];
    point2f_t p3 = grid[8][0];

    float twice_area = p0.x * p1.y - p1.x * p0.y
                    + p1.x * p2.y - p2.x * p1.y
                    + p2.x * p3.y - p3.x * p2.y
                    + p3.x * p0.y - p0.x * p3.y;

    float area = fabsf(twice_area) * 0.5f;

    if (area < MIN_BOARD_AREA) {

        return false;
    }

    return true;
}

static uint8_t bilinear_sample(const uint8_t *image, float x, float y)
{
    if (x < 0.0f || y < 0.0f || x > (float)(IMG_WIDTH - 1) || y > (float)(IMG_HEIGHT - 1)) {
        
        return 0;
    }

    int x0 = (int)floorf(x);
    int y0 = (int)floorf(y);
    int x1 = x0 + 1;
    int y1 = y0 + 1;

    if (x1 >= IMG_WIDTH) {
        
        x1 = IMG_WIDTH - 1;
    }

    if (y1 >= IMG_HEIGHT) {
        
        y1 = IMG_HEIGHT - 1;
    }

    float fx = x - (float)x0;
    float fy = y - (float)y0;
    float p00 = image[y0 * IMG_WIDTH + x0];
    float p10 = image[y0 * IMG_WIDTH + x1];
    float p01 = image[y1 * IMG_WIDTH + x0];
    float p11 = image[y1 * IMG_WIDTH + x1];

    float top = p00 + fx * (p10 - p00);
    float bottom = p01 + fx * (p11 - p01);
    float value = top + fy * (bottom - top);

    if (value < 0.0f) {
        value = 0.0f;
    }

    if (value > 255.0f) {
        value = 255.0f;
    }

    return (uint8_t)(value + 0.5f);
}

static bool warp_chessboard(const uint8_t *grayscale_buffer, const float h[8], uint8_t *warped_board)
{
    if (grayscale_buffer == NULL || warped_board == NULL) {
    
        return false;
    }

    for (int output_y = 0; output_y < WARP_HEIGHT; output_y++) {
        
        float board_y = 8.0f * (float)output_y / (float)(WARP_HEIGHT - 1);

        for (int output_x = 0; output_x < WARP_WIDTH; output_x++) {
            
            float board_x = 8.0f * (float)output_x / (float)(WARP_WIDTH - 1);

            point2f_t source;

            if (!apply_homography(h, board_x, board_y, &source)) {
                
                return false;
            }

            warped_board[output_y * WARP_WIDTH + output_x] = bilinear_sample(grayscale_buffer, source.x, source.y);
        }

    }

    return true;
}

esp_err_t localize_chessboard(const uint8_t *grayscale_buffer, uint8_t *warped_board)
{
    if (grayscale_buffer == NULL || warped_board == NULL) {
        
        return ESP_ERR_INVALID_ARG;
    }

    int detected_corners = process_harris_corners(grayscale_buffer);

    if (detected_corners < 8) {
        
        ESP_LOGW(TAG, "Not enough Harris corners: %d", detected_corners);

        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Harris corners: %d", detected_corners);

    grid_axes_t axes;

    if (!estimate_grid_axes(corners, detected_corners, &axes)) {
        
        ESP_LOGW(TAG, "Grid-axis estimation failed");

        return ESP_FAIL;
    }

    grid_cell_t cells [INNER_GRID_SIZE][INNER_GRID_SIZE];

    int grid_points = build_internal_grid(corners, detected_corners, &axes, cells);

    if (grid_points < MIN_GRID_CORRESPONDENCES) {
        
        ESP_LOGW(TAG, "Grid reconstruction failed: %d points", grid_points);

        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Grid correspondences: %d", grid_points);

    float homography[8];

    if (!fit_grid_homography(cells, homography)) {
        
        ESP_LOGW(TAG, "Homography fit failed");

        return ESP_FAIL;
    }

    point2f_t full_grid [FULL_GRID_SIZE][FULL_GRID_SIZE];

    if (!reconstruct_full_grid(homography, full_grid)) {
        
        ESP_LOGW(TAG, "9x9 chess grid reconstruction failed");

        return ESP_FAIL;
    }

    if (!warp_chessboard(grayscale_buffer, homography, warped_board)) {
        
        ESP_LOGW(TAG, "Chessboard warp failed");

        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Chessboard localized and warped to %dx%d", WARP_WIDTH, WARP_HEIGHT);

    return ESP_OK;
}

esp_err_t extract_board_square(const uint8_t *warped_board, int row, int column, uint8_t *square_buffer)
{
    if (warped_board == NULL || square_buffer == NULL) {
        
        return ESP_ERR_INVALID_ARG;
    }

    if (row < 0 || row >= CHESS_SQUARES || column < 0 || column >= CHESS_SQUARES) {
        
        return ESP_ERR_INVALID_ARG;
    }

    int start_x = column * SQUARE_SIZE;
    int start_y = row * SQUARE_SIZE;

    for (int y = 0; y < SQUARE_SIZE; y++) {
        for (int x = 0; x < SQUARE_SIZE; x++) {
            
            square_buffer[y * SQUARE_SIZE + x] = warped_board[(start_y + y) * WARP_WIDTH + (start_x + x)];
        }
    }

    return ESP_OK;
}