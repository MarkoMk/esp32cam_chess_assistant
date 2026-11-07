#include <stdio.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"

#include "camera.h"
#include "image_processing.h"


#define IMG_WIDTH               160
#define IMG_HEIGHT              120
#define IMG_PIXELS              (IMG_WIDTH * IMG_HEIGHT)

#define WARP_WIDTH              96
#define WARP_HEIGHT             96
#define WARP_PIXELS             (WARP_WIDTH * WARP_HEIGHT)


static const char *TAG = "MAIN_TEST";


/* ============================================================
 * ASCII PREVIEW
 *
 * This prints a coarse representation of the warped chessboard
 * directly into the ESP-IDF serial monitor.
 *
 * 96x96 is reduced to 24x24 by averaging 4x4 blocks.
 * ============================================================ */

static void print_warped_board_ascii(const uint8_t *warped_board)
{
    static const char brightness_chars[] =
        " .:-=+*#%@";

    const int character_count =
        sizeof(brightness_chars) - 1;

    const int block_size = 4;


    printf("\n");
    printf("========== WARPED CHESSBOARD ==========\n\n");


    for (int y = 0; y < WARP_HEIGHT; y += block_size) {

        for (int x = 0; x < WARP_WIDTH; x += block_size) {

            int sum = 0;
            int count = 0;


            /*
             * Average one 4x4 block.
             */
            for (int dy = 0; dy < block_size; dy++) {

                for (int dx = 0; dx < block_size; dx++) {

                    int px =
                        x + dx;

                    int py =
                        y + dy;


                    if (px >= WARP_WIDTH ||
                        py >= WARP_HEIGHT) {

                        continue;
                    }


                    sum += warped_board[
                        py * WARP_WIDTH + px
                    ];


                    count++;
                }
            }


            uint8_t average =
                (uint8_t)(sum / count);


            int character_index =
                average *
                (character_count - 1) /
                255;


            putchar(
                brightness_chars[
                    character_index
                ]
            );
        }


        putchar('\n');
    }


    printf("\n");
    printf("=======================================\n");
    printf("\n");
}


/* ============================================================
 * MAIN
 * ============================================================ */

void app_main(void)
{
    ESP_LOGI(
        TAG,
        "Starting chessboard localization test"
    );


    /* --------------------------------------------------------
     * 1. Initialize camera
     * -------------------------------------------------------- */

    esp_err_t err =
        init_camera();


    if (err != ESP_OK) {

        ESP_LOGE(
            TAG,
            "Camera initialization failed: %s",
            esp_err_to_name(err)
        );

        return;
    }


    ESP_LOGI(
        TAG,
        "Camera initialized successfully"
    );


    /* --------------------------------------------------------
     * 2. Allocate grayscale image
     *
     * 160 x 120 = 19,200 bytes
     * -------------------------------------------------------- */

    uint8_t *grayscale_buffer =
        heap_caps_malloc(
            IMG_PIXELS * sizeof(uint8_t),
            MALLOC_CAP_SPIRAM |
            MALLOC_CAP_8BIT
        );


    if (grayscale_buffer == NULL) {

        ESP_LOGE(
            TAG,
            "Failed to allocate grayscale buffer"
        );

        return;
    }


    /* --------------------------------------------------------
     * 3. Allocate normalized board
     *
     * 96 x 96 = 9,216 bytes
     * -------------------------------------------------------- */

    uint8_t *warped_board =
        heap_caps_malloc(
            WARP_PIXELS * sizeof(uint8_t),
            MALLOC_CAP_SPIRAM |
            MALLOC_CAP_8BIT
        );


    if (warped_board == NULL) {

        ESP_LOGE(
            TAG,
            "Failed to allocate warped-board buffer"
        );


        heap_caps_free(
            grayscale_buffer
        );


        return;
    }


    ESP_LOGI(
        TAG,
        "Test buffers allocated"
    );


    /* --------------------------------------------------------
     * CONTINUOUS TEST LOOP
     * -------------------------------------------------------- */

    while (1) {

        ESP_LOGI(
            TAG,
            "Capturing frame..."
        );


        /* ----------------------------------------------------
         * 4. Capture image
         * ---------------------------------------------------- */

        camera_fb_t *frame =
            camera_get_frame();


        if (frame == NULL) {

            ESP_LOGE(
                TAG,
                "Camera capture failed"
            );


            vTaskDelay(
                pdMS_TO_TICKS(2000)
            );


            continue;
        }


        ESP_LOGI(
            TAG,
            "Frame: %ux%u, %u bytes, format=%d",
            frame->width,
            frame->height,
            (unsigned int)frame->len,
            frame->format
        );


        /* ----------------------------------------------------
         * 5. Extract grayscale Y channel
         * ---------------------------------------------------- */

        err =
            extract_y_channel(
                frame,
                grayscale_buffer
            );


        /*
         * IMPORTANT:
         *
         * We are finished with the camera framebuffer after
         * Y extraction, so return it immediately.
         *
         * The DSP pipeline now works from grayscale_buffer.
         */
        camera_release_frame(
            frame
        );


        frame = NULL;


        if (err != ESP_OK) {

            ESP_LOGE(
                TAG,
                "Y extraction failed: %s",
                esp_err_to_name(err)
            );


            vTaskDelay(
                pdMS_TO_TICKS(2000)
            );


            continue;
        }


        ESP_LOGI(
            TAG,
            "Y extraction successful"
        );


        /* ----------------------------------------------------
         * 6. Run complete chessboard localization
         * ---------------------------------------------------- */

        ESP_LOGI(
            TAG,
            "Running chessboard localization..."
        );


        err =
            localize_chessboard(
                grayscale_buffer,
                warped_board
            );


        if (err != ESP_OK) {

            ESP_LOGW(
                TAG,
                "Chessboard localization FAILED"
            );


            ESP_LOGI(
                TAG,
                "Harris corners detected: %d",
                image_processing_last_corner_count()
            );


            vTaskDelay(
                pdMS_TO_TICKS(3000)
            );


            continue;
        }


        /* ----------------------------------------------------
         * 7. Success
         * ---------------------------------------------------- */

        ESP_LOGI(
            TAG,
            "CHESSBOARD LOCALIZATION SUCCESS"
        );


        ESP_LOGI(
            TAG,
            "Harris corners detected: %d",
            image_processing_last_corner_count()
        );


        ESP_LOGI(
            TAG,
            "Warped board size: %dx%d",
            WARP_WIDTH,
            WARP_HEIGHT
        );


        /*
         * Print a 24x24 ASCII preview.
         */
        print_warped_board_ascii(
            warped_board
        );


        /* ----------------------------------------------------
         * Wait before next capture
         * ---------------------------------------------------- */

        vTaskDelay(
            pdMS_TO_TICKS(3000)
        );
    }
}