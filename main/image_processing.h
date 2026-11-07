#ifndef IMAGE_PROCESSING_H
#define IMAGE_PROCESSING_H

#include <stdint.h>

#include "esp_camera.h"
#include "esp_err.h"


#define CHESSBOARD_WARP_WIDTH       96
#define CHESSBOARD_WARP_HEIGHT      96

#define CHESSBOARD_SQUARE_WIDTH     12
#define CHESSBOARD_SQUARE_HEIGHT    12


esp_err_t extract_y_channel(
    const camera_fb_t *fb,
    uint8_t *grayscale_buffer
);

esp_err_t localize_chessboard(
    const uint8_t *grayscale_buffer,
    uint8_t *warped_board
);

esp_err_t extract_board_square(
    const uint8_t *warped_board,
    int row,
    int column,
    uint8_t *square_buffer
);

int image_processing_last_corner_count(void);


#endif