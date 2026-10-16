#ifndef CHESS_BOARD_H
#define CHESS_BOARD_H

#include "ai_model.h"
#include <stdbool.h>

typedef struct
{
    char squares[8][8];
} chess_position_t;

bool chess_board_from_detections(
    const detection_result_t *detections,
    chess_position_t *position
);

void chess_board_print(
    const chess_position_t *position
);

#endif