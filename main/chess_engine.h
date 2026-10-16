#ifndef CHESS_ENGINE_H
#define CHESS_ENGINE_H

#include "chess_board.h"
#include <stdbool.h>

typedef struct
{
    int from_row;
    int from_col;

    int to_row;
    int to_col;
} chess_move_t;

bool chess_engine_init(void);

bool chess_engine_find_move(
    const chess_position_t *position,
    chess_move_t *move
);

#endif