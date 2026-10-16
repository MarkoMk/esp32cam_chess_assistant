#include "chess_engine.h"
#include "chess_board.h"
#include <stdbool.h>

bool chess_engine_init(void)
{
    return true;
}

bool chess_engine_find_move(
    const chess_position_t *position,
    chess_move_t *move)
{
    if (position == NULL || move == NULL) {
        return false;
    }

    /*
    * Chess engine goes here.
    */

    return true;
}