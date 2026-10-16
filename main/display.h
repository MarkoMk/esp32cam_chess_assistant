#ifndef DISPLAY_H
#define DISPLAY_H

#include "chess_engine.h"
#include <stdbool.h>

bool display_init(void);

void display_show_move(
    const chess_move_t *move
);

void display_show_text(
    const char *text
);

#endif