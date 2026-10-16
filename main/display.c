#include "display.h"

#include "chess_engine.h"
#include "ssd1306.h"
#include <stdbool.h>

bool display_init(void)
{
    /*
    * Initialize I2C
    *
    * Initialize SSD1306
    *
    * Clear display
    */
    return true;
}

void display_show_move(const chess_move_t *move)
{
    if (move == NULL) {
        return;
    }

    // Make sure the move coordinates are valid chess-board coordinates.
    if (move->from_col < 0 || move->from_col > 7 ||
        move->to_col   < 0 || move->to_col   > 7 ||
        move->from_row < 0 || move->from_row > 7 ||
        move->to_row   < 0 || move->to_row   > 7) {
        return;
    }

    char move_text[8];

    int from_row = move->from_row + 1;
    int to_row   = move->to_row + 1;

    snprintf(
        move_text,
        sizeof(move_text),
        "%c%d-%c%d",
        'a' + move->from_col,
        from_row,
        'a' + move->to_col,
        to_row
    );

    display_show_text(move_text);
}

void display_show_text(const char *text)
{
    /*
    * SSD1306 drawing code
    */
}