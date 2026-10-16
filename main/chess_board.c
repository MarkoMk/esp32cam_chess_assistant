#include "chess_board.h"
#include "ai_model.h"

#include <string.h>
#include <stdio.h>

bool chess_board_from_detections(
    const detection_result_t *detections,
    chess_position_t *position)
{
    if (detections == NULL || position == NULL) {
        return false;
    }

    memset(position, 0, sizeof(chess_position_t));

    for (int i =0; i < detections->count; i++) 
    {
        const detection_t *d = &detections->detections[i];
        /*
        * Convert detection coordinates to chess square.
        *
        * this will also use the detected board geometry.
        */

        int row = 0;
        int col = 0;

        // TODO:
        // convert class_id -> chess piece

        position->squares[row][col] = 'P';
    }
    
    return true;
}

void chess_board_print(
    const chess_position_t *position)
{
    for (int row = 0; row < 8; row++) 
    {
        printf("%d", 8 - row);

        for (int col = 0; col < 8; col++)
        {
            char piece = position->squares[row][col];

            if (piece == 0) {
                printf(".");
            } else {
            printf("%c", piece);
}   
        }
        printf("\n");
    }
    printf(" a b c d e f g h\n");
}