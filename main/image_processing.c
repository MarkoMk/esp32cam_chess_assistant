#include "image_processing.h"

#include <stdbool.h>
#include <string.h>

bool image_process(
    camera_fb_t *frame
    , processed_image_t *output)
    {
        if (frame == NULL || output == NULL) {
            return false;
        }

        /*
        * 1. JPEG -> RGB
        
        * 2. RESIZE IMAGE
        
        * 3. NORMALIZE PIXELS
        
        * 4. CONVERT TO Neural Network FORMAT
        
        *         
        */
        
        return true;
}