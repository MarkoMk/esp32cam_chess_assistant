#include "ai_model.h"

#include "esp_log.h"
#include "image_processing.h"
#include <stdbool.h>

static const char *TAG = "AI";

bool ai_model_init(void)
{
    /*
    * Load model
    *
    * Initialize neural-network runtime
    *
    * Allocate tensors
    *
    * Prepare inference
    */

    return true;
}

bool ai_model_detect(
    const processed_image_t *image
    , detection_result_t *result)
    {
        if (image == NULL || result == NULL) {
            return false;
        }

        /*
        * Copy image into input tensor
        *
        * run inference
        * Parse neural-network output
        * Fill result -> detections []
        */

        return true;
    }