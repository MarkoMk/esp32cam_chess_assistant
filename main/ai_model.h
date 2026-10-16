#ifndef AI_MODEL_H
#define AI_MODEL_H

#include "image_processing.h"
#include <stdbool.h>

#define MAX_DETECTIONS 32

typedef struct
{
    int class_id;

    float confidence;

    float x;
    float y;
    float width;
    float height;

} detection_t;

typedef struct
{
    detection_t detections[MAX_DETECTIONS];

    int count;

} detection_result_t;

bool ai_model_init(void);

bool ai_model_detect(
    const processed_image_t *image,
    detection_result_t *result
);

#endif
