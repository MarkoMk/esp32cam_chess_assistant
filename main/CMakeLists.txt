idf_component_register(
    SRCS
        "main.c"
        "camera.c"
        "ai_model.c"
        "image_processing.c"
        "chess_board.c"
        "chess_engine.c"
        "display.c"

    INCLUDE_DIRS
        "."

    REQUIRES
        esp32-camera
)