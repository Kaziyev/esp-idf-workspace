idf_component_register(
    SRCS
        "main.c"
        "bmi2.c"
        "bmi270.c"
        "bmm150.c"
        "bmp3.c"
        "barometer.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_i2c esp_timer
)

# Keep the API and adapter on the same Pa / degrees-C representation.
target_compile_definitions(${COMPONENT_LIB} PRIVATE BMP3_FLOAT_COMPENSATION)
