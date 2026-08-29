idf_component_register(
    SRCS
        "main.c"
        "bmi2.c"
        "bmi270.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_i2c
)