idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES esp_lcd driver nvs_flash esp_timer esp_adc esp_hw_support soc
)
