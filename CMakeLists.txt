idf_component_register(
    SRCS "main.c" "Utils.c"
    INCLUDE_DIRS "."
    REQUIRES htu31 lora_proto ssd1306 mqtt
    PRIV_REQUIRES nvs_flash esp_wifi
)
