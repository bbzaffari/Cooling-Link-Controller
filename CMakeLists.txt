idf_component_register(
    SRCS "main.c" "Utils.c"
    INCLUDE_DIRS "."
    REQUIRES htu31 lora_proto ssd1306 ds18b20 mqtt sht30 lora 
    PRIV_REQUIRES nvs_flash esp_wifi
)