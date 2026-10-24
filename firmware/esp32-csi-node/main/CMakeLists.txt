idf_component_register(
    SRCS "main.c" "wifi_csi.c"
    INCLUDE_DIRS "."
    REQUIRES esp_wifi esp_event nvs_flash esp_timer driver lwip freertos esp_netif
)
