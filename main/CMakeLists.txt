file(GLOB_RECURSE HAL_SRCS
    hal/*.cpp
)

file(GLOB_RECURSE SETTINGS_SRCS
    settings/*.cpp
)

file(GLOB_RECURSE UI_SRCS
    ui/*.c
)

idf_component_register(SRCS "main.cpp" ${HAL_SRCS} ${SETTINGS_SRCS} ${UI_SRCS}
    INCLUDE_DIRS "." "ui" "hal" "settings"
    REQUIRES spi_flash fatfs esp_wifi json esp_lcd driver nvs_flash esp_timer esp_adc usb usb_host_msc sanotts
    EMBED_FILES assets/coin16.wav assets/error.wav assets/usb_connected.wav assets/usb_disconnected.wav
)
                    

if(CONFIG_FLATSPHERE_FLASH_FACTORY_NVS)
    nvs_create_partition_image(nvs ../nvs.csv FLASH_IN_PROJECT)
else()
    nvs_create_partition_image(nvs ../nvs.csv)
endif()