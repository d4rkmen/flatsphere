/**
 * @file sdcard.cpp
 * @brief SD Card Driver Class
 * @author d4rkmen
 * @license Apache License 2.0
 * @copyright Copyright (c) 2024 - Anderson Antunes
 */

#include <string.h>
#include <format>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "sdcard.hpp"

#define PIN_NUM_CLK GPIO_NUM_14
#define PIN_NUM_CMD GPIO_NUM_17
#define PIN_NUM_D0 GPIO_NUM_16
#define PIN_NUM_D1 GPIO_NUM_NC
#define PIN_NUM_D2 GPIO_NUM_NC
#define PIN_NUM_D3 GPIO_NUM_NC

static const char* MOUNT_POINT = "/sdcard";
static const char* TAG = "SDCARD";

bool SDCard::mount(bool format_if_mount_failed)
{
    if (_is_mounted)
    {
        ESP_LOGI(TAG, "SD card already mounted");
        return true;
    }
    ESP_LOGI(TAG, "Mounting SD card");
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 1; // 1-wire  / 4-wire   slot_config.width = 4;

    slot_config.clk = PIN_NUM_CLK;
    slot_config.cmd = PIN_NUM_CMD;
    slot_config.d0 = PIN_NUM_D0;
    slot_config.d1 = PIN_NUM_D1;
    slot_config.d2 = PIN_NUM_D2;
    slot_config.d3 = PIN_NUM_D3;
    slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {.format_if_mount_failed = format_if_mount_failed,
                                                     .max_files = 5,
                                                     .allocation_unit_size = 16 * 1024,
                                                     .disk_status_check_enable = false,
                                                     .use_one_fat = false};

    esp_err_t ret = esp_vfs_fat_sdmmc_mount(MOUNT_POINT, &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK)
    {
        card = nullptr;
        spi_bus_free((spi_host_device_t)host.slot);
        ESP_LOGE(TAG, "Failed to mount filesystem");
        return false;
    };

    sdmmc_card_print_info(stdout, card);
    _is_mounted = true;

    return true;
}

bool SDCard::eject()
{
    if (!_is_mounted)
    {
        ESP_LOGI(TAG, "SD card not mounted");
        return true;
    }
    esp_err_t ret = esp_vfs_fat_sdcard_unmount(MOUNT_POINT, card);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to unmount SD card");
        return false;
    }
    card = nullptr;
    // spi_bus_free((spi_host_device_t)host.slot);
    _is_mounted = false;

    return true;
}

bool SDCard::is_mounted() { return _is_mounted; }

char* SDCard::get_mount_point() { return (char*)MOUNT_POINT; }

std::string SDCard::get_manufacturer()
{
    if (!_is_mounted || card == nullptr)
    {
        return "";
    }
    return std::format("0x{:02x}", card->cid.mfg_id);
}

std::string SDCard::get_device_name()
{
    if (!_is_mounted || card == nullptr)
    {
        return "";
    }
    return std::string((const char*)&card->cid.name);
}

uint64_t SDCard::get_capacity()
{
    if (!_is_mounted || card == nullptr)
    {
        return 0;
    }
    return ((uint64_t)card->csd.capacity) * card->csd.sector_size;
}