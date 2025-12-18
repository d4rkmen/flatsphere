/**
 * @file sdcard.cpp
 * @brief SD Card Driver Class
 * @author d4rkmen
 * @license Apache License 2.0
 * @copyright Copyright (c) 2024 - Anderson Antunes
 */

#ifndef SD_CARD_MANAGER_H
#define SD_CARD_MANAGER_H

#include "driver/sdmmc_host.h"
#include <string>

class SDCard
{
public:
    bool mount(bool format_if_mount_failed);
    bool eject();
    bool is_mounted();
    char* get_mount_point();

    // SDMMC info getters
    std::string get_manufacturer();
    std::string get_device_name();
    uint64_t get_capacity();
    uint32_t getSpeedKHz() const { return card ? card->max_freq_khz : 0; }

private:
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    sdmmc_card_t* card = nullptr;
    bool _is_mounted = false;
};

#endif // SD_CARD_MANAGER_H
