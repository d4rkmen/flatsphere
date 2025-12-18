/**
 * @file timezone.h
 * @brief Timezone definitions and DST handling
 *
 * Uses POSIX timezone strings for automatic DST calculation
 * Format: STD offset [DST [offset][,start[/time],end[/time]]]
 */

#ifndef TIMEZONE_H
#define TIMEZONE_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

    /**
     * @brief Timezone information structure
     */
    typedef struct
    {
        const char* name;        // Display name
        const char* posix_tz;    // POSIX timezone string
        const char* city;        // Representative city
        int8_t utc_offset_hours; // Standard time UTC offset in hours
        bool has_dst;            // Whether this timezone uses DST
    } timezone_info_t;

    /**
     * @brief List of common timezones with DST rules
     *
     * Organized by region for easier selection in UI
     */
    static const timezone_info_t TIMEZONE_LIST[] = {
        // North America
        {"Hawaii", "HST10", "Honolulu", -10, false},
        {"Alaska", "AKST9AKDT,M3.2.0,M11.1.0", "Anchorage", -9, true},
        {"Pacific", "PST8PDT,M3.2.0,M11.1.0", "Los Angeles", -8, true},
        {"Mountain", "MST7MDT,M3.2.0,M11.1.0", "Denver", -7, true},
        {"Arizona", "MST7", "Phoenix", -7, false},
        {"Central", "CST6CDT,M3.2.0,M11.1.0", "Chicago", -6, true},
        {"Eastern", "EST5EDT,M3.2.0,M11.1.0", "New York", -5, true},
        {"Atlantic", "AST4ADT,M3.2.0,M11.1.0", "Halifax", -4, true},
        {"Newfoundland", "NST3:30NDT,M3.2.0,M11.1.0", "St. John's", -3, true},

        // Europe
        {"GMT", "GMT0", "London", 0, false},
        {"UK/Ireland", "GMT0BST,M3.5.0/1,M10.5.0", "London", 0, true},
        {"Central Europe", "CET-1CEST,M3.5.0,M10.5.0/3", "Paris", 1, true},
        {"Eastern Europe", "EET-2EEST,M3.5.0/3,M10.5.0/4", "Athens", 2, true},
        {"Moscow", "MSK-3", "Moscow", 3, false},

        // Asia
        {"Dubai", "GST-4", "Dubai", 4, false},
        {"Pakistan", "PKT-5", "Karachi", 5, false},
        {"India", "IST-5:30", "New Delhi", 5, false},
        {"Bangladesh", "BST-6", "Dhaka", 6, false},
        {"Thailand", "ICT-7", "Bangkok", 7, false},
        {"China/HK/Singapore", "CST-8", "Beijing", 8, false},
        {"Japan", "JST-9", "Tokyo", 9, false},
        {"Korea", "KST-9", "Seoul", 9, false},
        {"Australia East", "AEST-10AEDT,M10.1.0,M4.1.0/3", "Sydney", 10, true},
        {"Australia Central", "ACST-9:30ACDT,M10.1.0,M4.1.0/3", "Adelaide", 9, true},
        {"Australia West", "AWST-8", "Perth", 8, false},

        // Pacific
        {"New Zealand", "NZST-12NZDT,M9.5.0,M4.1.0/3", "Auckland", 12, true},

        // South America
        {"Brazil East", "BRT3BRST,M10.3.0/0,M2.3.0/0", "Sao Paulo", -3, true},
        {"Argentina", "ART3", "Buenos Aires", -3, false},

        // Africa
        {"South Africa", "SAST-2", "Johannesburg", 2, false},
        {"Egypt", "EET-2", "Cairo", 2, false},

        // UTC
        {"UTC", "UTC0", "UTC", 0, false},
    };

#define TIMEZONE_COUNT (sizeof(TIMEZONE_LIST) / sizeof(timezone_info_t))
#define TIMEZONE_DEFAULT_INDEX 12 // Eastern Europe

    /**
     * @brief Get timezone count
     */
    static inline uint8_t timezone_get_count(void)
    {
        return TIMEZONE_COUNT;
    }

    /**
     * @brief Get timezone info by index
     * @param index Timezone index (0 to TIMEZONE_COUNT-1)
     * @return Pointer to timezone_info_t or NULL if invalid
     */
    static inline const timezone_info_t* timezone_get_info(uint8_t index)
    {
        if (index >= TIMEZONE_COUNT)
        {
            return NULL;
        }
        return &TIMEZONE_LIST[index];
    }

    /**
     * @brief Find timezone index by name
     * @param name Timezone name to search for
     * @return Index if found, -1 if not found
     */
    static inline int8_t timezone_find_by_name(const char* name)
    {
        for (uint8_t i = 0; i < TIMEZONE_COUNT; i++)
        {
            if (strcmp(TIMEZONE_LIST[i].name, name) == 0)
            {
                return i;
            }
        }
        return -1;
    }

#ifdef __cplusplus
}
#endif

#endif // TIMEZONE_H
