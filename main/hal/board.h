/**
 * @file board.h
 *
 */
#pragma once

namespace HAL
{
    /**
     * @brief Board type
     */
    enum class BoardType
    {
        UNKNOWN,
        WAVESHARE_KNOB_185C, // Waveshare Touch LCD 1.85C
        WAVESHARE_KNOB_18,   // Waveshare Knob 1.8 inch
    };
} // namespace HAL
