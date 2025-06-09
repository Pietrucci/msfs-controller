#pragma once
#include <Arduino.h>
#include "EncoderManager.hpp"

namespace config
{

    struct EncoderConfig
    {
        gpio_num_t a;
        gpio_num_t b;
        uint8_t gamepad_button_inc;
        uint8_t gamepad_button_dec;
    };

    constexpr size_t PCF_PINS_LENGTH = 16;
    constexpr size_t SWITCHES_LIST_LENGTH = 2;
    constexpr size_t ENCODERS_LIST_LENGTH = 2;

    constexpr size_t PCF_BUTTONS_INDEX_START = 1;
    constexpr size_t SWITCHES_BUTTONS_INDEX_START = PCF_BUTTONS_INDEX_START + PCF_PINS_LENGTH;
    constexpr size_t ENCODERS_BUTTONS_INDEX_START = SWITCHES_BUTTONS_INDEX_START + SWITCHES_LIST_LENGTH;

    constexpr std::array<EncoderConfig, ENCODERS_LIST_LENGTH> ENCODERS_CONFIG = {{{GPIO_NUM_22, GPIO_NUM_21, ENCODERS_BUTTONS_INDEX_START + 0, ENCODERS_BUTTONS_INDEX_START + 1},
                                                                                  {GPIO_NUM_27, GPIO_NUM_26, ENCODERS_BUTTONS_INDEX_START + 2, ENCODERS_BUTTONS_INDEX_START + 3}}};

    constexpr uint16_t NUMBER_OF_GAMEPAD_BUTTONS = PCF_PINS_LENGTH + ENCODERS_LIST_LENGTH * 2 + SWITCHES_LIST_LENGTH;

}
