#pragma once
#include <Arduino.h>

namespace config
{

    struct EncoderPinout
    {
        gpio_num_t a;
        gpio_num_t b;
    };

    struct PcfPinout
    {
        gpio_num_t sda;
        gpio_num_t scl;
        gpio_num_t interrupt;
    };

    constexpr gpio_num_t POTENTIOMETER_PIN = GPIO_NUM_36;
    constexpr gpio_num_t SWITCHES_LIST[] = {GPIO_NUM_39, GPIO_NUM_34, GPIO_NUM_35, GPIO_NUM_33, GPIO_NUM_25};
    constexpr EncoderPinout ENCODERS_LIST[] = {{GPIO_NUM_26, GPIO_NUM_27},
                                               {GPIO_NUM_14, GPIO_NUM_13},
                                               {GPIO_NUM_19, GPIO_NUM_18},
                                               {GPIO_NUM_5, GPIO_NUM_17},
                                               {GPIO_NUM_16, GPIO_NUM_4},
                                               {GPIO_NUM_2, GPIO_NUM_15}};
    constexpr PcfPinout PCF_PINOUT = {GPIO_NUM_21, GPIO_NUM_22, GPIO_NUM_23};

    constexpr size_t PCF_BUTTONS = 16;
    constexpr size_t SWITCHES_LIST_LENGTH = sizeof(SWITCHES_LIST) / sizeof(SWITCHES_LIST[0]);
    constexpr size_t ENCODERS_LIST_LENGTH = sizeof(ENCODERS_LIST) / sizeof(ENCODERS_LIST[0]);

    constexpr size_t PCF_BUTTONS_INDEX_START = 1;
    constexpr size_t SWITCHES_BUTTONS_INDEX_START = PCF_BUTTONS_INDEX_START + PCF_BUTTONS;
    constexpr size_t ENCODERS_BUTTONS_INDEX_START = SWITCHES_BUTTONS_INDEX_START + SWITCHES_LIST_LENGTH;

    constexpr uint16_t NUMBER_OF_GAMEPAD_BUTTONS = PCF_BUTTONS + SWITCHES_LIST_LENGTH + 2 * ENCODERS_LIST_LENGTH;

}
