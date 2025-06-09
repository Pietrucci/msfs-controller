#pragma once
#include <Arduino.h>
#include <ESP32Encoder.h>
#include <BleGamepad.h>
#include <memory>
#include "Configuration.hpp"

namespace msfs
{
    constexpr uint16_t ENCODER_FILTER = 1023;

    struct EncoderContext
    {
        ESP32Encoder encoder;
        QueueHandle_t queue_handle;
        int last_count;
        int dec_button;
        int inc_button;

        EncoderContext(config::EncoderPinout pinout, QueueHandle_t queue, enc_isr_cb_t cb)
            : encoder(true, cb, this),
              queue_handle(queue)
        {
            static auto button_index = config::ENCODERS_BUTTONS_INDEX_START;
            dec_button = button_index++;
            inc_button = button_index++;
            encoder.attachSingleEdge(pinout.a, pinout.b);
            encoder.clearCount();
            encoder.setFilter(ENCODER_FILTER);
        }
    };

    class EncoderManager
    {
    public:
        EncoderManager(BleGamepad &ble_gamepad);

        EncoderManager(const EncoderManager &) = delete;
        EncoderManager &operator=(const EncoderManager &) = delete;
        EncoderManager(EncoderManager &&) = delete;
        EncoderManager &operator=(EncoderManager &&) = delete;

    private:
        BleGamepad &m_ble_gamepad;
        QueueHandle_t m_queue;
        std::vector<std::unique_ptr<EncoderContext>> m_contexts;

        static void IRAM_ATTR isr(void *arg);
        static void task_entry(void *pv);

        void task();
    };
}