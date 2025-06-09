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
        config::EncoderConfig config;
        int last_count;

        EncoderContext(config::EncoderConfig config, QueueHandle_t queue, enc_isr_cb_t cb)
            : config(config),
              encoder(true, cb, this),
              queue_handle(queue)
        {
            encoder.attachSingleEdge(config.a, config.b);
            encoder.clearCount();
            encoder.setFilter(ENCODER_FILTER);
        }
    };

    class EncoderManager
    {
    public:
        EncoderManager(BleGamepad &ble_gamepad, const std::array<config::EncoderConfig, config::ENCODERS_LIST_LENGTH>& config);

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