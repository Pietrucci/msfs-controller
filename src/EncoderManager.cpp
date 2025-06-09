#include "EncoderManager.hpp"

namespace msfs
{

    constexpr uint16_t ENCODER_QUEUE_SIZE = 32;
    constexpr uint16_t TASK_STACK_DEPTH = 1024 * 8;

    EncoderManager::EncoderManager(BleGamepad &ble_gamepad, const std::array<config::EncoderConfig, config::ENCODERS_LIST_LENGTH> &config) : m_ble_gamepad(ble_gamepad)
    {
        m_queue = xQueueCreate(ENCODER_QUEUE_SIZE, sizeof(EncoderContext *));
        assert(m_queue);

        ESP32Encoder::useInternalWeakPullResistors = puType::up;

        for (auto &entry : config)
        {
            m_contexts.emplace_back(new EncoderContext(entry, m_queue, isr));
        }

        xTaskCreatePinnedToCore(task_entry, "encoder_task", TASK_STACK_DEPTH, this, 1, nullptr, 1);
    }

    void IRAM_ATTR EncoderManager::isr(void *arg)
    {
        EncoderContext *ctx = reinterpret_cast<EncoderContext *>(arg);
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(ctx->queue_handle, &ctx, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken)
        {
            portYIELD_FROM_ISR();
        }
    }

    void EncoderManager::task_entry(void *pv)
    {
        static_cast<EncoderManager *>(pv)->task();
    }

    void EncoderManager::task()
    {
        Serial.println("Encoder task start");

        for (auto &ctx : m_contexts)
        {
            ctx->last_count = ctx->encoder.getCount();
        }

        while (true)
        {
            EncoderContext *ctx = nullptr;

            if (xQueueReceive(m_queue, &ctx, portMAX_DELAY) == pdTRUE)
            {
                int currentCount = ctx->encoder.getCount();
                int diff = currentCount - ctx->last_count;

                if (diff > 0)
                {
                    Serial.printf("Encoder incremented (btn %zu)\n", ctx->config.gamepad_button_inc);
                    m_ble_gamepad.press(ctx->config.gamepad_button_inc);
                    m_ble_gamepad.sendReport();
                    vTaskDelay(pdMS_TO_TICKS(20));
                    m_ble_gamepad.release(ctx->config.gamepad_button_inc);
                    m_ble_gamepad.sendReport();
                }
                else if (diff < 0)
                {
                    Serial.printf("Encoder decremented (btn %zu)\n", ctx->config.gamepad_button_dec);
                    m_ble_gamepad.press(ctx->config.gamepad_button_dec);
                    m_ble_gamepad.sendReport();
                    vTaskDelay(pdMS_TO_TICKS(20));
                    m_ble_gamepad.release(ctx->config.gamepad_button_dec);
                    m_ble_gamepad.sendReport();
                }

                ctx->last_count = currentCount;

                vTaskDelay(pdMS_TO_TICKS(20));
            }
        }
    }

}
