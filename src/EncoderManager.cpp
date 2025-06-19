#include "EncoderManager.hpp"

namespace msfs
{

    constexpr uint16_t ENCODER_QUEUE_SIZE = 32;
    constexpr uint16_t TASK_STACK_DEPTH = 1024 * 8;

    EncoderManager::EncoderManager(BleGamepad &ble_gamepad) : m_ble_gamepad(ble_gamepad)
    {
        m_queue = xQueueCreate(ENCODER_QUEUE_SIZE, sizeof(EncoderContext *));
        assert(m_queue);

        ESP32Encoder::useInternalWeakPullResistors = puType::up;

        for (auto &entry : config::ENCODERS_LIST)
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
            ctx->last_count = ctx->encoder.getCount();

        uint16_t pulseSpacingMs = 25;
        uint16_t pulseWidthMs = 25;

        constexpr uint8_t MAX_IMPULSES_PER_EVENT = 5;
        constexpr uint16_t MIN_PULSE_MS = 1;

        while (true)
        {
            EncoderContext *ctx = nullptr;

            if (xQueueReceive(m_queue, &ctx, portMAX_DELAY) == pdTRUE)
            {
                int current = ctx->encoder.getCount();
                int diff = current - ctx->last_count;
                int steps = abs(diff);

                // =============================
                // 🧪 TEST FEATURE: Regulacja czasów
                // =============================
                // UWAGA: Sprawdzamy fizycznie naciśnięty przycisk, nie tylko "stronę" enkodera
                uint8_t test_button = (diff > 0) ? ctx->inc_button : ctx->dec_button;

                if (test_button == 24)
                {
                    pulseSpacingMs++;
                    Serial.printf("[TEST] pulseSpacingMs zwiększono: %ums\n", pulseSpacingMs);
                    ctx->last_count = current;
                    continue;
                }
                if (test_button == 25)
                {
                    if (pulseSpacingMs > MIN_PULSE_MS)
                    {
                        pulseSpacingMs--;
                        Serial.printf("[TEST] pulseSpacingMs zmniejszono: %ums\n", pulseSpacingMs);
                    }
                    else
                    {
                        Serial.println("[TEST] pulseSpacingMs już minimalne!");
                    }
                    ctx->last_count = current;
                    continue;
                }
                if (test_button == 28)
                {
                    pulseWidthMs++;
                    Serial.printf("[TEST] pulseWidthMs zwiększono: %ums\n", pulseWidthMs);
                    ctx->last_count = current;
                    continue;
                }
                if (test_button == 29)
                {
                    if (pulseWidthMs > MIN_PULSE_MS)
                    {
                        pulseWidthMs--;
                        Serial.printf("[TEST] pulseWidthMs zmniejszono: %ums\n", pulseWidthMs);
                    }
                    else
                    {
                        Serial.println("[TEST] pulseWidthMs już minimalne!");
                    }
                    ctx->last_count = current;
                    continue;
                }
                // =============================
                // 🧪 KONIEC TESTU
                // =============================

                if (steps > 0)
                {
                    if (steps > MAX_IMPULSES_PER_EVENT)
                        steps = MAX_IMPULSES_PER_EVENT;

                    uint8_t btn = (diff > 0) ? ctx->inc_button : ctx->dec_button;

                    Serial.printf("Encoder %s (btn %u), diff = %d → %d impulsów\n",
                                  diff > 0 ? "increment" : "decrement",
                                  btn, diff, steps);

                    for (int i = 0; i < steps; ++i)
                    {
                        m_ble_gamepad.press(btn);
                        m_ble_gamepad.sendReport();
                        vTaskDelay(pdMS_TO_TICKS(pulseWidthMs));

                        m_ble_gamepad.release(btn);
                        m_ble_gamepad.sendReport();
                        vTaskDelay(pdMS_TO_TICKS(pulseSpacingMs));
                    }
                }

                ctx->last_count = current;
            }
        }
    }

}
