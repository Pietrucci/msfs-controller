#include "SwitchManager.hpp"

namespace msfs
{
    SwitchManager::SwitchManager(BleGamepad &ble_gamepad)
        : m_ble_gamepad(ble_gamepad)
    {
        m_semaphore = xSemaphoreCreateBinary();
        assert(m_semaphore);

        for (size_t i = 0; i < config::SWITCHES_LIST_LENGTH; ++i)
        {
            pinMode(config::SWITCHES_LIST[i], INPUT_PULLUP);
            m_last_states[i] = digitalRead(config::SWITCHES_LIST[i]);
            attachInterruptArg(digitalPinToInterrupt(config::SWITCHES_LIST[i]), &SwitchManager::isr_handler, this, CHANGE);
        }

        xTaskCreatePinnedToCore(task_entry, "switch_task", sw::TASK_STACK_SIZE, this, 1, &m_task_handle, 1);
    }

    SwitchManager::~SwitchManager()
    {
        if (m_task_handle)
            vTaskDelete(m_task_handle);

        if (m_semaphore)
            vSemaphoreDelete(m_semaphore);
    }

    void IRAM_ATTR SwitchManager::isr_handler(void *arg)
    {
        auto *self = static_cast<SwitchManager *>(arg);
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(self->m_semaphore, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken)
            portYIELD_FROM_ISR();
    }

    void SwitchManager::task_entry(void *pv)
    {
        static_cast<SwitchManager *>(pv)->task();
    }

    void SwitchManager::task()
    {
        Serial.println("Switch task start");

        bool first_run = true;

        while (true)
        {
            if (first_run || xSemaphoreTake(m_semaphore, portMAX_DELAY) == pdTRUE)
            {
                vTaskDelay(pdMS_TO_TICKS(sw::DEBOUNCE_TIME_MS));

                for (size_t i = 0; i < config::SWITCHES_LIST_LENGTH; ++i)
                {
                    int current = digitalRead(config::SWITCHES_LIST[i]);
                    if (first_run || current != m_last_states[i])
                    {
                        m_last_states[i] = current;
                        Serial.printf("Switch on pin %d changed to: %s\n", config::SWITCHES_LIST[i], current == LOW ? "ON" : "OFF");

                        if (current == HIGH)
                            m_ble_gamepad.release(i + config::SWITCHES_BUTTONS_INDEX_START);
                        else
                            m_ble_gamepad.press(i + config::SWITCHES_BUTTONS_INDEX_START);

                        m_ble_gamepad.sendReport();
                    }
                }
                first_run = false;
            }
        }
    }
}
