#include "PotentiometerManager.hpp"
#include "Configuration.hpp"

namespace msfs
{

    PotentiometerManager::PotentiometerManager(BleGamepad &ble_gamepad, float alpha)
        : m_ble_gamepad(ble_gamepad), m_pin(config::POTENTIOMETER_PIN), m_alpha(alpha)
    {
        m_filtered_value = analogRead(m_pin);

        xTaskCreatePinnedToCore(task_entry, "pot_task", pot::TASK_STACK_SIZE, this, 1, nullptr, 1);
    }

    void PotentiometerManager::task_entry(void *pv)
    {
        static_cast<PotentiometerManager *>(pv)->task();
    }

    void PotentiometerManager::task()
    {
        Serial.println("Potentiometer task start");

        TickType_t xLastWakeTime = xTaskGetTickCount();
        int previous_value = 0;

        while (true)
        {
            /* ---------- multisampling ---------- */
            uint32_t acc = 0;
            for (uint8_t i = 0; i < pot::MS_SAMPLES; ++i)
                acc += analogRead(m_pin);

            int raw = acc / pot::MS_SAMPLES; // średnia arytmetyczna
            /* ----------------------------------- */

            m_filtered_value = m_alpha * raw + (1 - m_alpha) * m_filtered_value;
            int mapped = map((int)m_filtered_value,
                             pot::ADC_MIN, pot::ADC_MAX,
                             pot::OUTPUT_MIN, pot::OUTPUT_MAX);

            if (abs(mapped - previous_value) > pot::HYSTERESIS)
            {
                previous_value = mapped;
                m_ble_gamepad.setAxes(mapped);
                m_ble_gamepad.sendReport();

                Serial.printf("Pot raw(avg): %d  → mapped: %d  (%dmV)\n",
                              raw, mapped, analogReadMilliVolts(m_pin));
            }

            vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(pot::SAMPLING_PERIOD_MS));
        }
    }
}
