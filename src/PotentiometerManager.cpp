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
            int raw = analogRead(m_pin);
            m_filtered_value = m_alpha * raw + (1 - m_alpha) * m_filtered_value;
            int mapped = map((int)m_filtered_value, pot::ADC_MIN, pot::ADC_MAX, pot::OUTPUT_MIN, pot::OUTPUT_MAX);

            if (abs(mapped - previous_value) > pot::HYSTERESIS)
            {
                if (mapped != previous_value)
                {
                    Serial.print("Pot value: ");
                    Serial.println(mapped);
                    previous_value = mapped;
                    m_ble_gamepad.setAxes(mapped);
                    m_ble_gamepad.sendReport();
                }
            }

            vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(pot::SAMPLING_PERIOD_MS));
        }
    }
}
