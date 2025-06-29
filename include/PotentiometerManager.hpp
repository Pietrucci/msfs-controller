#pragma once
#include <Arduino.h>
#include <BleGamepad.h>

namespace msfs
{
    namespace pot
    {
        constexpr int TASK_STACK_SIZE = 1024 * 8;
        constexpr TickType_t SAMPLING_PERIOD_MS = 20;
        constexpr float DEFAULT_ALPHA = 1.0f;
        constexpr int HYSTERESIS = 0;
        constexpr int ADC_MIN = 152; // To be calibrated based on voltage divider
        constexpr int ADC_MAX = 3761; // To be calibrated based on voltage divider
        constexpr int OUTPUT_MIN = 0;
        constexpr int OUTPUT_MAX = 255;
        constexpr uint8_t MS_SAMPLES = 32; // 16 próbek ≈ +2 bity / 4× mniejszy szum

    }

    class PotentiometerManager
    {
    public:
        PotentiometerManager(BleGamepad &ble_gamepad, float alpha = pot::DEFAULT_ALPHA);

        PotentiometerManager(const PotentiometerManager &) = delete;
        PotentiometerManager &operator=(const PotentiometerManager &) = delete;
        PotentiometerManager(PotentiometerManager &&) = delete;
        PotentiometerManager &operator=(PotentiometerManager &&) = delete;

    private:
        static void task_entry(void *pv);
        void task();

        BleGamepad &m_ble_gamepad;
        gpio_num_t m_pin;
        float m_alpha;
        float m_filtered_value = 0.0f;
    };
}
