#pragma once
#include <Arduino.h>
#include <BleGamepad.h>
#include <freertos/semphr.h>
#include <array>
#include "Configuration.hpp"

namespace msfs
{
    namespace sw
    {
        constexpr int TASK_STACK_SIZE = 1024 * 8;
        constexpr TickType_t DEBOUNCE_TIME_MS = 50;
    }

    class SwitchManager
    {
    public:
        SwitchManager(BleGamepad &ble_gamepad);

        SwitchManager(const SwitchManager &) = delete;
        SwitchManager &operator=(const SwitchManager &) = delete;
        SwitchManager(SwitchManager &&) = delete;
        SwitchManager &operator=(SwitchManager &&) = delete;

        ~SwitchManager();

    private:
        static void task_entry(void *pv);
        void task();

        static void IRAM_ATTR isr_handler(void *arg);

        BleGamepad &m_ble_gamepad;
        std::array<int, config::SWITCHES_LIST_LENGTH> m_last_states;
        SemaphoreHandle_t m_semaphore;

        TaskHandle_t m_task_handle = nullptr;
    };
}
