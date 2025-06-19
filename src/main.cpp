#include <Arduino.h>
#include <BleGamepad.h>
#include <PCF8575.h>
#include "driver/gpio.h"
#include <vector>
#include <memory>

#include "Configuration.hpp"
#include "EncoderManager.hpp"
#include "PotentiometerManager.hpp"
#include "SwitchManager.hpp"

BleGamepad bleGamepad("MSFS Controller", "Pietraszak sp. z o.o.");
BleGamepadConfiguration bleGamepadConfig;

std::unique_ptr<msfs::EncoderManager> encoderManager;
std::unique_ptr<msfs::PotentiometerManager> potManager;
std::unique_ptr<msfs::SwitchManager> switchManager;

void setup_gamepad()
{
  bleGamepadConfig.setButtonCount(config::NUMBER_OF_GAMEPAD_BUTTONS);
  bleGamepadConfig.setControllerType(CONTROLLER_TYPE_GAMEPAD);
  bleGamepadConfig.setHatSwitchCount(0);
  bleGamepadConfig.setWhichAxes(true, false, false, false, false, false, false, false);
  bleGamepadConfig.setAxesMin(0);
  bleGamepadConfig.setAxesMax(255);
  bleGamepadConfig.setAutoReport(false);

  bleGamepad.begin(&bleGamepadConfig);
}

SemaphoreHandle_t pcf_semaphore = nullptr;
PCF8575 pcf;

void pcf_task(void *pvParameter)
{
  Serial.println("PCF Task start");

  constexpr uint16_t DEBOUNCE_TIME_MS = 20;
  uint16_t last_readout = 0xFFFF; // wszystkie przyciski „nie wciśnięte” na starcie

  while (true)
  {
    if (xSemaphoreTake(pcf_semaphore, portMAX_DELAY) == pdTRUE)
    {

      gpio_intr_disable((gpio_num_t)config::PCF_PINOUT.interrupt);

      do
      {
        vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_TIME_MS));

        uint16_t readout = pcf.read16();

        for (int i = 15; i >= 0; i--)
        {
          bool prev_bit = (last_readout >> i) & 0x01;
          bool curr_bit = (readout >> i) & 0x01;

          if (prev_bit != curr_bit)
          {
            if (curr_bit)
            {
              Serial.printf("Button depressed: (btn %d)\n", i);
              bleGamepad.release(i + 1);
            }
            else
            {
              Serial.printf("Button pressed: (btn %d)\n", i);
              bleGamepad.press(i + 1);
            }
          }
        }

        last_readout = readout;

      } while (digitalRead(config::PCF_PINOUT.interrupt) == LOW);

      bleGamepad.sendReport();

      gpio_intr_enable((gpio_num_t)config::PCF_PINOUT.interrupt);
    }
  }
}


void setup_buttons()
{
  Serial.println("Button setup");

  assert(Wire.begin(config::PCF_PINOUT.sda, config::PCF_PINOUT.scl));
  assert(pcf.begin());

  pcf_semaphore = xSemaphoreCreateBinary();
  assert(pcf_semaphore);
  xTaskCreate(pcf_task, "pcf_task", 1024 * 16, nullptr, 1, nullptr);

  pinMode(config::PCF_PINOUT.interrupt, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(config::PCF_PINOUT.interrupt), []()
                  { xSemaphoreGiveFromISR(pcf_semaphore, nullptr); }, FALLING);
}

void setup()
{
  setCpuFrequencyMhz(80);
  Serial.begin(115200);
  Serial.println("MSFS Controller app");

  setup_gamepad();

  encoderManager = std::unique_ptr<msfs::EncoderManager>(new msfs::EncoderManager(bleGamepad));
  potManager = std::unique_ptr<msfs::PotentiometerManager>(new msfs::PotentiometerManager(bleGamepad));
  switchManager = std::unique_ptr<msfs::SwitchManager>(new msfs::SwitchManager(bleGamepad));
  setup_buttons();
}

void loop()
{
  vTaskDelay(portMAX_DELAY);
}
