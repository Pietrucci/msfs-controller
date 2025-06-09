#include <Arduino.h>
#include <BleGamepad.h>
#include <PCF8575.h>
#include "driver/gpio.h"
#include <ESP32Encoder.h>
#include <vector>
#include <memory>

#include "Configuration.hpp"
#include "EncoderManager.hpp"
#include "PotentiometerManager.hpp"
#include "SwitchManager.hpp"


struct PcfPin
{
  gpio_num_t sda;
  gpio_num_t scl;
  gpio_num_t interrupt;
};



constexpr PcfPin PCF_PINS{GPIO_NUM_18, GPIO_NUM_19, GPIO_NUM_5};

constexpr gpio_num_t SWITCHES_LIST[] = {GPIO_NUM_23, GPIO_NUM_17};




constexpr uint16_t DEBOUNCE_TIME_MS = 20;


SemaphoreHandle_t pcf_semaphore = nullptr;
SemaphoreHandle_t switch_semaphore = nullptr;

BleGamepad bleGamepad("MSFS Controller", "Pietraszak sp. z o.o.");
BleGamepadConfiguration bleGamepadConfig;
PCF8575 pcf;

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

void pcf_task(void *pvParameter)
{
  Serial.println("PCF Task start");

  while (true)
  {
    if (xSemaphoreTake(pcf_semaphore, portMAX_DELAY) == pdTRUE)
    {
      Serial.println("PCF Task woken");

      gpio_intr_disable((gpio_num_t)PCF_PINS.interrupt);

      vTaskDelay(DEBOUNCE_TIME_MS / portTICK_PERIOD_MS);

      // auto readout = pcf.read16();
      uint16_t readout = esp_random();

      for (int i = 15; i >= 0; i--)
      {
        bool bit = (readout >> i) & 0x01;
        if (bit)
        {
          bleGamepad.press(i + 1);
        }
        else
        {
          bleGamepad.release(i + 1);
        }
      }
      bleGamepad.sendReport();

      while (digitalRead(PCF_PINS.interrupt) == LOW)
      {
        vTaskDelay(1);
      }
      gpio_intr_enable((gpio_num_t)PCF_PINS.interrupt);
    }
  }
}

void setup_buttons()
{
  Serial.println("Button setup");

  assert(Wire.begin(PCF_PINS.sda, PCF_PINS.scl));
  // assert(pcf.begin());

  pcf_semaphore = xSemaphoreCreateBinary();
  assert(pcf_semaphore);
  xTaskCreate(pcf_task, "pcf_task", 1024 * 16, nullptr, 1, nullptr);

  pinMode(PCF_PINS.interrupt, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PCF_PINS.interrupt), []()
                  { xSemaphoreGiveFromISR(pcf_semaphore, nullptr); }, FALLING);
}




void setup()
{
  setCpuFrequencyMhz(80);
  Serial.begin(115200);
  Serial.println("MSFS Controller app");

  setup_gamepad();

  encoderManager = std::unique_ptr<msfs::EncoderManager>(new msfs::EncoderManager(bleGamepad, config::ENCODERS_CONFIG));
  potManager = std::unique_ptr<msfs::PotentiometerManager>(new msfs::PotentiometerManager(bleGamepad, config::POTENTIOMETER_PIN));
  switchManager = std::unique_ptr<msfs::SwitchManager>(new msfs::SwitchManager(bleGamepad));
  setup_buttons();

}

void loop()
{
  vTaskDelay(portMAX_DELAY);
}
