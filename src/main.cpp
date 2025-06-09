#include <Arduino.h>
#include <BleGamepad.h>
#include <PCF8575.h>
#include "driver/gpio.h"
#include <ESP32Encoder.h>
#include <vector>
#include <memory>
#include "EncoderManager.hpp"
#include "Configuration.hpp"


struct PcfPin
{
  gpio_num_t sda;
  gpio_num_t scl;
  gpio_num_t interrupt;
};



constexpr PcfPin PCF_PINS{GPIO_NUM_18, GPIO_NUM_19, GPIO_NUM_5};

constexpr gpio_num_t SWITCHES_LIST[] = {GPIO_NUM_23, GPIO_NUM_17};




constexpr config::EncoderConfig ENCODERS_LIST[] = {{GPIO_NUM_22, GPIO_NUM_21}, {GPIO_NUM_27, GPIO_NUM_26}};



constexpr gpio_num_t POTENTIOMETER_PIN = GPIO_NUM_36;

constexpr uint16_t DEBOUNCE_TIME_MS = 20;
constexpr uint16_t ENCODER_DEBOUNCE_MS = 20;


SemaphoreHandle_t pcf_semaphore = nullptr;
SemaphoreHandle_t switch_semaphore = nullptr;

BleGamepad bleGamepad("MSFS Controller", "Pietraszak sp. z o.o.");
BleGamepadConfiguration bleGamepadConfig;
PCF8575 pcf;

std::unique_ptr<msfs::EncoderManager> encoderManager;


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

void switch_task(void *pvParameter)
{
  Serial.println("Switch task start");

  int last_state[config::SWITCHES_LIST_LENGTH];

  for (int i = 0; i < config::SWITCHES_LIST_LENGTH; ++i)
  {
    last_state[i] = digitalRead(SWITCHES_LIST[i]);
  }

  while (true)
  {
    if (xSemaphoreTake(switch_semaphore, portMAX_DELAY) == pdTRUE)
    {
      vTaskDelay(DEBOUNCE_TIME_MS / portTICK_PERIOD_MS);

      for (int i = 0; i < config::SWITCHES_LIST_LENGTH; ++i)
      {
        uint8_t pin = SWITCHES_LIST[i];
        uint8_t current = digitalRead(pin);
        if (current != last_state[i])
        {
          last_state[i] = current;
          // Obsłuż zmianę stanu przełącznika
          Serial.printf("Switch on pin %d changed to: %s\n", pin, current == LOW ? "ON" : "OFF");
          if (current == HIGH)
          {
            bleGamepad.release(i + config::SWITCHES_BUTTONS_INDEX_START);
          }
          else
          {
            bleGamepad.press(i + config::SWITCHES_BUTTONS_INDEX_START);
          }
          bleGamepad.sendReport();
        }
      }
    }
  }
}

void setup_switches()
{
  switch_semaphore = xSemaphoreCreateBinary();
  assert(switch_semaphore);

  for (auto pin : SWITCHES_LIST)
  {
    pinMode(pin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(pin), []()
                    { xSemaphoreGiveFromISR(switch_semaphore, nullptr); }, CHANGE);
  }
  xTaskCreate(switch_task, "switch_task", 1024 * 8, nullptr, 1, nullptr);

  xSemaphoreGive(switch_semaphore);
}




float alpha = 0.7; // 0.0–1.0, im mniejsze, tym silniejsze wygładzanie
static float filtered_value = analogRead(POTENTIOMETER_PIN);

void potentiometer_task(void *pv)
{
  Serial.println("Potentiometer task start");

  TickType_t xLastWakeTime;
  const TickType_t POTENTIOMETER_SAMPLING_FREQUENCY_MS = 20;

  while (true)
  {
    xLastWakeTime = xTaskGetTickCount();
    int raw = analogRead(POTENTIOMETER_PIN);

    filtered_value = alpha * raw + (1 - alpha) * filtered_value;
    int mapped = map((int)filtered_value, 0, 4095, 0, 255);
    static int previous_value = 0;

    const int HYSTERESIS = 3;
    if (abs(mapped - previous_value) > HYSTERESIS)
    {
      if (mapped != previous_value)
      {
        Serial.print("Pot value:");
        Serial.println(mapped);
        previous_value = mapped;
        bleGamepad.setAxes(mapped);
        bleGamepad.sendReport();
      }
    }

    vTaskDelayUntil(&xLastWakeTime, POTENTIOMETER_SAMPLING_FREQUENCY_MS / portTICK_PERIOD_MS);
  }
}

void ARDUINO_ISR_ATTR potentiometer_callback()
{
}
void setup_potentiometer()
{
  xTaskCreate(potentiometer_task, "potentiometer_task", 1024 * 8, nullptr, 1, nullptr);
}




void setup()
{
  setCpuFrequencyMhz(80);
  Serial.begin(115200);
  Serial.println("MSFS Controller app");

  setup_gamepad();
  encoderManager = std::unique_ptr<msfs::EncoderManager>(new msfs::EncoderManager(bleGamepad, config::ENCODERS_CONFIG));
  setup_buttons();
  setup_switches();

  setup_potentiometer();
}

void loop()
{
  vTaskDelay(portMAX_DELAY);
}
