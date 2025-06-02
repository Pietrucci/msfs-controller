#include <Arduino.h>
#include <BleGamepad.h>
#include <PCF8575.h>
#include "driver/gpio.h"

struct EncoderPin
{
  gpio_num_t a;
  gpio_num_t b;
};

struct EncoderState
{
  volatile uint8_t last_state;
  volatile uint32_t last_isr_time;
};

struct PcfPin
{
  gpio_num_t sda;
  gpio_num_t scl;
  gpio_num_t interrupt;
};

constexpr PcfPin PCF_PINS{GPIO_NUM_18, GPIO_NUM_19, GPIO_NUM_5};
constexpr size_t PCF_PINS_LENGTH = 16;

constexpr gpio_num_t SWITCHES_LIST[] = {GPIO_NUM_23, GPIO_NUM_17};
constexpr size_t SWITCHES_LIST_LENGTH = sizeof(SWITCHES_LIST) / sizeof(SWITCHES_LIST[0]);

constexpr EncoderPin ENCODERS_LIST[] = {{GPIO_NUM_22, GPIO_NUM_21}};
constexpr size_t ENCODERS_LIST_LENGTH = sizeof(ENCODERS_LIST) / sizeof(ENCODERS_LIST[0]);

constexpr gpio_num_t POTENTIOMETER_PINS = {};

constexpr uint16_t NUMBER_OF_GAMEPAD_BUTTONS = PCF_PINS_LENGTH + ENCODERS_LIST_LENGTH * 2 + SWITCHES_LIST_LENGTH;

constexpr uint16_t DEBOUNCE_TIME_MS = 20;
constexpr uint16_t ENCODER_DEBOUNCE_MS = 5;

SemaphoreHandle_t pcf_semaphore = nullptr;
SemaphoreHandle_t switch_semaphore = nullptr;
SemaphoreHandle_t encoder_semaphore = nullptr;

BleGamepad bleGamepad("MSFS Controller", "Pietraszak sp. z o.o.");
BleGamepadConfiguration bleGamepadConfig;
PCF8575 pcf;
EncoderState ENCODERS_STATE[ENCODERS_LIST_LENGTH] = {};

void setup_gamepad()
{
  bleGamepadConfig.setButtonCount(NUMBER_OF_GAMEPAD_BUTTONS);
  bleGamepadConfig.setControllerType(CONTROLLER_TYPE_GAMEPAD);
  bleGamepadConfig.setHatSwitchCount(0);
  bleGamepadConfig.setWhichAxes(false, false, false, false, false, false, false, false);
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

  Wire.begin(PCF_PINS.sda, PCF_PINS.scl);
  pcf.begin();

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

  uint8_t last_state[SWITCHES_LIST_LENGTH];

  for (int i = 0; i < SWITCHES_LIST_LENGTH; ++i)
  {
    last_state[i] = digitalRead(SWITCHES_LIST[i]);
  }

  while (true)
  {
    if (xSemaphoreTake(switch_semaphore, portMAX_DELAY) == pdTRUE)
    {
      vTaskDelay(DEBOUNCE_TIME_MS / portTICK_PERIOD_MS);

      for (int i = 0; i < SWITCHES_LIST_LENGTH; ++i)
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
            bleGamepad.release(i + PCF_PINS_LENGTH + 1);
          }
          else
          {
            bleGamepad.press(i + PCF_PINS_LENGTH + 1);
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
}

void encoder_task(void *pv)
{
  Serial.println("Encoder task start");

  while (true)
  {
    if (xSemaphoreTake(encoder_semaphore, portMAX_DELAY) == pdTRUE)
    {
      uint32_t now = millis();

      for (size_t i = 0; i < ENCODERS_LIST_LENGTH; i++)
      {
        auto &pins = ENCODERS_LIST[i];
        auto &state = ENCODERS_STATE[i];

        if (now - state.last_isr_time < ENCODER_DEBOUNCE_MS)
        {
          continue;
        }

        state.last_isr_time = now;

        bool a = digitalRead(pins.a);
        bool b = digitalRead(pins.b);
        uint8_t curr_state = (a << 1) | b;

        if ((state.last_state == 0b00 && curr_state == 0b01) ||
            (state.last_state == 0b01 && curr_state == 0b11) ||
            (state.last_state == 0b11 && curr_state == 0b10) ||
            (state.last_state == 0b10 && curr_state == 0b00))
        {
          static bool state = false;

          Serial.println("Encoder decremented");
          if (state)
          {
            bleGamepad.press(20);
            bleGamepad.sendReport();
          }
          else
          {
            bleGamepad.release(20);
            bleGamepad.sendReport();
          }
          state = !state;
          // bleGamepad.press(20);
          // bleGamepad.sendReport();
          // bleGamepad.release(20);
          // bleGamepad.sendReport();
        }
        else if ((state.last_state == 0b00 && curr_state == 0b10) ||
                 (state.last_state == 0b10 && curr_state == 0b11) ||
                 (state.last_state == 0b11 && curr_state == 0b01) ||
                 (state.last_state == 0b01 && curr_state == 0b00))
        {
          Serial.println("Encoder incremented");
          bleGamepad.press(19);
          bleGamepad.sendReport();
          bleGamepad.release(19);
          bleGamepad.sendReport();
        }

        state.last_state = curr_state;
      }
    }
  }
}

void setup_encoders()
{
  encoder_semaphore = xSemaphoreCreateBinary();
  assert(encoder_semaphore);

  for (size_t i = 0; i < ENCODERS_LIST_LENGTH; i++)
  {
    pinMode(ENCODERS_LIST[i].a, INPUT_PULLUP);
    pinMode(ENCODERS_LIST[i].b, INPUT_PULLUP);

    ENCODERS_STATE[i].last_state = (digitalRead(ENCODERS_LIST[i].a) << 1) | digitalRead(ENCODERS_LIST[i].b);

    attachInterrupt(digitalPinToInterrupt(ENCODERS_LIST[i].a), []()
                    { xSemaphoreGiveFromISR(encoder_semaphore, nullptr); }, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODERS_LIST[i].b), []()
                    { xSemaphoreGiveFromISR(encoder_semaphore, nullptr); }, CHANGE);
  }

  xTaskCreate(encoder_task, "encoder_task", 1024*8, nullptr, 1, nullptr);
}

void setup()
{
  Serial.begin(115200);
  Serial.println("MSFS Controller app");

  setup_gamepad();
  setup_buttons();
  setup_switches();
  setup_encoders();
}

void loop()
{
  vTaskDelay(portMAX_DELAY);
}
