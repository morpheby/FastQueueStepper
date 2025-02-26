#include "StepperISR.h"

#if defined(SUPPORT_ESP32)

// Here are the global variables to interface with the interrupts
StepperQueue fas_queue[NUM_QUEUES];

StepperQueue *StepperQueue::getFreeQueue() {
  for (int i = 0; i < NUM_QUEUES; ++i) {
    if (fas_queue[i].isConnected()) continue;
    return &fas_queue[i];
  }
  return NULL;
}

void StepperQueue::connect(uint8_t step_pin, FastQueueStepperEngine *engine) {
  _engine = engine;
  _stepPin = step_pin;
#ifdef SUPPORT_ESP32_RMT
  connect_rmt();
#endif
}

void StepperQueue::disconnect() {
#ifdef SUPPORT_ESP32_RMT
  disconnect_rmt();
#endif
  _engine = NULL;
}

bool StepperQueue::isConnected() const {
#ifdef SUPPORT_ESP32_RMT
  return isConnected_rmt();
#endif
}

void StepperQueue::startQueue() {
#ifdef SUPPORT_ESP32_RMT
  startQueue_rmt();
  return;
#endif
}
void StepperQueue::forceStop() {
#ifdef SUPPORT_ESP32_RMT
  forceStop_rmt();
  return;
#endif
}

//*************************************************************************************************

bool StepperQueue::isValidStepPin(uint8_t step_pin) {
  gpio_drive_cap_t strength;
  esp_err_t res = gpio_get_drive_capability((gpio_num_t)step_pin, &strength);
  return res == ESP_OK;
}

//*************************************************************************************************
void StepperTask(void *parameter) {
  FastQueueStepperEngine *engine = (FastQueueStepperEngine *)parameter;
  TickType_t tm = xTaskGetTickCount();
  while (true) {
    engine->manageSteppers();
    const TickType_t delay_time =
        (engine->_delay_ms + portTICK_PERIOD_MS - 1) / portTICK_PERIOD_MS;
    xTaskDelayUntil(&tm, delay_time);
  }
}

void StepperQueue::adjustSpeedToStepperCount(uint8_t steppers) {
  max_speed_in_ticks = 64;
}

void fas_init_engine(FastQueueStepperEngine *engine, uint8_t cpu_core) {
#if ESP_IDF_VERSION_MAJOR == 4
#define STACK_SIZE 2000
#define PRIORITY configMAX_PRIORITIES
#else
#define STACK_SIZE 3000
#define PRIORITY (configMAX_PRIORITIES - 1)
#endif
  engine->_delay_ms = DELAY_MS_BASE;
  if (cpu_core > 1) {
    xTaskCreate(StepperTask, "StepperTask", STACK_SIZE, engine, PRIORITY, NULL);
  } else {
    xTaskCreatePinnedToCore(StepperTask, "StepperTask", STACK_SIZE, engine,
                            PRIORITY, NULL, cpu_core);
  }
}

#endif
