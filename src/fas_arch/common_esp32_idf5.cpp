
#include "common.h"

#if defined(ARDUINO_ARCH_ESP32) || defined(ESP_PLATFORM)
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;

void fasDisableInterrupts() {
    portENTER_CRITICAL(&lock);
}

void fasEnableInterrupts() {
    portEXIT_CRITICAL(&lock);
}

#endif
