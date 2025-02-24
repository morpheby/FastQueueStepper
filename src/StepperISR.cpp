#include <stdint.h>
#include <algorithm>
#include "StepperISR.h"

int8_t StepperQueue::addQueueEntry(const queue_entry &cmd) {
  fasDisableInterrupts();

  if (!isReadyForCommands()) {
    return AQE_DEVICE_NOT_READY;
  }
  
  if (isQueueFull()) {
    return AQE_QUEUE_FULL;
  }

  // uint16_t period = cmd->ticks;
  // uint8_t steps = cmd->steps;
  // uint32_t command_rate_ticks = period;
  // if (steps > 1) {
  //   command_rate_ticks *= steps;
  // }
  // if (command_rate_ticks < MIN_CMD_TICKS) {
  //   return AQE_ERROR_TICKS_TOO_LOW;
  // }

  queue[queueWriteIdx] = cmd;

  queueWriteIdx = (queueWriteIdx + 1) % QUEUE_LEN;

  return AQE_OK;
}

uint32_t StepperQueue::ticksInQueue() {
  return hasTicksInQueue(UINT32_MAX);
}

uint32_t StepperQueue::hasTicksInQueue(uint32_t min_ticks) {
  fasDisableInterrupts();

  uint32_t ticks = 0;
  for (uint16_t rp = queueReadIdx; rp < queueWriteIdx && ticks < min_ticks; rp = (rp + 1) % QUEUE_LEN) {
    const queue_entry &entry = queue[rp];
    ticks += entry.ticks;
  }

  fasEnableInterrupts();
  return ticks;
}

void StepperQueue::_initVars() {
  queueReadIdx = 0;
  queueWriteIdx = 0;
#if defined(SUPPORT_ESP32)
  _isRunning = false;
  _nextCommandIsPrepared = false;
#endif
#if defined(SUPPORT_ESP32_RMT)
  _rmtStopped = true;
  _rmtQueueRunning = false;
#endif
}
