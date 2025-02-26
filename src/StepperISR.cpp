#include <stdint.h>
#include <algorithm>
#include "StepperISR.h"

int8_t StepperQueue::addQueueEntry(const queue_entry &cmd) {
  fasDisableInterrupts();
  
  if (isQueueFull()) {
    fasEnableInterrupts();
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
  fasEnableInterrupts();

  return AQE_OK;
}

uint32_t StepperQueue::ticksInQueue() const {
  return hasTicksInQueue(UINT32_MAX);
}

uint32_t StepperQueue::hasTicksInQueue(uint32_t min_ticks) const {
  fasDisableInterrupts();

  uint32_t ticks = 0;
  for (uint16_t rp = queueReadIdx; rp < queueWriteIdx && ticks < min_ticks; rp = (rp + 1) % QUEUE_LEN) {
    const queue_entry &entry = queue[rp];
    ticks += entry.ticks;
  }

  fasEnableInterrupts();
  return ticks;
}

uint32_t StepperQueue::stepsInQueue() const {
  fasDisableInterrupts();

  uint32_t steps = 0;
  uint32_t multiplier = currentDirection();
  for (uint16_t rp = queueReadIdx; rp < queueWriteIdx; rp = (rp + 1) % QUEUE_LEN) {
    const queue_entry &entry = queue[rp];
    if (entry.cmd == QueueCommand::TOGGLE_DIR) {
      multiplier = entry.ticks == QUEUE_ENTRY_DIRECTION_NEGATIVE ? -1 : 1;
    } else if (entry.cmd == QueueCommand::STEP) {
      steps += entry.steps * multiplier;
    }
  }

  fasEnableInterrupts();
  return steps;
}

int8_t StepperQueue::directionAfterLastEntry() const {
  fasDisableInterrupts();

  uint32_t direction = currentDirection();
  for (uint16_t rp = queueReadIdx; rp < queueWriteIdx; rp = (rp + 1) % QUEUE_LEN) {
    const queue_entry &entry = queue[rp];
    if (entry.cmd == QueueCommand::TOGGLE_DIR) {
      direction = entry.ticks == QUEUE_ENTRY_DIRECTION_NEGATIVE ? -1 : 1;
    }
  }

  fasEnableInterrupts();
  return direction;
}

void StepperQueue::resetQueue() {
  queueReadIdx = queueWriteIdx;
}

void StepperQueue::_initVars() {
  queueReadIdx = 0;
  queueWriteIdx = 0;
  _dirChangePending = 0;
  _currentDirection = 1;
#if defined(SUPPORT_ESP32)
  _isRunning = false;
  _nextCommandIsPrepared = false;
#endif
#if defined(SUPPORT_ESP32_RMT)
  _rmtQueueRunning = false;
  _cmdWriteIdx = 0;
#endif
}
