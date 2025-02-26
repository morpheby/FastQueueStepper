#include <stdint.h>
#include <algorithm>
#include "StepperISR.h"

int8_t StepperQueue::addQueueEntry(const queue_entry &cmd) {
  fasDisableInterrupts();
  
  if (isQueueFull()) {
    fasEnableInterrupts();
    return AQE_QUEUE_FULL;
  }

  queue[queueWriteIdx] = cmd;

  queueWriteIdx = (queueWriteIdx + 1) % QUEUE_LEN;
  fasEnableInterrupts();

#ifdef SUPPORT_ESP32_RMT
  notifyRmt();
#endif

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

int32_t StepperQueue::stepsInQueue() const {
  fasDisableInterrupts();

  int32_t steps = 0;
  int32_t multiplier = currentDirection();
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

  int8_t direction = currentDirection();
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
  _cmdWriteIdx = 0;
#endif
}
