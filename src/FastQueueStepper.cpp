#include "FastQueueStepper.h"
#include "StepperISR.h"

// Here are the global variables to interface with the interrupts

// To realize the 1 Hz debug led
static uint8_t fas_ledPin = PIN_UNDEFINED;
static uint16_t fas_debug_led_cnt = 0;

// dynamic allocation seems to not work so well on avr
FastQueueStepper fas_stepper[MAX_STEPPER];

//*************************************************************************************************
//*************************************************************************************************
#if defined(SUPPORT_CPU_AFFINITY)
void FastQueueStepperEngine::init(uint8_t cpu_core) {
  _externalCallForPin = NULL;
  for (uint8_t i = 0; i < MAX_STEPPER; i++) {
    sharedDirectionPinList[i] = i;
    _steppers[i] = NULL;
  }
  fas_init_engine(this, cpu_core);
}
#else
void FastAccelStepperEngine::init() {
  _externalCallForPin = NULL;
  for (uint8_t i = 0; i < MAX_STEPPER; i++) {
    sharedDirectionPinList[i] = i;
    _steppers[i] = NULL;
  }
  fas_init_engine(this);
}
#endif

void FastQueueStepperEngine::setExternalCallForPin(
    bool (*func)(uint8_t pin, uint8_t value)) {
  _externalCallForPin = func;
}

void FastQueueStepperEngine::setStepperDirectionPin(FastQueueStepper *stepper, uint8_t dir_pin) {
  setStepperSharedPin(stepper, dir_pin, sharedDirectionPinList, &FastQueueStepper::getDirectionPin);
}

void FastQueueStepperEngine::setStepperEnablePin(FastQueueStepper *stepper, uint8_t en_pin) {
  setStepperSharedPin(stepper, en_pin, sharedEnablePinList, &FastQueueStepper::getEnablePin);
}

//*************************************************************************************************
void FastQueueStepperEngine::setStepperSharedPin(FastQueueStepper *stepper, uint8_t pin, uint8_t *list, uint8_t (FastQueueStepper::*pinFunc)() const) {
  int stepperIdx;
  for (stepperIdx = 0; stepperIdx < MAX_STEPPER; ++stepperIdx) if (stepper == _steppers[stepperIdx]) break;
  
  assert(stepperIdx != MAX_STEPPER);

  // Remove stepper from the circular linked list of steppers that share direction pins
  for (int otherStepperIdx = 0; otherStepperIdx < MAX_STEPPER; ++otherStepperIdx) {
    if (otherStepperIdx == stepperIdx) continue;

    if (list[otherStepperIdx] == stepperIdx) {
      std::swap(list[stepperIdx], list[otherStepperIdx]);
      break;
    }
  }

  if (pin == PIN_UNDEFINED) return;

  // Insert stepper into the circular linked list of steppers that share direction pins
  for (int otherStepperIdx = 0; otherStepperIdx < MAX_STEPPER; ++otherStepperIdx) {
    if (otherStepperIdx == stepperIdx) continue;

    FastQueueStepper *otherStepper = _steppers[otherStepperIdx];
    if (otherStepper == NULL) continue;
     
    if ((otherStepper->*pinFunc)() == pin) {
      std::swap(sharedDirectionPinList[stepperIdx], sharedDirectionPinList[otherStepperIdx]);
      break;
    }
  }
}

//*************************************************************************************************
bool FastQueueStepperEngine::_isValidStepPin(uint8_t step_pin) {
  return StepperQueue::isValidStepPin(step_pin);
}

//*************************************************************************************************
void FastQueueStepperEngine::changeDirectionIfNeeded() {  
  for (int stepperIdx = 0; stepperIdx < MAX_STEPPER; ++stepperIdx) {
    FastQueueStepper *stepper = _steppers[stepperIdx];
    if (stepper == NULL) continue;

    int8_t currentDirection = stepper->_queue->currentDirection() * (stepper->directionPinHighCountsUp() ? 1 : -1);
    int8_t wantedDirection = stepper->isQueueRunning() 
      ? stepper->_queue->directionChangePending() * (stepper->directionPinHighCountsUp() ? 1 : -1)
      : currentDirection;

    if (wantedDirection == 0) continue; // This stepper doesn't need direction change

    if (wantedDirection == currentDirection) {
      // The stepper has not been updated or other internal inconsistency.
      // Just start it directly
      stepper->_queue->startQueue();
      continue;
    }

    bool changeAllowed = true;

    // Use circular linked list to agree with other steppers in chain.
    fasDisableInterrupts(); // Lock any status changes while we investigate the chain
    for (int otherStepperIdx = sharedDirectionPinList[stepperIdx];
         otherStepperIdx != stepperIdx;
         otherStepperIdx = sharedDirectionPinList[otherStepperIdx]) {
      FastQueueStepper *otherStepper = _steppers[otherStepperIdx];
      if (otherStepper == NULL) continue;
      if (!otherStepper->isQueueRunning()) continue; // This stepper doesn't care about direction

      // Check direction
      int8_t otherWantedDirection = otherStepper->_queue->directionChangePending() * (otherStepper->directionPinHighCountsUp() ? 1 : -1);
      int8_t otherCurrentDirection = otherStepper->_queue->currentDirection() * (otherStepper->directionPinHighCountsUp() ? 1 : -1);
      if (wantedDirection == otherCurrentDirection) continue; // This stepper already agrees
      if (wantedDirection == otherWantedDirection) continue; // This stepper will agree after it is updated 
      
      // This stepper is blocking the change of direction. Skip this stepper
      changeAllowed = false;
      break;
    }
    fasEnableInterrupts();

    if (!changeAllowed) continue;

    if (stepper->getDirectionPin() & PIN_EXTERNAL_FLAG) {
      _externalCallForPin(stepper->getDirectionPin() ^ PIN_EXTERNAL_FLAG, wantedDirection > 0 ? 1 : 0);
    } else {
      LL_SET_PIN(stepper->getDirectionPin(), wantedDirection > 0 ? 1 : 0);
    }

    stepper->_queue->startQueue();

    // Use circular linked list to update other steppers in chain.
    for (int otherStepperIdx = sharedDirectionPinList[stepperIdx];
         otherStepperIdx != stepperIdx;
         otherStepperIdx = sharedDirectionPinList[otherStepperIdx]) {
      FastQueueStepper *otherStepper = _steppers[otherStepperIdx];
      if (otherStepper == NULL) continue;

      otherStepper->_queue->setDirection(wantedDirection * (otherStepper->directionPinHighCountsUp() ? 1 : -1));

      if (!otherStepper->isQueueRunning()) continue; // The stepper is not awaiting direction change

      otherStepper->_queue->startQueue();
    }
  }
}

void FastQueueStepperEngine::autoEnableDisableIfNeeded() {
  for (int stepperIdx = 0; stepperIdx < MAX_STEPPER; ++stepperIdx) {
    FastQueueStepper *stepper = _steppers[stepperIdx];
    if (stepper == NULL) continue;

    bool neededStatus = (!stepper->enablePinHighIsActive()) ^ (stepper->isAutoEnable()
                                                               ? (!stepper->_queue->isQueueEmpty())
                                                               : stepper->isEnabled());

    bool changeAllowed = true;

    // Use circular linked list to agree with other steppers in chain.
    fasDisableInterrupts(); // Lock any status changes while we investigate the chain
    for (int otherStepperIdx = sharedEnablePinList[stepperIdx];
         otherStepperIdx != stepperIdx;
         otherStepperIdx = sharedEnablePinList[otherStepperIdx]) {
      FastQueueStepper *otherStepper = _steppers[otherStepperIdx];
      if (otherStepper == NULL) continue;
      
      bool otherNeededStatus = (!otherStepper->enablePinHighIsActive()) ^ (otherStepper->isAutoEnable()
                                                                           ? (!otherStepper->_queue->isQueueEmpty())
                                                                           : otherStepper->isEnabled());
      bool otherWillBeEnabled = (!otherStepper->enablePinHighIsActive()) ^ neededStatus;

      if (otherNeededStatus == neededStatus) continue;
      if (otherWillBeEnabled) continue; // We don't care if a stepper gets enabled instead of being disabled
      
      // This stepper is blocking the change of direction. Skip this stepper
      changeAllowed = false;
      break;
    }
    fasEnableInterrupts();

    if (!changeAllowed) continue;

    if (stepper->getEnablePin() & PIN_EXTERNAL_FLAG) {
      _externalCallForPin(stepper->getEnablePin() ^ PIN_EXTERNAL_FLAG, neededStatus);
    } else {
      LL_SET_PIN(stepper->getEnablePin(), neededStatus);
    }

    // Use circular linked list to update other steppers in chain.
    for (int otherStepperIdx = sharedDirectionPinList[stepperIdx];
         otherStepperIdx != stepperIdx;
         otherStepperIdx = sharedDirectionPinList[otherStepperIdx]) {
      FastQueueStepper *otherStepper = _steppers[otherStepperIdx];
      if (otherStepper == NULL) continue;

      otherStepper->setEnabled(neededStatus ^ (!otherStepper->enablePinHighIsActive()));
    }
  }
}

void FastQueueStepperEngine::detachStepper(FastQueueStepper *stepper) {
  int stepperIdx;
  for (stepperIdx = 0; stepperIdx < MAX_STEPPER; ++stepperIdx) if (stepper == _steppers[stepperIdx]) break;
  
  assert(stepperIdx != MAX_STEPPER);

  stepper->_queue->forceStop();
  StepperQueue *q = stepper->_queue;
  stepper->_queue = NULL;
  stepper->_queue->disconnect();

  setStepperDirectionPin(stepper, PIN_UNDEFINED);
  setStepperEnablePin(stepper, PIN_UNDEFINED);

  _steppers[stepperIdx] = NULL;
}

//*************************************************************************************************
#if !defined(SUPPORT_SELECT_DRIVER_TYPE)
FastQueueStepper* FastQueueStepperEngine::stepperConnectToPin(uint8_t step_pin)
#else
#error "Not implemented yet"
#endif
{
  // Check if already connected
  for (uint8_t i = 0; i < MAX_STEPPER; i++) {
    FastQueueStepper* s = _steppers[i];
    if (s) {
      if (s->getStepPin() == step_pin) {
        return NULL;
      }
    }
  }
  if (!_isValidStepPin(step_pin)) {
    return NULL;
  }

  StepperQueue *queue = StepperQueue::getFreeQueue();
  if (queue == NULL) return NULL;

  queue->connect(step_pin, this);

  FastQueueStepper *s = NULL;
  for (int stepperIdx = 0; stepperIdx < MAX_STEPPER; ++stepperIdx) {
    int j;
    for (j = 0; j < MAX_STEPPER; ++j) if (&fas_stepper[stepperIdx] == _steppers[j]) break;
    if (j != MAX_STEPPER) continue;
    s = &fas_stepper[stepperIdx];
    break;
  }

  if (s == NULL) return NULL;
  
  s->init(this, queue);
  
  int stepperCount = 0;
  for (uint8_t i = 0; i < MAX_STEPPER; i++) {
    FastQueueStepper* sx = _steppers[i];
    if (sx) {
      ++stepperCount;
    }
  }
  for (uint8_t i = 0; i < MAX_STEPPER; i++) {
    FastQueueStepper* sx = _steppers[i];
    if (sx) {
      sx->_queue->adjustSpeedToStepperCount(stepperCount);
    }
  }
  return s;
}
//*************************************************************************************************
void FastQueueStepperEngine::setDebugLed(uint8_t ledPin) {
  fas_ledPin = ledPin;
  PIN_OUTPUT(fas_ledPin, LOW);
}

//*************************************************************************************************
void FastQueueStepperEngine::manageSteppers() {
#ifdef DEBUG_LED_HALF_PERIOD
  if (fas_ledPin != PIN_UNDEFINED) {
    fas_debug_led_cnt++;
    if (fas_debug_led_cnt == DEBUG_LED_HALF_PERIOD) {
      digitalWrite(fas_ledPin, HIGH);
    }
    if (fas_debug_led_cnt == 2 * DEBUG_LED_HALF_PERIOD) {
      digitalWrite(fas_ledPin, LOW);
      fas_debug_led_cnt = 0;
    }
  }
#endif
  for (uint8_t i = 0; i < MAX_STEPPER; i++) {
    FastQueueStepper* s = _steppers[i];
    if (s == NULL) continue;

    // TODO: Execute planner
  }

  autoEnableDisableIfNeeded();
  changeDirectionIfNeeded();
}

//*************************************************************************************************
int8_t FastQueueStepper::addQueueEntry(const stepper_command_s &cmd) {
  int result = AQE_OK;

  if (cmd.steps == 0 && cmd.ticks == 0) {
    // STOP command
    queue_entry e {
      .cmd = QueueCommand::STOP,
      .steps = 0,
      .ticks = 0,
    };
    result = _queue->addQueueEntry(e);
    return result;
  }

  if (cmd.ticks < _queue->getMaxSpeedInTicks()) {
    return AQE_ERROR_TICKS_TOO_LOW;
  }

  // Check how much stepper commands we will need
  const uint32_t stepCommandCount = std::max((uint32_t) std::abs(cmd.steps) / UINT8_MAX + (std::abs(cmd.steps) % UINT8_MAX != 0 ? 1 : 0),
                                             cmd.ticks / QUEUE_ENTRY_MAX_TICKS + (cmd.ticks % QUEUE_ENTRY_MAX_TICKS != 0 ? 1 : 0));

  if (stepCommandCount > _queue->queueSize() - 1) {
    return AQE_ERROR_MOVE_TOO_LARGE;
  }

  if (_queue->queueEntriesAvailable() < stepCommandCount) {
    return AQE_QUEUE_FULL;
  }
 
  uint32_t delay = 0;
  if ((cmd.steps < 0 && _queue->currentDirection() > 0) || (cmd.steps > 0 && _queue->currentDirection() < 0)) {
    // We need to change direction

    if (getDirectionPin() == PIN_UNDEFINED) {
      return AQE_ERROR_NO_DIR_PIN_TO_TOGGLE;
    }

    if (_queue->queueEntriesAvailable() < stepCommandCount + 2) {
      // We will not fit TOGGLE_DIR + DELAY + STEP(s)
      return AQE_QUEUE_FULL;
    }

    queue_entry e {
      .cmd = QueueCommand::TOGGLE_DIR,
      .steps = 0,
      .ticks = cmd.steps < 0 ? QUEUE_ENTRY_DIRECTION_NEGATIVE : QUEUE_ENTRY_DIRECTION_POSITIVE,
    };

    result = _queue->addQueueEntry(e);
    if (result != AQE_OK) return result;

    delay = _dir_change_delay_ticks;
  }

  if (cmd.steps == 0) {
    // The command is a delay, so merge it with the delay we already have
    delay = std::max(cmd.ticks, delay);
  }

  if (isAutoEnable() && !isEnabled()) {
    // if on delay is defined, fill queue with required amount of pauses before
    // the first step
    
    delay = std::max(_on_delay_ticks, delay);
    
    if (delay == 0 && isQueueRunning()) {
      // Since queue is already running and no delay is given, we need to force-enable stepper
      fasDisableInterrupts(); // Don't allow stepper to remove this command before we have it enabled
      queue_entry e {
        .cmd = QueueCommand::STEP,
        .steps = 0,
        .ticks = 0,
      };
      result = _queue->addQueueEntry(e);
      _engine->autoEnableDisableIfNeeded();
      fasEnableInterrupts();
      if (result != AQE_OK) return result;
      // By this point the stepper could be not enabled only if other stepper is blocking it
      if (!isEnabled()) {
        return AQE_WAIT_FOR_ENABLE_PIN_ACTIVE;
      }
    }
  }
  
  if (delay > 0) {
    // Add all delays to the queue combined

    const uint32_t delayCommandCount = delay / QUEUE_ENTRY_MAX_TICKS + (delay % QUEUE_ENTRY_MAX_TICKS != 0 ? 1 : 0);

    if (_queue->queueEntriesAvailable() < stepCommandCount + delayCommandCount) {
      // TOGGLE_DIR if present is already in queue, so we are only checking if we can put DELAY(s) + STEP(s)
      return AQE_QUEUE_FULL;
    }

    while (delay > 0) {
      uint16_t ticks_u16;
      if (delay > ((uint32_t) QUEUE_ENTRY_MAX_TICKS) * 2) {
        ticks_u16 = QUEUE_ENTRY_MAX_TICKS;
      } else if (delay <= QUEUE_ENTRY_MAX_TICKS) {
        ticks_u16 = delay;
      } else {
        ticks_u16 = delay / 2;
      }
      
      queue_entry e {
        .cmd = QueueCommand::STEP,
        .steps = 0,
        .ticks = ticks_u16,
      };

      result = _queue->addQueueEntry(e);
      if (result != AQE_OK) return result;
      delay -= ticks_u16;
    }
  }
  
  if (cmd.steps != 0) {
    // Delay was already processed separately, so we only process step command here
    uint32_t ticksDone = cmd.ticks, ticksRemainder = 0;
    uint32_t stepsDone = cmd.steps, stepsRemainder = 0;

    while (ticksDone < cmd.ticks || stepsDone < cmd.steps) {
      uint32_t ticksInEntry = (cmd.ticks + ticksRemainder) / stepCommandCount;
      uint32_t stepsInEntry = (cmd.steps + stepsRemainder) / stepCommandCount;
      ticksRemainder = (cmd.ticks + ticksRemainder) % stepCommandCount;
      stepsRemainder = (cmd.steps + stepsRemainder) % stepCommandCount;

      assert(ticksInEntry <= QUEUE_ENTRY_MAX_TICKS);
      assert(stepsInEntry <= UINT8_MAX);
      
      queue_entry e {
        .cmd = QueueCommand::STEP,
        .steps = (uint8_t) std::abs(cmd.steps),
        .ticks = (uint16_t) cmd.ticks,
      };

      result = _queue->addQueueEntry(e);
      if (result != AQE_OK) return result;

      stepsDone += e.steps;
      ticksDone += e.ticks;
    }
  }
  
  _engine->autoEnableDisableIfNeeded();

  return result;
}

void FastQueueStepper::init(FastQueueStepperEngine* engine, StepperQueue *queue) {
  _engine = engine;
  _autoEnable = false;
  _dir_change_delay_ticks = 0;
  _on_delay_ticks = 0;
  _dirHighCountsUp = true;
  _dirPin = PIN_UNDEFINED;
  _enablePin = PIN_UNDEFINED;
  _enablePinHighIsActive = true;
  _currentPosition = 0;

#if defined(SUPPORT_ESP32_PULSE_COUNTER) && (ESP_IDF_VERSION_MAJOR == 5)
  _attached_pulse_unit = NULL;
#endif
}

void FastQueueStepper::setDirectionPin(uint8_t dirPin, bool dirHighCountsUp,
                                       uint32_t dir_change_delay_us) {
  _engine->setStepperDirectionPin(this, dirPin);
  _dirPin = dirPin;
  _dirHighCountsUp = dirHighCountsUp;
  
  _engine->changeDirectionIfNeeded();
  
  if (dir_change_delay_us != 0) {
    _dir_change_delay_ticks = US_TO_TICKS(dir_change_delay_us);
  } else {
    _dir_change_delay_ticks = 0;
  }
}

void FastQueueStepper::setEnablePin(uint8_t enablePin,
                                    bool highIsActive) {
  _engine->setStepperEnablePin(this, enablePin);
  
  _enablePin = enablePin;
  _enablePinHighIsActive = highIsActive;

  _engine->autoEnableDisableIfNeeded();
  
}

void FastQueueStepper::setDelayToEnable(uint32_t delay_us) {
  uint32_t delay_ticks = US_TO_TICKS(delay_us);
  _on_delay_ticks = delay_ticks;
}

int8_t FastQueueStepper::stop() {
  queue_entry e {
    .cmd = QueueCommand::STOP,
    .steps = 0,
    .ticks = 0,
  };
  return _queue->addQueueEntry(e);
}

void FastQueueStepper::forceStopAndNewPosition(int32_t new_pos) {
  _queue->forceStop();

  // We set position here, because we can not know for certain what is the current position...  
  setPositionAfterCommandsCompleted(new_pos);
}

#if defined(SUPPORT_ESP32_PULSE_COUNTER)
void FastQueueStepper::forceStop() {
  _queue->forceStop();

  // Synchronize PCNT and internal counter
  setPositionAfterCommandsCompleted(getCurrentPosition());
}
#endif

void FastQueueStepper::disableOutputs() {
  if (isAutoEnable()) {
    return;
  }
  _enabled = false;
  _engine->autoEnableDisableIfNeeded();
}

void FastQueueStepper::enableOutputs() {
  if (isAutoEnable()) {
    return;
  }
  _enabled = true;
  _engine->autoEnableDisableIfNeeded();
}

int32_t FastQueueStepper::getCurrentPosition() {
  _currentPosition += _queue->readAndClearCurrentPosition();
#if defined(SUPPORT_ESP32_PULSE_COUNTER)
  if (pulseCounterAttached()) {
    return pcnt_offset + readPulseCounter();
  }
#endif
  return _currentPosition;
}

int32_t FastQueueStepper::getPositionAfterCommandsCompleted() {
  int32_t position;
  fasDisableInterrupts();
  _currentPosition += _queue->readAndClearCurrentPosition();
  position = _currentPosition + _queue->stepsInQueue();
  fasEnableInterrupts();
  return position;
}

#if SUPPORT_UNSAFE_ABS_SPEED_LIMIT_SETTING == 1
void FastQueueStepper::setAbsoluteSpeedLimit(uint16_t max_speed_in_ticks) {
  _queue->setAbsoluteSpeedLimit(max_speed_in_ticks);
}
#endif

void FastQueueStepper::setCurrentPosition(int32_t new_pos) {
  fasDisableInterrupts();
#if defined(SUPPORT_ESP32_PULSE_COUNTER)
  if (pulseCounterAttached()) {
    pcnt_offset = new_pos;
    clearPulseCounter();
  }
#endif
  _queue->readAndClearCurrentPosition();
  _currentPosition = new_pos;
  fasEnableInterrupts();
}

void FastQueueStepper::setPositionAfterCommandsCompleted(int32_t new_pos) {
  fasDisableInterrupts();
  _currentPosition += _queue->readAndClearCurrentPosition();
  int32_t offset = _queue->stepsInQueue();
#if defined(SUPPORT_ESP32_PULSE_COUNTER)
  if (pulseCounterAttached()) {
    int32_t currentPcntPosition = pcnt_offset + readPulseCounter();
    pcnt_offset = new_pos + currentPcntPosition - _currentPosition - offset;
    clearPulseCounter();
  }
#endif
  _currentPosition = new_pos - offset;
  fasEnableInterrupts();
}
int8_t FastQueueStepper::performOneStep(bool count_up, bool blocking) {
  int8_t result = addQueueEntry({.ticks = 16, .steps = count_up ? 1 : -1});
  if (result != AQE_OK) return result;
  if (blocking) {
    // Make sure queue is running
    startQueue();
    while (_queue->hasTicksInQueue(1)) {
      delay(4);
    }
  }
  return result;
}

int8_t FastQueueStepper::forwardStep(bool blocking) {
  return performOneStep(true, blocking);
}

int8_t FastQueueStepper::backwardStep(bool blocking) {
  return performOneStep(false, blocking);
}

bool FastQueueStepper::isQueueEmpty() const {
  return _queue->isQueueEmpty();\
}

bool FastQueueStepper::isQueueFull() const {
  return _queue->isQueueFull();
}

void FastQueueStepper::startQueue() {
  if (!_queue->isRunning())
    _queue->startQueue();
}

void FastQueueStepper::resetQueue() {
  if (!_queue->isRunning())
    _queue->resetQueue();
}


uint32_t FastQueueStepper::ticksInQueue() const {
  return _queue->ticksInQueue();
}

bool FastQueueStepper::hasTicksInQueue(uint32_t min_ticks) const {
  return _queue->hasTicksInQueue(min_ticks);
}


bool FastQueueStepper::isRunning() const {
  return isQueueRunning() && _queue->hasTicksInQueue(1);
}

bool FastQueueStepper::isQueueRunning() const {
  return _queue->isRunning();
}

int8_t FastQueueStepper::directionAfterCommandsCompleted() const {
  return _queue->directionAfterLastEntry();
}

uint8_t FastQueueStepper::getStepPin() const {
  return _queue->getStepPin();
}
