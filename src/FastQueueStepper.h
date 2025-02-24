#ifndef FASTQUEUESTEPPER_H
#define FASTQUEUESTEPPER_H
#include <stdint.h>
#include "fas_arch/common.h"

class FastQueueStepper;

class FastQueueStepperEngine {
 public:

#if defined(SUPPORT_CPU_AFFINITY)
  // In a multitasking and multicore system like ESP32, the steppers are
  // controlled by a continuously running task. This task can be fixed to one
  // CPU core with this modified init()-call. ESP32 implementation detail: For
  // values 0 and 1, xTaskCreatePinnedToCore() is used, or else xTaskCreate()
  void init(uint8_t cpu_core = 255);
#else
  void init();
#endif

  // ### Creation of FastAccelStepper
  //
  // Using a call to `stepperConnectToPin()` a FastAccelStepper instance is
  // created. This call tells the stepper, which step pin to use. As the
  // hardware may have limitations - e.g. no stepper resources anymore, or the
  // step pin cannot be used, then NULL is returned. So it is advised to check
  // the return value of this call.
#if !defined(SUPPORT_SELECT_DRIVER_TYPE)
  FastAccelStepper* stepperConnectToPin(uint8_t step_pin);
#endif

#if defined(SUPPORT_TASK_RATE_CHANGE)
  // For e.g. esp32 the repetition rate of the stepper task can be changed.
  // The default delay is 4ms.
  //
  // The steppertask is looping with:
  //       manageSteppers()
  //       delay()
  //
  // The actual repetition rate of the stepper task is delay + execution time of
  // manageSteppers()
  //
  // This function is primary of interest in conjunction with
  // setForwardPlanningTimeInMs(). If the delay is larger then forward planning
  // time, then the stepper queue will always run out of commands, which lead to
  // a sudden stop of the motor. If the delay is 0, then the stepper task will
  // constantly looping, which may lead to the task blocking other tasks.
  // Consequently, this function is intended for advanced users.
  //
  // There is not planned to test this functionality, because automatic testing
  // is only available for avr devices and those continue to use fixed 4ms rate.
  //
  // Please be aware, that the configured tick rate aka portTICK_PERIOD_MS is
  // relevant. Apparently, arduino-esp32 has FreeRTOS configured to have a
  // tick-rate of 1000Hz
  inline void task_rate(uint8_t delay_ms) { _delay_ms = delay_ms; };
  uint8_t _delay_ms;
#endif

  // Comments to valid pins:
  //
  // clang-format off
  // | Device          | Comment                                                                                           |
  // |:----------------|:--------------------------------------------------------------------------------------------------|
  // | ESP32           | Every output capable GPIO can be used                                                             |
  // | ESP32S2         | Every output capable GPIO can be used                                                             |
  // | Atmega168/328/p | Only the pins connected to OC1A and OC1B are allowed                                              |
  // | Atmega2560      | Only the pins connected to OC4A, OC4B and OC4C are allowed.                                       |
  // | Atmega32u4      | Only the pins connected to OC1A, OC1B and OC1C are allowed                                        |
  // | Atmel SAM       | This can be one of each group of pins: 34/67/74/35, 17/36/72/37/42, 40/64/69/41, 9, 8/44, 7/45, 6 |
  // clang-format on

  // ## External Pins
  //
  // If the direction/enable pins are e.g. connected via external HW (shift
  // registers), then an external callback function can be supplied. The
  // supplied value is either LOW or HIGH. The return value shall be the status
  // of the pin (false for LOW or true for HIGH). If returned value and supplied
  // value do not match, the stepper does not continue, but calls this function
  // again.
  //
  // This function is called from cyclic task/interrupt with 4ms rate, which
  // creates the commands to put into the command queue. Thus the supplied
  // function should take much less time than 4ms. Otherwise there is risk, that
  // other running steppers are running out of commands in the queue. If this
  // takes longer, then the function should be offloaded and return the new
  // status, after the pin change has been successfully completed.
  //
  // The callback has to be called on the FastAccelStepperEngine.
  // See examples/ExternalCall
  //
  // Stepperpins (enable or direction), which should use this external callback,
  // need to be or'ed with PIN_EXTERNAL_FLAG ! FastAccelStepper uses this flag
  // to determine, if a pin is external or internal.
  void setExternalCallForPin(bool (*func)(uint8_t pin, uint8_t value));

  // ### Debug LED
  //
  // If blinking of a LED is required to indicate, the stepper controller is
  // still running, then the port. to which the LED is connected, can be told to
  // the engine. The periodic task will let the associated LED blink with 1 Hz
  void setDebugLed(uint8_t ledPin);

  /* This should be only called from ISR or stepper task. So do not call it */
  void manageSteppers();

 private:
  bool isDirPinBusy(uint8_t dirPin, uint8_t except_stepper);

  uint8_t _stepper_cnt;
  FastAccelStepper* _stepper[MAX_STEPPER];

  bool _isValidStepPin(uint8_t step_pin);
  bool (*_externalCallForPin)(uint8_t pin, uint8_t value);

  friend class FastAccelStepper;
};

//
// ## Timing values - Architecture dependent
//
// ### AVR
// |VARIABLE         | Value       | Unit                    |
// |:----------------|------------:|:------------------------|
// |TICKS_PER_S      | 16_000_000  | [ticks/s]               |
// |MIN_CMD_TICKS    |  640        | [1/TICKS_PER_S seconds] |
// |MIN_DIR_DELAY_US |   40        | [µs]                    |
// |MAX_DIR_DELAY_US | 4095        | [µs]                    |
//
// ### ESP32
// |VARIABLE         | Value       | Unit                    |
// |:----------------|------------:|:------------------------|
// |TICKS_PER_S      | 16_000_000  | [ticks/s]               |
// |MIN_CMD_TICKS    | 3200        | [1/TICKS_PER_S seconds] |
// |MIN_DIR_DELAY_US |  200        | [µs]                    |
// |MAX_DIR_DELAY_US | 4095        | [µs]                    |
//
// ### SAM DUE
// |VARIABLE         | Value       | Unit                    |
// |:----------------|------------:|:------------------------|
// |TICKS_PER_S      | 21_000_000  | [ticks/s]               |
// |MIN_CMD_TICKS    | 4200        | [1/TICKS_PER_S seconds] |
// |MIN_DIR_DELAY_US |  200        | [µs]                    |
// |MAX_DIR_DELAY_US | 3120        | [µs]                    |
//
// # FastAccelStepper

#define MAX_ON_DELAY_TICKS ((uint32_t)(65535 * (QUEUE_LEN - 1)))

#define PIN_UNDEFINED 255
#define PIN_EXTERNAL_FLAG 128

class FastQueueStepper {
 private:
  void init(FastQueueStepperEngine* engine, uint8_t num, uint8_t step_pin);

 public:
  // ## Step Pin
  // step pin is defined at creation. Here can retrieve the pin
  uint8_t getStepPin();

  // ## Direction Pin
  // if direction pin is connected, call this function.
  //
  // If the pin number is >= 128, then the direction pin is assumed to be
  // external and the external callback function (set by
  // `setExternalCallForPin()`) is used to set the pin. For direction pin, this
  // is implemented for esp32 and its supported derivates, and avr and its
  // derivates except atmega32u4
  //
  // For slow driver hardware the first step after any polarity change of the
  // direction pin can be delayed by the value dir_change_delay_us. The allowed
  // range is MIN_DIR_DELAY_US and MAX_DIR_DELAY_US. The special value of 0
  // means, that no delay is added. Values 1 up to MIN_DIR_DELAY_US will be
  // clamped to MIN_DIR_DELAY_US. Values above MAX_DIR_DELAY_US will be clamped
  // to MAX_DIR_DELAY_US. For external pins, dir_change_delay_us is ignored,
  // because the mechanism applied for external pins provides already pause
  // in the range of ms or more.
  void setDirectionPin(uint8_t dirPin, bool dirHighCountsUp = true,
                       uint16_t dir_change_delay_us = 0);
  inline uint8_t getDirectionPin() { return _dirPin; }
  inline bool directionPinHighCountsUp() { return _dirHighCountsUp; }

  // ## Enable Pin
  // if enable pin is connected, then use this function.
  //
  // If the pin number is >= 128, then the enable pin is assumed to be
  // external and the external callback function (set by
  // `setExternalCallForPin()`) is used to set the pin.
  //
  // In case there are two enable pins: one low and one high active, then
  // these calls are valid and both pins will be operated:
  //    setEnablePin(pin1, true);
  //    setEnablePin(pin2, false);
  // If pin1 and pin2 are same, then the last call will be used.
  void setEnablePin(uint8_t enablePin, bool low_active_enables_stepper = true);
  inline uint8_t getEnablePinHighActive() { return _enablePinHighActive; }
  inline uint8_t getEnablePinLowActive() { return _enablePinLowActive; }

  // using enableOutputs/disableOutputs the stepper can be enabled and disabled
  // For a running motor with autoEnable set, disableOutputs() will return false
  bool enableOutputs();   // returns true, if enabled
  bool disableOutputs();  // returns true, if disabled

  // In auto enable mode, the stepper is enabled before stepping and disabled
  // afterwards. The delay from stepper enabled till first step and from
  // last step to stepper disabled can be separately adjusted.
  // The delay from enable to first step is done in ticks and as such is limited
  // to MAX_ON_DELAY_TICKS, which translates approximately to 120ms for
  // esp32 and 60ms for avr at 16 MHz). The delay till disable is done in period
  // interrupt/task with 4 or 10 ms repetition rate and as such is with several
  // ms jitter.
  void setAutoEnable(bool auto_enable);
  int8_t setDelayToEnable(uint32_t delay_us);
  void setDelayToDisable(uint16_t delay_ms);
#define DELAY_OK 0
#define DELAY_TOO_LOW -1
#define DELAY_TOO_HIGH -2

  // ## Stepper Position
  // Retrieve the current position of the stepper
  int32_t getCurrentPosition();
  
  // ## ESP32 only: Free pulse counter
  // These functions are only available on esp32.
  //
  // PCNT allows to output immediate stepper position, which may be needed for exact
  // and realtime stepper synchronization and step planning.
  //
  // Pulse counter 6 and 7 are not used by the stepper library and are judged as
  // available. If only five steppers are defined, then 5 gets available. If
  // four steppers are defined, then 4 is usable,too.
  //
  // Update for idf5 version:
  // Pulse counter counters all steps performed from the start of movement
  //
#if defined(SUPPORT_ESP32_PULSE_COUNTER) && (ESP_IDF_VERSION_MAJOR == 5)
  bool enablePulseCounter();
  inline bool pulseCounterAttached() { return _attached_pulse_unit != NULL; }
  void disablePulseCounter();
#endif

  // Set the current position of the stepper - either in standstill or while moving
  void setCurrentPosition(int32_t new_pos);

  // ## Stepper running status
  // is true while the stepper is running
  bool isRunning();

  void setAbsoluteSpeedLimit(uint16_t max_speed_in_ticks);

  // ### forwardStep() and backwardStep()
  // forwardStep()/backwardstep() can be called, while stepper is not moving
  // If stepper is moving, this is a no-op.
  // backwardStep() is a no-op, if no direction pin defined
  // It will immediately let the stepper perform one single step.
  // If blocking = true, then the routine will wait till isRunning() is false
  void forwardStep(bool blocking = false);
  void backwardStep(bool blocking = false);

  // ### forceStop()
  // Abruptly stop the running stepper.
  //
  // This command only places the STOP command into the queue, 
  // so it will only be processed when the queue reaches this point.
  //
  // If you need to stop the stepper immediately, use forceStopAndNewPosition().
  //
  // Note that there are two (three on ESP32) queues:
  // 1. Planner queue (this is controlled in the planner, not here). It
  //    the planner keeps outputting steps, then they will be added to the
  //    queue, but the queue will only process them in the running state
  // 2. Stepper queue. Has a list of operations that are available for ISR
  //    routine to create actual steps for the stepper. Usually ~20 ms full.
  // 3. (ESP32 only) RMT queue. This is internal to the MCU, and can only
  //    be drained fully or not. Should not be too large but still relevant
  //
  // forceStopAndNewPosition() empties 2 and 3.
  // forceStop() only adds a new entry to 2 to indicate that no further queue
  // processing should be done until the queue is started again with
  // startQueue();
  void forceStop();

  // abruptly stop the running stepper without deceleration.
  //
  // Same as forceStop(), but completely empties the running queue allowing
  // for a near immediate stop.
  void forceStopAndNewPosition(int32_t new_pos);

  // ### Task planning
  // The stepper task adds commands to the stepper queue until
  // either at least two commands are planned, or the commands
  // cover sufficient time into the future. Default value for that time is 20ms.
  //
  // The stepper task is cyclically executed every ~4ms.
  // Especially for avr, the step interrupts puts a significant load on the uC,
  // so the cyclical stepper task can even run for 2-3 ms. On top of that,
  // other interrupts caused by the application could increase the load even
  // further.
  //
  // Consequently, the forward planning should fill the queue for ideally two
  // cycles, this means 8ms. This means, the default 20ms provide a sufficient
  // margin and even a missed cycle is not an issue.
  //
  // The drawback of the 20ms is, that any change in speed/acceleration are
  // added after those 20ms and for an application, requiring fast reaction
  // times, this may impact the expected performance.
  //
  // Due to this the forward planning time can be adjusted with the following
  // API call for each stepper individually.
  //
  // Attention:
  // - This is only for advanced users: no error checking is implemented.
  // - Only change the forward planning time, if the stepper is not running.
  // - Too small values bear the risk of a stepper running at full speed
  // suddenly stopping
  //   due to lack of commands in the queue.
  inline void setForwardPlanningTimeInMs(uint8_t ms) {
    _forward_planning_in_ticks = ms;
    _forward_planning_in_ticks *= TICKS_PER_S / 1000;  // ticks per ms
  }

  int8_t addQueueEntry(const struct stepper_command_s* cmd);

  // Return codes for addQueueEntry
  //    positive values mean, that caller should retry later
#define AQE_OK 0
#define AQE_QUEUE_FULL 1
#define AQE_DIR_PIN_IS_BUSY 2
#define AQE_WAIT_FOR_ENABLE_PIN_ACTIVE 3
#define AQE_DEVICE_NOT_READY 4
#define AQE_ERROR_TICKS_TOO_LOW -1
#define AQE_ERROR_EMPTY_QUEUE_TO_START -2
#define AQE_ERROR_NO_DIR_PIN_TO_TOGGLE -3

  // ### check functions for command queue being empty, full or running.
  bool isQueueEmpty();
  bool isQueueFull();

  // Check if the queue is currently running (on) or not.
  //
  // Unlike in FAS, the queue does not stop running when it is dry,
  // so you need to stop queue manually if you don't need it in the
  // running state.
  bool isQueueRunning();

  void startQueue();

  // ### functions to get the fill level of the queue
  //
  // To retrieve the forward planning time in the queue, ticksInQueue()
  // can be used. It sums up all ticks of the not yet processed commands.
  // For commands defining pauses, the summed up value is entry.ticks.
  // For commands with steps, the summed up value is entry.steps*entry.ticks
  uint32_t ticksInQueue();

  // This function can be used to check, if the commands in the queue
  // will last for <min_ticks> ticks. This is again without the
  // currently processed command.
  bool hasTicksInQueue(uint32_t min_ticks);

  // This function allows to check the number of commands in the queue.
  // This is including the currently processed command.
  uint8_t queueEntries();

  // Get the future position of the stepper after all commands in queue are
  // completed
  int32_t getPositionAfterCommandsCompleted();

  // Set the future position of the stepper after all commands in queue are
  // completed
  void setPositionAfterCommandsCompleted(int32_t new_pos);

  // These functions allow to detach and reAttach a step pin for other use.
  // Pretty low level, use with care or not at all
  void detachFromPin();
  void reAttachToPin();

 private:
  void performOneStep(bool count_up, bool blocking = false);
#ifdef SUPPORT_EXTERNAL_DIRECTION_PIN
  bool externalDirPinChangeCompletedIfNeeded();
#endif
  void updateAutoDisable();
  void blockingWaitForForceStopComplete();
  bool needAutoDisable();
  bool agreeWithAutoDisable();
  bool usesAutoEnablePin(uint8_t pin);

  FastQueueStepperEngine* _engine;
  uint8_t _stepPin;
  uint8_t _dirPin;
  bool _dirHighCountsUp;
  bool _autoEnable;
  uint8_t _enablePinLowActive;
  uint8_t _enablePinHighActive;
  uint8_t _queue_num;

  uint16_t _dir_change_delay_ticks;
  uint32_t _on_delay_ticks;
  uint16_t _off_delay_count;
  uint16_t _auto_disable_delay_counter;

  uint32_t _forward_planning_in_ticks;

#if defined(SUPPORT_ESP32_PULSE_COUNTER) && (ESP_IDF_VERSION_MAJOR == 5)
  pcnt_unit_handle_t _attached_pulse_unit;
  void clearPulseCounter();
  int32_t readPulseCounter();

  // PCNT can not be set to an arbitrary value, so instead we store this as an offset
  int32_t pcnt_offset = 0;
#endif

  friend class FastQueueStepperEngine;
};

#endif /* FASTQUEUESTEPPER_H */
