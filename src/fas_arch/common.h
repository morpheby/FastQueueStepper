#ifndef FAS_COMMON_H
#define FAS_COMMON_H

#define TICKS_FOR_STOPPED_MOTOR 0xffffffff

#define MOVE_OK 0
#define MOVE_ERR_NO_DIRECTION_PIN -1
#define MOVE_ERR_SPEED_IS_UNDEFINED -2
#define MOVE_ERR_ACCELERATION_IS_UNDEFINED -3

// Low level stepper motor command.
//
// You can add these using the addQueueEntry method.
// They will be executed sequentially until the queue is empty.
//
// There are some constraints on the values:
// - `ticks` must be greater or equal to FastAccelStepper::getMaxSpeedInTicks.
// - `ticks*steps` must be greater or equal to MIN_CMD_TICKS
//
// For example:
// A command with ticks=3*TICKS_PER_S/1000, steps = 3, count_up = true means that:
// 1. The direction pin is set to HIGH.
// 2. One step is generated.
// 3. Exactly 1 ms after the first step, the second step is issued.
// 4. Exactly 1 ms after the second step, the third step is issued.
// 5. The stepper waits for 1 ms.
// 6. The next command is processed.
struct stepper_command_s {
  // Number of ticks the command should take
  //
  // Value of zero means a STOP command.
  // 
  // Note: this is different from FAS, where this meant time between consequential
  // steps. Simple sigma-delta like modulation is used to push out steps at uneven times,
  // so keep in mind that the queue may produce some non-uniform noise in the steppers and
  // slightly uneven movements. The benefit though is that the queue will produce exact count
  // of steps at exact times, making sure no deviation happens at any speed.
  uint32_t ticks;

  // Number of steps to send to the stepper motor during this command.
  // If zero, then this command will be treated as a pause, lasting for a number
  // of ticks given by `ticks`.
  // If negative, then the direction pin will be asserted low.
  int32_t steps;
};

//==============================================================================
// All architecture specific definitions should be located here
//==============================================================================

//==========================================================================
#if defined(TEST)
// TEST "architecture" is in use with pc_based testing.
#include "fas_arch/test_pc.h"

#elif defined(ARDUINO_ARCH_ESP32)
// ESP32 derivates using arduino core
#include "fas_arch/arduino_esp32.h"

#elif defined(ESP_PLATFORM)
// ESP32 derivates using espidf
#include "fas_arch/espidf_esp32.h"

#else
#error "Unsupported devices"
#endif

// in order to avoid spikes, first set the value and then make an output
// esp32 idf5 does not like this approach
#ifndef PIN_OUTPUT
#define PIN_OUTPUT(pin, value)  \
  {                             \
    digitalWrite(pin, (value)); \
    pinMode(pin, OUTPUT);       \
  }
#endif

#endif /* FAS_COMMON_H */
