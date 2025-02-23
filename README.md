
## FastQueueStepper

This project is a rework of a great [FastAccelStepper](https://github.com/gin66/FastAccelStepper) that removes everything but the queue management from the
code. Ramp Generator and other planning methods are to be provided as separate libraries (TODO).

FastQueueStepper provides a simple atomic interface for step queue planning, with the intention of final result being practically same as in FastAccelStepper.
The purpose is to be able to create custom planners (e.g. for CNC movement) that provide a more complex path than simple acceleration and deceleration.

The rationale for the split is not only the custom planners, but a more clear separation of the design, so that there is a clear split between what
actually manages steppers and what creates new steps. This should reduce the need for complicated multithreading workarounds and issues in the
original library and make everything more concise.

Current implementation is only tested on ESP32-S2. Arduino version may come some time later. PRs for support of other implementations are totally welcome.

Only IDF5 is supported for ESP32. IDF4 may be added some time in the future if its even needed.

## Documentation

TODO

The library operates by managing a private queue (can be called directly on non RTOS systems) that controls the step planning. Upon creation
of the stepper object by the engine, a planner instance should be provided, which controls the movement.

There are three main components that need to operate together to make everything work:

- StepperPlanner: The class that provides high-level control of the stepper. The instance of this class stores the instance of the actual
  stepper and supplies the engine with the PlanGenerator instance.
- PlanGenerator: This class maintains current running configuration of the stepper and iteratively outputs new steps for the queue. The
  instance is designed to be provide atomic-like interface for the queue manager. That is, any configuration changes happen on an uncommitted
  r/w instance, that can be committed only by the queue manager. After changes are committed, the instance is essentially locked and can only
  produce an updated state by performing a generation. This updated state is then moved to the uncommitted area.
- FastQueueStepper: The main component of this library that effectively manages the queue.