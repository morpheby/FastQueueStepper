
## FastQueueStepper

This project is a rework of a great [FastAccelStepper](https://github.com/gin66/FastAccelStepper) that removes everything but the queue management from the
code. Ramp Generator and other planning methods are to be provided as separate libraries (TODO).

FastQueueStepper provides a simple atomic interface for step queue planning, with the intention of final result being practically same as in FastAccelStepper.
The purpose is to be able to create custom planners (e.g. for CNC movement) that provide a more complex path than simple acceleration and deceleration.

Current implementation is only tested on ESP32-S2. Arduino version may come some time later. PRs for support of other implementations are totally welcome.

## Documentation

TODO
