#include <stdint.h>

#include "FastQueueStepper.h"
#include "fas_arch/common.h"

enum class QueueCommand: uint8_t {
  STEP,
  TOGGLE_DIR,
  STOP,
  SYNC // not used yet, but intended for ESP32's RMT synchronization if super-exact movement is needed
};

struct queue_entry {
  QueueCommand cmd;
  int8_t steps;
  uint16_t ticks; // Should be less or equal to 0xFFFE
};

#ifdef SUPPORT_ESP32_RMT
struct rmt_queue_command_t {
  uint8_t steps;
  uint16_t ticks; // NOTE: Should be less or equal to 0xFFFE (65534)
};
#endif

class StepperQueue {

 public:
  inline bool isRunning() { return _isRunning; }
#if defined(SUPPORT_ESP32)
  bool isReadyForCommands();
#endif

  void init(uint8_t queue_num, uint8_t step_pin);

  inline uint8_t queueEntries() {
    fasDisableInterrupts();
    uint16_t rp = queueReadIdx;
    uint16_t wp = queueWriteIdx;
    fasEnableInterrupts();
    return (QUEUE_LEN + wp - rp) % QUEUE_LEN;
  }
  inline constexpr uint8_t queueSize() { return QUEUE_LEN; }
  inline bool isQueueFull() { return queueEntries() == QUEUE_LEN - 1; }
  inline bool isQueueEmpty() { return queueEntries() == 0; }

  int8_t addQueueEntry(const queue_entry &cmd);
  
  // Read current moves that are already done (erased from queue)
  inline int32_t readAndClearCurrentPosition() {
    fasDisableInterrupts();
    int32_t cp = currentPosition;
    currentPosition = 0;
    fasEnableInterrupts();
    return cp;
  }

  uint32_t ticksInQueue();
  uint32_t hasTicksInQueue(uint32_t min_ticks);

  inline uint16_t getMaxSpeedInTicks() { return max_speed_in_ticks; }

  void startQueue();
  void forceStop();
  void _initVars();
  void connect();
  void disconnect();

#if SUPPORT_UNSAFE_ABS_SPEED_LIMIT_SETTING == 1
  void setAbsoluteSpeedLimit(uint16_t ticks) { max_speed_in_ticks = ticks; }
#endif
  void adjustSpeedToStepperCount(uint8_t steppers);
  static bool isValidStepPin(uint8_t step_pin);
  static int8_t queueNumForStepPin(uint8_t step_pin);

 private:
  queue_entry queue[QUEUE_LEN];
  uint16_t queueReadIdx;
  uint16_t queueWriteIdx;
  int32_t currentPosition;
  int _stepPin;
  int _dirPin;

  uint16_t max_speed_in_ticks;
  
  inline bool peekQueue(queue_entry &e) {
    fasDisableInterrupts();
    if (isQueueEmpty) return false;
    e = queue[queueReadIdx];
    fasEnableInterrupts();
    return true;
  }

  inline bool readQueue(queue_entry &e) {
    fasDisableInterrupts();
    if (isQueueEmpty) return false;
    e = queue[queueReadIdx];
    queueReadIdx = (queueReadIdx + 1) % QUEUE_LEN;
    fasEnableInterrupts();
    return true;
  }

#if defined(SUPPORT_ESP32)
  bool _isRunning;
  bool _nextCommandIsPrepared;
#endif

#ifdef SUPPORT_ESP32_RMT
  RMT_CHANNEL_T channel;
  bool _channel_enabled;
  bool _rmtQueueRunning;
  bool _dirChangePending;
  rmt_encoder_handle_t _tx_encoder;
  static TaskHandle_t _rmtFeederTask;
  rmt_queue_command_t rmtCmdStorage[RMT_TX_QUEUE_DEPTH];
  uint16_t _cmdWriteIdx;

  friend bool IRAM_ATTR queue_done(rmt_channel_handle_t tx_chan,
                                   const rmt_tx_done_event_data_t *edata,
                                   void *user_ctx);
  friend void rmt_feeder_task_fn(void *arg);

  bool isReadyForCommands_rmt();
  void init_rmt(uint8_t channel_num, uint8_t step_pin);
  void startQueue_rmt();
  void forceStop_rmt();
  void connect_rmt();
  void disconnect_rmt();
  bool feedRmt();
  
#endif
};

extern StepperQueue fas_queue[NUM_QUEUES];

class FastQueueStepperEngine;

#if defined(SUPPORT_CPU_AFFINITY)
void fas_init_engine(FastQueueStepperEngine* engine, uint8_t cpu_core);
#else
void fas_init_engine(FastQueueStepperEngine* engine);
#endif
