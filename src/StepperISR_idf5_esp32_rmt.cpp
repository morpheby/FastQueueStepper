#include <cmath>
#include <numeric>
#include "StepperISR.h"
#if defined(HAVE_ESP32_RMT) && (ESP_IDF_VERSION_MAJOR == 5)

TaskHandle_t StepperQueue::_rmtFeederTask = NULL;

struct queue_command_encoder_t {
  rmt_encoder_t base;
  rmt_queue_command_t currentQueueEntry {
    .steps = 0,
    .ticks = 0,
  };
  
  uint16_t r = 0;
  uint16_t ticksDone = 0;
  
  rmt_encoder_t *copy_encoder;
};

bool IRAM_ATTR fas_rmt_queue_done_fn(rmt_channel_handle_t tx_chan,
                                     const rmt_tx_done_event_data_t *edata,
                                     void *user_ctx) {
  StepperQueue *q = (StepperQueue *)user_ctx;
  if (q->_rmtCommandsQueued != 0)
    q->_rmtCommandsQueued -= 1;
  if (q->_rmtCommandsQueued == 0) {
    q->_statusFlags |= StepperQueueStatusFlags::QUEUE_RMT_STARVED;
    if (q->_rmtDirToggleDelayCommandsQueued != 0) {
      q->_statusFlags |= StepperQueueStatusFlags::QUEUE_DIRECTION_TOGGLE_DELAY_TOO_LOW;
    }
  }
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  
  // Check if the queue is empty (or in a delay) and request additional feed
  if (q->directionChangeIsAllowed()) {
    xTaskNotifyFromISR(StepperQueue::_rmtFeederTask, 0, eIncrement, &xHigherPriorityTaskWoken);
  }

  return xHigherPriorityTaskWoken == pdTRUE;
}
 
static size_t encode_current_command(queue_command_encoder_t *queue_command_encoder, rmt_channel_handle_t tx_channel,
                                     rmt_encode_state_t *ret_state) {
  size_t symbolsEncoded = 0;
  while (queue_command_encoder->currentQueueEntry.ticks != queue_command_encoder->ticksDone) {
    assert(queue_command_encoder->currentQueueEntry.ticks > queue_command_encoder->ticksDone);
    // Has ticks in current command left. Continue to fill queue
    rmt_encode_state_t session_state = RMT_ENCODING_RESET;
    bool hasSteps = queue_command_encoder->currentQueueEntry.steps != 0;
    uint16_t duration = hasSteps != 0
      ? (queue_command_encoder->currentQueueEntry.ticks + queue_command_encoder->r) / queue_command_encoder->currentQueueEntry.steps
      : queue_command_encoder->currentQueueEntry.ticks;
    
    if (duration == 0) {
      // This is a no-op, skip it
      queue_command_encoder->r = 0;
      queue_command_encoder->ticksDone += 1;
      continue;
    }

    uint16_t halfDuration = duration >> 1,
             remainingDuration = duration - halfDuration;

    rmt_symbol_word_t s = {
      .duration0 = halfDuration,
      .level0 = hasSteps ? (uint16_t) 1 : (uint16_t) 0,
      .duration1 = remainingDuration,
      .level1 = 0,
    };

    symbolsEncoded += queue_command_encoder->copy_encoder->encode(queue_command_encoder->copy_encoder, tx_channel, &s, sizeof(rmt_symbol_word_t), &session_state);
    if (session_state & RMT_ENCODING_MEM_FULL) {
      *ret_state = RMT_ENCODING_MEM_FULL;
      return symbolsEncoded;
    }

    // Symbol ok, continue

    queue_command_encoder->r = hasSteps
      ? (queue_command_encoder->currentQueueEntry.ticks + queue_command_encoder->r) % queue_command_encoder->currentQueueEntry.steps
      : queue_command_encoder->r;
    queue_command_encoder->ticksDone += duration;
  }

  *ret_state = RMT_ENCODING_COMPLETE;

  return symbolsEncoded;
}

static size_t encode_command(rmt_encoder_t *encoder, rmt_channel_handle_t tx_channel,
                             const void *primary_data, size_t data_size, rmt_encode_state_t *ret_state) {
  queue_command_encoder_t *queue_command_encoder = __containerof(encoder, queue_command_encoder_t, base);
  size_t symbolsEncoded = 0;

  assert(data_size >= sizeof(rmt_queue_command_t));

  if (queue_command_encoder->currentQueueEntry.ticks <= queue_command_encoder->ticksDone) {
    const rmt_queue_command_t &q = *(rmt_queue_command_t *) primary_data;
    queue_command_encoder->currentQueueEntry = q;
    queue_command_encoder->ticksDone = 0;
    queue_command_encoder->r = 0;
  }

  {
    rmt_encode_state_t session_state = RMT_ENCODING_RESET;
    symbolsEncoded += encode_current_command(queue_command_encoder, tx_channel, &session_state);
    if (session_state & RMT_ENCODING_MEM_FULL) {
      *ret_state = RMT_ENCODING_MEM_FULL;
      return symbolsEncoded;
    }
  }
  
  queue_command_encoder->currentQueueEntry.ticks = 0;
  queue_command_encoder->currentQueueEntry.steps = 0;
  queue_command_encoder->ticksDone = 0;
  queue_command_encoder->r = 0;

  *ret_state = RMT_ENCODING_COMPLETE;

  return symbolsEncoded;
}

static esp_err_t encoder_delete(rmt_encoder_t *encoder) {
  queue_command_encoder_t *queue_command_encoder = __containerof(encoder, queue_command_encoder_t, base);
  rmt_del_encoder(queue_command_encoder->copy_encoder);
  free(queue_command_encoder);
  return ESP_OK;
}

static esp_err_t encoder_reset(rmt_encoder_t *encoder) {
  queue_command_encoder_t *queue_command_encoder = __containerof(encoder, queue_command_encoder_t, base);
  rmt_encoder_reset(queue_command_encoder->copy_encoder);
  
  queue_command_encoder->currentQueueEntry.ticks = 0;
  queue_command_encoder->currentQueueEntry.steps = 0;
  queue_command_encoder->ticksDone = 0;
  queue_command_encoder->r = 0;

  return ESP_OK;
}

rmt_encoder_handle_t encoder_create() {
  queue_command_encoder_t *queue_encoder = (queue_command_encoder_t *) rmt_alloc_encoder_mem(sizeof(queue_command_encoder_t));
  queue_encoder->base.del = encoder_delete;
  queue_encoder->base.reset = encoder_reset;
  queue_encoder->base.encode = encode_command;
  queue_encoder->ticksDone = 0;
  queue_encoder->r = 0;
  queue_encoder->currentQueueEntry.steps = 0;
  queue_encoder->currentQueueEntry.ticks = 0;
  rmt_copy_encoder_config_t copy_encoder_config = {};
  rmt_new_copy_encoder(&copy_encoder_config, &queue_encoder->copy_encoder);

  return &(queue_encoder->base);
}

void rmt_feeder_task_fn(void *arg) {
  while (true) {
    bool nothingToDo = true;
    for (int i = 0; i < NUM_QUEUES; ++i) {
      fasDisableInterrupts();
      bool shouldFeed = fas_queue[i]._isRunning;
      fasEnableInterrupts();
      if (shouldFeed && fas_queue[i].feedRmt()) {
        nothingToDo = false;
      }
    }
    if (nothingToDo) {
      // If queue not running, or queue was not fed just wait
      ulTaskNotifyTake(true, pdMS_TO_TICKS(4));
    }
  }
}

void StepperQueue::notifyRmt() {
  xTaskNotifyGive(_rmtFeederTask);
}

void StepperQueue::connect_rmt() {
  _initVars();
  
  if (StepperQueue::_rmtFeederTask == NULL) {
    xTaskCreate(rmt_feeder_task_fn, "RMT Feed", 2048, NULL, 6, &_rmtFeederTask);
  }

  pinMode(_stepPin, OUTPUT);
  digitalWrite(_stepPin, LOW);

  _isRunning = false;

  rmt_tx_channel_config_t config {
    .gpio_num = (gpio_num_t)_stepPin,          /*!< GPIO number used by RMT TX channel. Set to -1 if unused */
    .clk_src = RMT_CLK_SRC_DEFAULT,            /*!< Clock source of RMT TX channel, channels in the same group must use the same clock source */
    .resolution_hz = TICKS_PER_S,              /*!< Channel clock resolution, in Hz */
    .mem_block_symbols = 64,                   /*!< Size of memory block, in number of `rmt_symbol_word_t`, must be an even.
                                                    In the DMA mode, this field controls the DMA buffer size, it can be set to a large value;
                                                    In the normal mode, this field controls the number of RMT memory block that will be used by the channel. */
    .trans_queue_depth = RMT_TX_QUEUE_DEPTH,   /*!< Depth of internal transfer queue, increase this value can support more transfers pending in the background */
    .intr_priority = 0,                        /*!< RMT interrupt priority,
                                                    if set to 0, the driver will try to allocate an interrupt with a relative low priority (1,2,3) */
    .flags {                                   /*!< TX channel config flags */
        .invert_out = false,                   /*!< Whether to invert the RMT channel signal before output to GPIO pad */
        .with_dma = false,                     /*!< If set, the driver will allocate an RMT channel with DMA capability */
        .io_loop_back = false,                 /*!< The signal output from the GPIO will be fed to the input path as well */
        .io_od_mode = false,                   /*!< Configure the GPIO as open-drain mode */
    }
  };
  
  esp_err_t rc = rmt_new_tx_channel(&config, &channel);
  ESP_ERROR_CHECK_WITHOUT_ABORT(rc);

  rmt_tx_event_callbacks_t callbacks = {.on_trans_done = fas_rmt_queue_done_fn};
  rmt_tx_register_event_callbacks(channel, &callbacks, this);
  
  _tx_encoder = encoder_create();

  _channel_enabled = false;
}

void StepperQueue::disconnect_rmt() {
  if (_channel_enabled || _isRunning) {
    return;
  }
  rmt_del_channel(channel);
  channel = NULL;
  _tx_encoder->del(_tx_encoder);
  _tx_encoder = NULL;
}

bool StepperQueue::isConnected_rmt() const {
  return channel != NULL;
}

void StepperQueue::startQueue_rmt() {
  if (channel == NULL) {
    return;
  }

  fasDisableInterrupts();

  if (_rmtCommandsQueued == 0) {
    // The RMT is empty, so just in case complete full reset
    _tx_encoder->reset(_tx_encoder);
    _rmtDirToggleDelayCommandsQueued = 0;
  }

  if (!_isRunning) {
    // Clear any STOP commands to make sure we start
    queue_entry entry;
    while (peekQueue(entry) && entry.cmd == QueueCommand::STOP) {
      advanceReadQueue();
    }
  }

  if (!_channel_enabled) {
    rmt_enable(channel);
    _channel_enabled = true;
  }

  _isRunning = true;
  fasEnableInterrupts();

  // Trigger RMT feed task to start working immediately
  notifyRmt();
}

bool StepperQueue::feedRmt() {
  rmt_transmit_config_t tx_config {
    .loop_count = 0,             /*!< Specify the times of transmission in a loop, -1 means transmitting in an infinite loop */
    .flags {                     /*!< Transmit specific config flags */
        .eot_level = 0,          /*!< Set the output level for the "End Of Transmission" */
        .queue_nonblocking = 1,  /*!< If set, when the transaction queue is full, driver will not block the thread but return directly */
    }
  };
  
  queue_entry entry;
  if (!peekQueue(entry)) {
    // Queue is empty, nothing to start
    _statusFlags |= StepperQueueStatusFlags::QUEUE_STARVED_FLAG;
    return false;
  }

  if (entry.cmd == QueueCommand::TOGGLE_DIR) {
    // Temporarily starve the queue, until direction is changed
    fasDisableInterrupts();
    advanceReadQueue();
    _dirChangePending = entry.ticks == QUEUE_ENTRY_DIRECTION_NEGATIVE ? -1 : 1;
    // TODO: Probably weird and unobvious choice. Think about it later
    bool queueNeedsImmediateChange = (_rmtCommandsQueued == 0);
    fasEnableInterrupts();

    // Perform immediate direction change
    if (!queueNeedsImmediateChange || !_engine->changeDirectionIfNeeded(this))
      // Direction change did not succeed, so keep waiting
      return false;
    else
      // We will try again in next loop
      return true;
  } else if (entry.cmd == QueueCommand::STOP) {
    // Starve the queue till it is stopped
    fasDisableInterrupts();
    advanceReadQueue();
    _isRunning = false;
    fasEnableInterrupts();
    return false;
  }

  fasDisableInterrupts();

  // Check direction change request
  if (_dirChangePending) {
    if (_dirChangePending == currentDirection()) {
      // Direction already changed or wasn't needed
      _dirChangePending = 0;
      _rmtDirToggleDelayCommandsQueued = 0;
    } else if (directionChangeIsAllowed()) {
      // Nothing (but direction toggle delays) in queue, we can toggle direction now
      fasEnableInterrupts();
      // Initiate direction change
      if (!_engine->changeDirectionIfNeeded(this))
        // Direction change did not succeed, so we can not start
        return false;
      fasDisableInterrupts();
      _dirChangePending = 0;
      _rmtDirToggleDelayCommandsQueued = 0;
    } else if (entry.steps != 0) {
      // We can not do steps until the direction change is completed,
      // so just wait till it is done.
      fasEnableInterrupts();
      return false;
    } else {
      // Mark that we have some delays in queue, so that the direction change
      // logic will allow the direction change
      _rmtDirToggleDelayCommandsQueued += 1;
    }
  }

  // Prepare the commands
  rmtCmdStorage[_cmdWriteIdx] = rmt_queue_command_t {
    .steps = entry.steps,
    .ticks = entry.ticks,
  };

  _rmtCommandsQueued += 1;
  
  fasEnableInterrupts();

  if (_rmtCommandsQueued >= RMT_TX_QUEUE_DEPTH ||
      rmt_transmit(channel, _tx_encoder, &rmtCmdStorage[_cmdWriteIdx], sizeof(rmt_queue_command_t), &tx_config) != ESP_OK) {
    fasDisableInterrupts();
    _statusFlags |= StepperQueueStatusFlags::QUEUE_RMT_TX_FULL;
    if (_rmtCommandsQueued != 0)
      _rmtCommandsQueued -= 1;
    if (_rmtDirToggleDelayCommandsQueued != 0)
      _rmtDirToggleDelayCommandsQueued -= 1;
    fasEnableInterrupts();
    return false;
  }

  fasDisableInterrupts();

  // The command was sent successfully, so we can advence read pointer
  advanceReadQueue();

  // Commit the position change
  currentPosition += ((int32_t)(uint16_t)entry.steps) * currentDirection();

  // And advance RMT TX queue pointer
  _cmdWriteIdx = (_cmdWriteIdx + 1) % (RMT_TX_QUEUE_DEPTH+1);

  fasEnableInterrupts();

  return true;
}

bool StepperQueue::directionChangeIsAllowed() const {
  fasDisableInterrupts();
  bool allowed = !_isRunning || (_rmtCommandsQueued <= _rmtDirToggleDelayCommandsQueued);
  fasEnableInterrupts();
  return allowed;
}

void StepperQueue::forceStop_rmt() {
  if (_channel_enabled)
    rmt_disable(channel);
  fasDisableInterrupts();
  _channel_enabled = false;
  _isRunning = false;
  _rmtCommandsQueued = 0;
  _rmtDirToggleDelayCommandsQueued = 0;
  currentPosition = 0;
  queueReadIdx = queueWriteIdx;
  fasEnableInterrupts();
}

#endif
