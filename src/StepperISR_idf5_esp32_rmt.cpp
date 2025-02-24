#include <cmath>
#include <numeric>
#include "StepperISR.h"
#if defined(HAVE_ESP32_RMT) && (ESP_IDF_VERSION_MAJOR == 5)

// #define TEST_MODE

#include "fas_arch/test_probe.h"

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

static void change_direction_and_continue_queue_fn(void *pvParameter1, uint32_t ulParameter2) {
  StepperQueue *q = (StepperQueue *)pvParameter1;
  q->startQueue();
}

static bool IRAM_ATTR queue_done(rmt_channel_handle_t tx_chan,
                                 const rmt_tx_done_event_data_t *edata,
                                 void *user_ctx) {
  StepperQueue *q = (StepperQueue *)user_ctx;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  fasDisableInterrupts();
  q->_rmtQueueRunning = false;
  if (q->_dirChangePending) {
    // Next commands require change of direction, so wake up
    xTimerPendFunctionCallFromISR(change_direction_and_continue_queue_fn, q, 0, &xHigherPriorityTaskWoken);
  }
  fasEnableInterrupts();
  return xHigherPriorityTaskWoken == pdTRUE;
}

// #define ENTER_PAUSE(ticks)                                          \
//   {                                                                 \
//     uint16_t remaining_ticks = ticks;                               \
//     uint16_t half_ticks_per_symbol = ticks / (2 * PART_SIZE);       \
//     uint32_t main_symbol = 0x00010001 * half_ticks_per_symbol;      \
//     for (uint8_t i = 0; i < PART_SIZE - 1; i++) {                   \
//       (*symbols++).val = main_symbol;                               \
//     }                                                               \
//     remaining_ticks -= 2 * (PART_SIZE - 1) * half_ticks_per_symbol; \
//     uint16_t first_ticks = remaining_ticks / 2;                     \
//     remaining_ticks -= first_ticks;                                 \
//     last_entry = 0x00010000 * first_ticks + remaining_ticks;        \
//   }
 
static size_t encode_current_command(queue_command_encoder_t *queue_command_encoder, rmt_channel_handle_t tx_channel,
                                     rmt_encode_state_t *ret_state) {
  size_t symbolsEncoded = 0;
  while (queue_command_encoder->currentQueueEntry.ticks != queue_command_encoder->ticksDone) {
    assert(queue_command_encoder->currentQueueEntry.ticks < queue_command_encoder->ticksDone);
    // Has ticks in current command left. Continue to fill queue
    rmt_encode_state_t session_state = RMT_ENCODING_RESET;
    bool hasSteps = queue_command_encoder->currentQueueEntry.steps != 0;
    uint16_t duration = hasSteps != 0
      ? (queue_command_encoder->currentQueueEntry.ticks + queue_command_encoder->r) / queue_command_encoder->currentQueueEntry.steps
      : queue_command_encoder->currentQueueEntry.ticks;

    rmt_symbol_word_t s = {
      .level0 = hasSteps ? 1 : 0,
      .duration0 = duration / 2,
      .level1 = 0,
      .duration1 = duration - (duration / 2),
    };

    symbolsEncoded += queue_command_encoder->copy_encoder->encode(queue_command_encoder->copy_encoder, tx_channel, &s, sizeof(rmt_symbol_word_t), &session_state);
    if (session_state & RMT_ENCODING_MEM_FULL) {
      *ret_state = RMT_ENCODING_MEM_FULL;
      return symbolsEncoded;
    }

    // Symbol ok, continue

    queue_command_encoder->r = (queue_command_encoder->currentQueueEntry.ticks + queue_command_encoder->r) % queue_command_encoder->currentQueueEntry.steps;
    queue_command_encoder->ticksDone += duration;
  }

  *ret_state = RMT_ENCODING_COMPLETE;

  return symbolsEncoded;
}

static size_t encode_command(rmt_encoder_t *encoder, rmt_channel_handle_t tx_channel,
                             const void *primary_data, size_t data_size, rmt_encode_state_t *ret_state) {
  queue_command_encoder_t *queue_command_encoder = __containerof(encoder, queue_command_encoder_t, base);
  size_t symbolsEncoded = 0;

  if (data_size < sizeof(rmt_queue_command_t)) {
    printf("Invalid RMT command encoded\n");
    *ret_state = RMT_ENCODING_RESET;
    queue_command_encoder->currentQueueEntry.steps = 0;
    queue_command_encoder->currentQueueEntry.ticks = 0;
    queue_command_encoder->r = 0;
    queue_command_encoder->ticksDone = 0;
    queue_command_encoder->copy_encoder->reset(queue_command_encoder->copy_encoder);
    return symbolsEncoded;
  }

  {
    rmt_encode_state_t session_state = RMT_ENCODING_RESET;
    symbolsEncoded += encode_current_command(queue_command_encoder, tx_channel, &session_state);
    if (session_state & RMT_ENCODING_MEM_FULL) {
      *ret_state = RMT_ENCODING_MEM_FULL;
      return symbolsEncoded;
    }
  }

  // Last symbol finished, load a new one
  const rmt_queue_command_t &q = *(rmt_queue_command_t *) primary_data;
  queue_command_encoder->currentQueueEntry = q;
  queue_command_encoder->ticksDone = 0;

  {
    rmt_encode_state_t session_state = RMT_ENCODING_RESET;
    symbolsEncoded += encode_current_command(queue_command_encoder, tx_channel, &session_state);
    if (session_state & RMT_ENCODING_MEM_FULL) {
      *ret_state = RMT_ENCODING_MEM_FULL;
      return symbolsEncoded;
    }
  }
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
  return ESP_OK;
}

rmt_encoder_handle_t encoder_create() {
  queue_command_encoder_t *queue_encoder = (queue_command_encoder_t *) rmt_alloc_encoder_mem(sizeof(queue_command_encoder_t));
  rmt_copy_encoder_config_t copy_encoder_config = {};
  rmt_new_copy_encoder(&copy_encoder_config, &queue_encoder->copy_encoder);

  return &(queue_encoder->base);
}

// static size_t IRAM_ATTR encode_commands(const void *data, size_t data_size,
//                                         size_t symbols_written,
//                                         size_t symbols_free,
//                                         rmt_symbol_word_t *symbols, bool *done,
//                                         void *arg) {

//   StepperQueue *q = (StepperQueue *)arg;

//   *done = false;
//   if (symbols_free < PART_SIZE) {
//     // not sufficient space for the symbols
//     return 0;
//   }

//   uint8_t rp = q->read_idx;
//   if (q->_rmtStopped) {
//     *done = true;
//     return 0;
//   }
//   if ((rp == q->next_write_idx) || q->_rmtStopped) {
//     // if we return done already here, then single stepping fails
//     q->_rmtStopped = true;
//     // Not sure if this pause is really needed
//     uint16_t last_entry;
//     ENTER_PAUSE(MIN_CMD_TICKS);
//     symbols->val = last_entry;
//     return PART_SIZE;
//   }

//   // Process command
//   struct queue_entry *e_curr = &q->entry[rp & QUEUE_LEN_MASK];

//   if (e_curr->toggle_dir) {
//     // the command requests dir pin toggle
//     // This is ok only, if the ongoing command does not contain steps
//     if (q->lastChunkContainsSteps) {
//       // So we need a pause. change the finished read entry into a pause
//       q->lastChunkContainsSteps = false;
//       uint16_t last_entry;
//       ENTER_PAUSE(MIN_CMD_TICKS);
//       symbols->val = last_entry;
//       return PART_SIZE;
//     }
//     // The ongoing command does not contain steps, so change dir here should be
//     // ok
//     LL_TOGGLE_PIN(q->dirPin);
//     // and delete the request
//     e_curr->toggle_dir = 0;
//   }

//   uint8_t steps = e_curr->steps;
//   uint16_t ticks = e_curr->ticks;
//   //  if (steps != 0) {
//   //  	PROBE_2_TOGGLE;
//   //}
//   uint32_t last_entry;
//   if (steps == 0) {
//     q->lastChunkContainsSteps = false;
//     ENTER_PAUSE(ticks);
//   } else {
//     q->lastChunkContainsSteps = true;
//     if (ticks == 0xffff) {
//       // special treatment for this case, because an rmt entry can only cover up
//       // to 0xfffe ticks every step must be minimum split into two rmt entries,
//       // so at max PART/2 steps can be done.
//       if (steps < PART_SIZE / 2) {
//         for (uint8_t i = 1; i < steps; i++) {
//           // steps-1 iterations
//           (*symbols++).val = 0x40007fff | 0x8000;
//           (*symbols++).val = 0x20002000;
//         }
//         // the last step needs to be stretched to fill PART_SIZE entries
//         (*symbols++).val = 0x40007fff | 0x8000;
//         uint16_t delta = PART_SIZE - 2 * steps;
//         delta <<= 5;
//         (*symbols++).val = 0x20002000 - delta;
//         // 2*(steps - 1) + 1 already stored => 2*steps - 1
//         // and after this for loop one entry added => 2*steps
//         for (uint8_t i = 2 * steps; i < PART_SIZE - 1; i++) {
//           (*symbols++).val = 0x00100010;
//         }
//         last_entry = 0x00100010;
//         steps = 0;
//       } else {
//         steps -= PART_SIZE / 2;
//         for (uint8_t i = 0; i < PART_SIZE / 2 - 1; i++) {
//           (*symbols++).val = 0x40007fff | 0x8000;
//           (*symbols++).val = 0x20002000;
//         }
//         (*symbols++).val = 0x40007fff | 0x8000;
//         last_entry = 0x20002000;
//       }
//     } else if ((steps < 2 * PART_SIZE) && (steps != PART_SIZE)) {
//       uint8_t steps_to_do = steps;
//       if (steps > PART_SIZE) {
//         steps_to_do /= 2;
//       }

//       uint16_t ticks_l = ticks >> 1;
//       uint16_t ticks_r = ticks - ticks_l;
//       uint32_t rmt_entry = ticks_l;
//       rmt_entry <<= 16;
//       rmt_entry |= ticks_r | 0x8000;  // with step
//       for (uint8_t i = 1; i < steps_to_do; i++) {
//         (*symbols++).val = rmt_entry;
//       }
//       // the last step needs to be stretched to fill PART_SIZE entries
//       uint32_t delta = PART_SIZE - steps_to_do;
//       delta <<= 18;  // shift in upper 16bit and multiply with 4
//       (*symbols++).val = rmt_entry - delta;
//       for (uint8_t i = steps_to_do; i < PART_SIZE - 1; i++) {
//         (*symbols++).val = 0x00020002;
//       }
//       last_entry = 0x00020002;
//       steps -= steps_to_do;
//     } else {
//       // either >= 2*PART_SIZE or = PART_SIZE
//       // every entry one step
//       uint16_t ticks_l = ticks >> 1;
//       uint16_t ticks_r = ticks - ticks_l;
//       uint32_t rmt_entry = ticks_l;
//       rmt_entry <<= 16;
//       rmt_entry |= ticks_r | 0x8000;  // with step
//       for (uint8_t i = 0; i < PART_SIZE - 1; i++) {
//         (*symbols++).val = rmt_entry;
//       }
//       last_entry = rmt_entry;
//       steps -= PART_SIZE;
//     }
//   }

//   // if (!fill_part_one) {
//   //  Note: When enabling the continuous transmission mode by setting
//   //  RMT_REG_TX_CONTI_MODE, the transmitter will transmit the data on the
//   //  channel continuously, that is, from the first byte to the last one,
//   //  then from the first to the last again, and so on. In this mode, there
//   //  will be an idle level lasting one clk_div cycle between N and N+1
//   //  transmissions.
//   // last_entry -= 1;
//   //}
//   symbols->val = last_entry;

//   // Data is complete
//   if (steps == 0) {
//     // The command has been completed
//     if (e_curr->repeat_entry == 0) {
//       q->read_idx = rp + 1;
//     }
//   } else {
//     e_curr->steps = steps;
//   }

//   return PART_SIZE;
// }

void rmt_feeder_task_fn(void *arg) {
  while (true) {
    bool nothingToDo = true;
    for (int i = 0; i < NUM_QUEUES; ++i) {
      if (fas_queue[i]._isRunning && fas_queue[i].feedRmt()) {
        nothingToDo = false;
      }
    }
    if (nothingToDo) {
      // If queue not running, or queue was not fed just wait
      ulTaskNotifyTake(true, pdMS_TO_TICKS(10));
    }
  }
}

TaskHandle_t StepperQueue::_rmtFeederTask = NULL;

void StepperQueue::init_rmt(uint8_t channel_num, uint8_t step_pin) {
  _initVars();
  _stepPin = step_pin;
  pinMode(step_pin, OUTPUT);
  digitalWrite(step_pin, LOW);

  _tx_encoder = encoder_create();
  connect_rmt();
  _isRunning = false;
  _rmtQueueRunning = false;

  if (StepperQueue::_rmtFeederTask == NULL) {
    xTaskCreate(rmt_feeder_task_fn, "RMT Feed", 2048, NULL, 6, &_rmtFeederTask);
  }
}

void StepperQueue::connect_rmt() {
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

  rmt_tx_event_callbacks_t callbacks = {.on_trans_done = queue_done};
  rmt_tx_register_event_callbacks(channel, &callbacks, this);

  _channel_enabled = false;
}

void StepperQueue::disconnect_rmt() {
  if (_channel_enabled || _isRunning || _rmtQueueRunning) {
    return;
  }
  rmt_del_channel(channel);
  channel = NULL;
}

void StepperQueue::startQueue_rmt() {
  if (channel == NULL) {
    return;
  }

  // set dirpin toggle here
  fasDisableInterrupts();
  if (_dirChangePending) {
    LL_TOGGLE_PIN(_dirPin);
    _dirChangePending = false;
  }

  if (!_channel_enabled) {
    rmt_enable(channel);
    _channel_enabled = true;
  }

  _isRunning = true;
  fasEnableInterrupts();

  xTaskNotifyGive(_rmtFeederTask);
}

bool StepperQueue::feedRmt() {
  _tx_encoder->reset(_tx_encoder);

  rmt_transmit_config_t tx_config {
    .loop_count = 0,             /*!< Specify the times of transmission in a loop, -1 means transmitting in an infinite loop */
    .flags {                     /*!< Transmit specific config flags */
        .eot_level = 0,          /*!< Set the output level for the "End Of Transmission" */
        .queue_nonblocking = 0,  /*!< If set, when the transaction queue is full, driver will not block the thread but return directly */
    }
  };
  
  queue_entry entry;
  if (!readQueue(entry)) {
    // Queue is empty, nothing to start
    return false;
  }
  if (entry.cmd == QueueCommand::TOGGLE_DIR) {
    // Temporarily starve the queue, until direction is changed
    fasDisableInterrupts();
    _dirChangePending = true;
    fasEnableInterrupts();
    return false;
  } else if (entry.cmd == QueueCommand::STOP) {
    // Starve the queue till it is stopped
    fasDisableInterrupts();
    _isRunning = false;
    fasEnableInterrupts();
    return false;
  }

  fasDisableInterrupts();
  // Prepare the commands
  rmtCmdStorage[_cmdWriteIdx] = rmt_queue_command_t {
    .steps = std::abs(entry.steps),
    .ticks = entry.ticks,
  };
  
  _rmtQueueRunning = true;
  rmt_transmit(channel, _tx_encoder, &rmtCmdStorage[_cmdWriteIdx], sizeof(rmt_queue_command_t), &tx_config);

  _cmdWriteIdx = (_cmdWriteIdx + 1) % RMT_TX_QUEUE_DEPTH;

  fasEnableInterrupts();
}

void StepperQueue::forceStop_rmt() {
  fasDisableInterrupts();
  if (_channel_enabled)
    rmt_disable(channel);
  _channel_enabled = false;
  _isRunning = false;
  _rmtQueueRunning = false;
  queueReadIdx = queueWriteIdx;
  fasEnableInterrupts();
}

bool StepperQueue::isReadyForCommands_rmt() {
  return true;
}

#endif
