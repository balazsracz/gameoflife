#include "stm32f0xx_hal_tsc.h"
#include "USBSerial.h"
#include "stm32_def.h"
#include "stm32f0xx_hal.h"



TSC_HandleTypeDef TscHandle;
TSC_IOConfigTypeDef IoConfig;


/* Array used to store the three acquisition value (one per channel) */
__IO uint32_t uhTSCAcquisitionValue[3];

// Stores the acquisition results. The lower these numbers are, the closer a finger is touching the respective buttons.
// There are four rows and four columns. The first index selects rows vs columns, see kChRow, kChCol.
uint32_t channel_results[2][4];
// Use this constant in the first index of channel_results[][] to see the output for rows.
static constexpr unsigned kChRow = 0;
// Use this constant in the first index of channel_results[][] to see the output for columns.
static constexpr unsigned kChCol = 1;

// Stores average value over all readings of the channel. This is the sum.
uint64_t channel_sum[2][4] = { 0 };
// Stores average value over all readings of the channel. This is the count.
unsigned channel_count[2][4] = { 0 };

// We use the first this many ticks after boot for calibration purposes.
static constexpr uint32_t kCalibrateTickCount = 1000;

// Output variables for row 0..3 to be used by the application. True: active (pressed), false: inactive (not pressed).
bool btn_row_active[4];
// Output variables for column 0..3 to be used by the application.
bool btn_col_active[4];

unsigned conv_count = 0;

struct AcquisitionPhase {
  // Bitmask of which channel IOs to capture.
  uint32_t channel_ios;
  // Group2 channel: row or col.
  uint8_t group2_rc;
  // Group2 channel: which entry.
  uint8_t group2_num;
  // Group3 channel: row or col.
  uint8_t group3_rc;
  // Group3 channel: which entry.
  uint8_t group3_num;
  // Group5 channel: row or col.
  uint8_t group5_rc;
  // Group5 channel: which entry.
  uint8_t group5_num;
};

static constexpr unsigned kNumPhases = 3;
#if defined(HW_REV1)
static const AcquisitionPhase kPhases[kNumPhases] = {
  { TSC_GROUP2_IO2 | TSC_GROUP3_IO3 | TSC_GROUP5_IO2, kChCol, 0, kChRow, 1, kChCol, 2 },
  { TSC_GROUP2_IO3 | TSC_GROUP3_IO4 | TSC_GROUP5_IO3, kChRow, 2, kChCol, 1, kChRow, 3 },
  { TSC_GROUP2_IO4 | TSC_GROUP3_IO3 | TSC_GROUP5_IO2, kChRow, 0, kChRow, 1, kChCol, 2 }
};
#else
static const AcquisitionPhase kPhases[kNumPhases] = {
  { TSC_GROUP2_IO2 | TSC_GROUP3_IO3 | TSC_GROUP5_IO2, kChCol, 0, kChRow, 1, kChCol, 2 },
  { TSC_GROUP2_IO3 | TSC_GROUP3_IO4 | TSC_GROUP5_IO3, kChRow, 2, kChCol, 1, kChCol, 3 },
  { TSC_GROUP2_IO4 | TSC_GROUP3_IO3 | TSC_GROUP5_IO4, kChRow, 0, kChRow, 1, kChRow, 3 }
};
#endif

// Which phase was last started.
unsigned current_phase = 0;
uint32_t next_conv_tick = 0;

extern "C" {
  void TSC_IRQHandler(void) {
    HAL_TSC_IRQHandler(&TscHandle);
  }
}

void TouchIoStart(unsigned phase) {
  if (phase >= kNumPhases) {
    phase = 0;
  }
  /*##-2- Configure the touch-sensing IOs ####################################*/
  IoConfig.ChannelIOs = kPhases[phase].channel_ios;
  IoConfig.SamplingIOs = TSC_GROUP2_IO1 | TSC_GROUP3_IO2 | TSC_GROUP5_IO1;
  IoConfig.ShieldIOs = 0;

  if (HAL_TSC_IOConfig(&TscHandle, &IoConfig) != HAL_OK) {
    /* Initialization Error */
    Error_Handler();
  }
}

void TouchSetup() {
  __HAL_RCC_TSC_CLK_ENABLE();
  GPIO_InitTypeDef GPIO_InitStruct;

  /*##-2- Configure Sampling Capacitor IOs (Alternate-Function Open-Drain) ###*/
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  GPIO_InitStruct.Alternate = GPIO_AF3_TSC;
  // caps are A4 B0 B3
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*##-3- Configure Channel IOs (Alternate-Function Output PP) ###############*/
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  // A 5-6-7
  GPIO_InitStruct.Pin = GPIO_PIN_5;  // group2 io2
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_PIN_6;  // group2 io3
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_PIN_7;  // group2 io4
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // B 1-2 4-5-6
  GPIO_InitStruct.Pin = GPIO_PIN_1;  // group3 io3
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_PIN_2;  // group3 io4
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_PIN_4;  // group5 io2
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_PIN_6;  // group5 io3
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  // B5 is incorrectly wired to a TSC circuit. B7 is a spare pin.
  GPIO_InitStruct.Pin = GPIO_PIN_7;  // group5 io4
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(TSC_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TSC_IRQn);

  /*##-1- Configure the TSC peripheral #######################################*/
  TscHandle.Instance = TSC;
  TscHandle.Init.AcquisitionMode = TSC_ACQ_MODE_NORMAL;
  TscHandle.Init.CTPulseHighLength = TSC_CTPH_1CYCLE;
  TscHandle.Init.CTPulseLowLength = TSC_CTPL_1CYCLE;
  TscHandle.Init.IODefaultMode = TSC_IODEF_IN_FLOAT; /* Because the electrodes are interlaced on this board */
  TscHandle.Init.MaxCountInterrupt = ENABLE;
  TscHandle.Init.MaxCountValue = TSC_MCV_16383;
  TscHandle.Init.PulseGeneratorPrescaler = TSC_PG_PRESC_DIV64;
  TscHandle.Init.SpreadSpectrum = DISABLE;
  TscHandle.Init.SpreadSpectrumDeviation = 127;
  TscHandle.Init.SpreadSpectrumPrescaler = TSC_SS_PRESC_DIV1;
  TscHandle.Init.SynchroPinPolarity = TSC_SYNC_POLARITY_FALLING;
  TscHandle.Init.ChannelIOs = 0;  /* Not needed yet. Will be set with HAL_TSC_IOConfig() */
  TscHandle.Init.SamplingIOs = 0; /* Not needed yet. Will be set with HAL_TSC_IOConfig() */
  TscHandle.Init.ShieldIOs = 0;   /* Not needed yet. Will be set with HAL_TSC_IOConfig() */

  if (HAL_TSC_Init(&TscHandle) != HAL_OK) {
    /* Initialization Error */
    Error_Handler();
  }

  // B5 we switch to analog mode.
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  TouchIoStart(0);
  HAL_TSC_IODischarge(&TscHandle, ENABLE);
  next_conv_tick = HAL_GetTick() + 2;
}

// Updates the calibration data structure with the more recent measurement results.
void TouchCalibrate() {
  uint8_t results = 0;
  for (unsigned rc : { 0, 1 }) {
    for (unsigned num : { 0, 1, 2, 3 }) {
      channel_sum[rc][num] += channel_results[rc][num];
      ++channel_count[rc][num];
    }
  }
}

static constexpr int kMinDeltaThreshold = 50;
static constexpr int kDeltaThreshold = 200;
static constexpr unsigned kRelThresholdNom = 85;
static constexpr unsigned kRelThresholdDenom = 100;

/// Evaluates a touch sense channel.
/// @param rc is kChRow or kChCol
/// @param num is 0..3
/// @return true if this channel is active (finger is on it).
bool TouchEval(unsigned rc, unsigned num) {
  int ref = channel_sum[rc][num] / channel_count[rc][num];
  int actual = channel_results[rc][num];
  if (actual >= ref - kMinDeltaThreshold) return false;
  if (actual < ref - kDeltaThreshold) return true;
  if (actual < (ref * kRelThresholdNom / kRelThresholdDenom)) return true;
  return false;
}

/// Helper function for debug printout.
/// @param rc is kChRow or kChCol
/// @param num is 0..3
/// @return value to print in debug log line
int tdeb(unsigned rc, unsigned num) {
  return channel_results[rc][num];
  //return (channel_sum[rc][num] / channel_count[rc][num]) - channel_results[rc][num];
}

/**
  * @brief  Acquisition completed callback in non blocking mode 
  * @param  htsc: pointer to a TSC_HandleTypeDef structure that contains
  *         the configuration information for the specified TSC.
  * @retval None
  */
extern "C" void HAL_TSC_ConvCpltCallback(TSC_HandleTypeDef* htsc) {
  //Error_Handler();
  const AcquisitionPhase& cphase = kPhases[current_phase];
  /*##-5- Discharge the touch-sensing IOs ####################################*/
  HAL_TSC_IODischarge(&TscHandle, ENABLE);
  /* Note: a delay can be added here */

  /*##-6- Check if the First channel acquisition is correct (no max count) ###*/
  if (HAL_TSC_GroupGetStatus(&TscHandle, TSC_GROUP2_IDX) == TSC_GROUP_COMPLETED) {
    channel_results[cphase.group2_rc][cphase.group2_num] = HAL_TSC_GroupGetValue(&TscHandle, TSC_GROUP2_IDX);
  } else {
    channel_results[cphase.group2_rc][cphase.group2_num] = 0;
  }

  if (HAL_TSC_GroupGetStatus(&TscHandle, TSC_GROUP3_IDX) == TSC_GROUP_COMPLETED) {
    channel_results[cphase.group3_rc][cphase.group3_num] = HAL_TSC_GroupGetValue(&TscHandle, TSC_GROUP3_IDX);
  } else {
    channel_results[cphase.group3_rc][cphase.group3_num] = 0;
  }

  if (HAL_TSC_GroupGetStatus(&TscHandle, TSC_GROUP5_IDX) == TSC_GROUP_COMPLETED) {
    channel_results[cphase.group5_rc][cphase.group5_num] = HAL_TSC_GroupGetValue(&TscHandle, TSC_GROUP5_IDX);
  } else {
    channel_results[cphase.group5_rc][cphase.group5_num] = 0;
  }

  next_conv_tick = HAL_GetTick() + 2;
}

bool tsc_error = false;

void HAL_TSC_ErrorCallback(TSC_HandleTypeDef* htsc) {
  tsc_error = true;
}

uint32_t conv_start_tick = -1;

// Call this function from the loop().
void TouchLoop() {
  if (tsc_error) {
    SerialUSB.printf("TSC error phase %d\n", current_phase);
    tsc_error = false;
    //HAL_TSC_DeInit(&TscHandle);
    //TouchSetup();
    next_conv_tick = HAL_GetTick() + 100;
  }
  //Error_Handler();
  auto t = HAL_GetTick();
  if (t >= next_conv_tick) {
    next_conv_tick = -1;
    ++current_phase;
    if (current_phase >= kNumPhases) {
      if (HAL_GetTick() < kCalibrateTickCount) {
        // Records this data for calibration purposes.
        TouchCalibrate();
      } else {
        // Evaluates the measured data against the stored calibration.
        for (unsigned i = 0; i < 4; ++i) {
          btn_row_active[i] = TouchEval(kChRow, i);
          btn_col_active[i] = TouchEval(kChCol, i);
        }
      }
      current_phase = 0;
    }
    // Prepare for next sample.
    TouchIoStart(current_phase);
    HAL_TSC_IODischarge(&TscHandle, ENABLE);
    conv_start_tick = t + 1;
  }
  if (t >= conv_start_tick) {
    conv_start_tick = -1;
    HAL_TSC_IODischarge(&TscHandle, DISABLE);
    if (HAL_TSC_Start_IT(&TscHandle) != HAL_OK) {
      /* Acquisition Error */
      Error_Handler();
    }
    ++conv_count;
  }
}
