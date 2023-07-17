/*
  Test app

  Various hardware features will be tested in this sketch. 

  To compile, use these settings:
  - board manager = stm32
  - Board = "generic STM32F0 series"
  - Board part number = "Generic STM32F072CBTx"
  - Upload method = "Stm32CubeProgrammer (DFU)"

  To upload, press finger firmly on the FLASH pad, then use a metal coin two hit the RESET pad. Then click upload in the Arduino toolbar.
*/

#include "stm32f0xx_hal_tsc.h"
//#include "stm32f0xx_hal_tsc.c"


//Serial north;

static constexpr int LED13_PIN = PF0;
static constexpr int LED14_PIN = PF1;

static constexpr int MENU_PIN = PB15;

static constexpr int R1_PIN = PC14;
static constexpr int R2_PIN = PC15;
static constexpr int R3_PIN = PC13;

static constexpr int R4_PIN = PA15;
static constexpr int R5_PIN = PB12;
static constexpr int R6_PIN = PA8;



TSC_HandleTypeDef TscHandle;
TSC_IOConfigTypeDef IoConfig;

bool start_conv = false;

/* Array used to store the three acquisition value (one per channel) */
__IO uint32_t uhTSCAcquisitionValue[3];

// Stores the acquisition results. The lower these numbers are, the closer a finger is touching the respective buttons.
// There are four rows and four columns. The first index selects rows vs columns, see kChRow, kChCol.
uint32_t channel_results[2][4];
// Use this constant in the first index of channel_results[][] to see the output for rows.
static constexpr unsigned kChRow = 0;
// Use this constant in the first index of channel_results[][] to see the output for columns.
static constexpr unsigned kChCol = 1;

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
static const AcquisitionPhase kPhases[kNumPhases] = {
  { TSC_GROUP2_IO2 | TSC_GROUP3_IO3 | TSC_GROUP5_IO2, kChCol, 0, kChRow, 1, kChCol, 2 },
  { TSC_GROUP2_IO3 | TSC_GROUP3_IO4 | TSC_GROUP5_IO3, kChRow, 2, kChCol, 1, kChCol, 3 },
  { TSC_GROUP2_IO4 | TSC_GROUP3_IO3 | TSC_GROUP5_IO4, kChRow, 0, kChRow, 1, kChRow, 3 }
};
// Which phase was last started.
unsigned current_phase = 0;

extern "C" {
  void TSC_IRQHandler(void) {
    HAL_TSC_IRQHandler(&TscHandle);
  }
}

void touch_setup() {
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
  // B5 is incorrectly wired to a TCS circuit. B7 is a spare pin.
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
}

void touch_iostart(unsigned phase) {
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
  current_phase = phase;
  //start_conv = false;
}


#define BLINKER_TIMER TIM14

HardwareTimer timCharlie(BLINKER_TIMER);

void charliehandler(void) {
  static int i = 0;
  i++;
  //return;
  //digitalWrite(LED14_PIN, (i % 6) == 0 ? LOW : HIGH);
  switch (i) {
    case 1:
      pinMode(R3_PIN, INPUT);
      pinMode(R1_PIN, OUTPUT);
      pinMode(R2_PIN, OUTPUT);
      digitalWrite(R1_PIN, LOW);
      digitalWrite(R2_PIN, HIGH);
      pinMode(R6_PIN, INPUT);
      pinMode(R4_PIN, OUTPUT);
      pinMode(R5_PIN, OUTPUT);
      digitalWrite(R4_PIN, LOW);
      digitalWrite(R5_PIN, HIGH);
      break;
    case 2:
      pinMode(R3_PIN, INPUT);
      pinMode(R6_PIN, INPUT);
      digitalWrite(R1_PIN, HIGH);
      digitalWrite(R2_PIN, LOW);
      digitalWrite(R4_PIN, LOW);
      digitalWrite(R5_PIN, LOW);
      break;
    case 3:
      pinMode(R1_PIN, INPUT);
      pinMode(R4_PIN, INPUT);
      break;


    case 6: i = 0; break;
  }
}




void setup() {
  pinMode(LED13_PIN, OUTPUT);
  pinMode(LED14_PIN, OUTPUT);
  pinMode(MENU_PIN, INPUT_PULLUP);

  timCharlie.attachInterrupt(charliehandler);
  timCharlie.setMode(1, TIMER_DISABLED);
  timCharlie.setOverflow(6000, HERTZ_FORMAT);
  timCharlie.resume();

  touch_setup();
  start_conv = true;
}


/**
  * @brief  Acquisition completed callback in non blocking mode 
  * @param  htsc: pointer to a TSC_HandleTypeDef structure that contains
  *         the configuration information for the specified TSC.
  * @retval None
  */
void HAL_TSC_ConvCpltCallback(TSC_HandleTypeDef* htsc) {
  const AcquisitionPhase& cphase = kPhases[current_phase];
  /*##-5- Discharge the touch-sensing IOs ####################################*/
  HAL_TSC_IODischarge(&TscHandle, ENABLE);
  /* Note: a delay can be added here */

  /*##-6- Check if the First channel acquisition is correct (no max count) ###*/
  if (HAL_TSC_GroupGetStatus(&TscHandle, TSC_GROUP2_IDX) == TSC_GROUP_COMPLETED) {
    channel_results[cphase.group2_rc][cphase.group2_num] = HAL_TSC_GroupGetValue(&TscHandle, TSC_GROUP2_IDX);
  } else {
    channel_results[cphase.group2_rc][cphase.group2_num] = 1;
  }

  if (HAL_TSC_GroupGetStatus(&TscHandle, TSC_GROUP3_IDX) == TSC_GROUP_COMPLETED) {
    channel_results[cphase.group3_rc][cphase.group3_num] = HAL_TSC_GroupGetValue(&TscHandle, TSC_GROUP3_IDX);
  } else {
    channel_results[cphase.group3_rc][cphase.group3_num] = 1;
  }

  if (HAL_TSC_GroupGetStatus(&TscHandle, TSC_GROUP5_IDX) == TSC_GROUP_COMPLETED) {
    channel_results[cphase.group5_rc][cphase.group5_num] = HAL_TSC_GroupGetValue(&TscHandle, TSC_GROUP5_IDX);
  } else {
    channel_results[cphase.group5_rc][cphase.group5_num] = 1;
  }
  start_conv = true;
}

void loop() {
  static int i = 0;
  i++;
  delay(1);

  /*
  pinMode(R3_PIN, INPUT);
  pinMode(R1_PIN, OUTPUT);
  pinMode(R2_PIN, OUTPUT);
  digitalWrite(R1_PIN, LOW);
  digitalWrite(R2_PIN, HIGH);
  pinMode(R6_PIN, INPUT);
  pinMode(R4_PIN, OUTPUT);
  pinMode(R5_PIN, OUTPUT);
  digitalWrite(R4_PIN, LOW);
  digitalWrite(R5_PIN, HIGH);
  */

  if (start_conv) {
    start_conv = false;
    if (current_phase == 0) {
      static char buf[1000];
      static int kk = 0;

      snprintf(buf, sizeof(buf), "%d: R0 %d R1 %d R2 %d R3 %d| C0 %d C1 %d C2 %d C3 %d", kk, channel_results[0][0], channel_results[0][1], channel_results[0][2], channel_results[0][3],
               channel_results[1][0], channel_results[1][1], channel_results[1][2], channel_results[1][3]);
      ++kk;
      while (Serial.availableForWrite() < strlen(buf) + 2);
      Serial.println(buf);
    }
    /*##-3- Discharge the touch-sensing IOs ####################################*/
    HAL_TSC_IODischarge(&TscHandle, ENABLE);
    delay(10); /* 1 ms is more than enough to discharge all capacitors */

    touch_iostart(++current_phase);
    /*##-4- Start the acquisition process ######################################*/
    if (HAL_TSC_Start_IT(&TscHandle) != HAL_OK) {
      /* Acquisition Error */
      Error_Handler();
    }
  }

  //digitalWrite(LED14_PIN, 0);
  //digitalWrite(LED14_PIN, digitalRead(MENU_PIN));
  if (i % 6 >= 1) {
    digitalWrite(LED13_PIN, HIGH);
    digitalWrite(LED14_PIN, HIGH);
  } else {
    digitalWrite(LED13_PIN, LOW);
    digitalWrite(LED14_PIN, LOW);
  }
}