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

// #define HW_REV1
 #define HW_REV2


#if defined (HW_REV1) + defined(HW_REV2) != 1
#error must define one of HW_REV1 and HW_REV2.
#endif

static constexpr int LED13_PIN = PF0;
static constexpr int LED14_PIN = PF1;
#ifdef HW_REV1
static constexpr int LED15_PIN = PB13;
static constexpr int LED16_PIN = PB14;

static constexpr int MENU_PIN = PB15;
#endif

static constexpr int R1_PIN = PC14;
static constexpr int R2_PIN = PC15;
static constexpr int R3_PIN = PC13;

static constexpr int R4_PIN = PA15;
static constexpr int R5_PIN = PB12;
static constexpr int R6_PIN = PA8;

#ifdef HW_REV2
static constexpr int R15_PIN = PB13;
static constexpr int R16_PIN = PB14;
#endif

static constexpr int NORTH_TX_PIN = PA2;
static constexpr int NORTH_RX_PIN = PA3;
static constexpr int SOUTH_TX_PIN = PA9;
static constexpr int SOUTH_RX_PIN = PA10;
static constexpr int EAST_TX_PIN = PB10;
static constexpr int EAST_RX_PIN = PB11;
static constexpr int WEST_TX_PIN = PA0;
static constexpr int WEST_RX_PIN = PA1;

static constexpr int DBG2_PIN = PB15;


HardwareSerial serial_north(NORTH_RX_PIN, NORTH_TX_PIN);
HardwareSerial serial_south(SOUTH_RX_PIN, SOUTH_TX_PIN);
HardwareSerial serial_east(EAST_RX_PIN, EAST_TX_PIN);
HardwareSerial serial_west(WEST_RX_PIN, WEST_TX_PIN);


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

// Stores average value over all readings of the channel. This is the sum.
uint64_t channel_sum[2][4] = { 0 };
// Stores average value over all readings of the channel. This is the count.
unsigned channel_count[2][4] = { 0 };

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
#elif defined (HW_REV2)
static const AcquisitionPhase kPhases[kNumPhases] = {
  { TSC_GROUP2_IO2 | TSC_GROUP3_IO3 | TSC_GROUP5_IO2, kChCol, 0, kChRow, 1, kChCol, 2 },
  { TSC_GROUP2_IO3 | TSC_GROUP3_IO4 | TSC_GROUP5_IO3, kChRow, 2, kChCol, 1, kChCol, 3 },
  { TSC_GROUP2_IO4 | TSC_GROUP3_IO3 | TSC_GROUP5_IO4, kChRow, 0, kChRow, 1, kChRow, 3 }
};
#endif

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

// Updates the calibration data structure with the more recent measurement results.
void touch_calibrate() {
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
static constexpr unsigned kRelThresholdNom = 8;
static constexpr unsigned kRelThresholdDenom = 10;


bool touch_eval(unsigned rc, unsigned num) {
  int ref = channel_sum[rc][num] / channel_count[rc][num];
  int actual = channel_results[rc][num];
  if (actual >= ref - kMinDeltaThreshold) return false;
  if (actual < ref - kDeltaThreshold) return true;
  if (actual < ref * kRelThresholdNom / kRelThresholdDenom) return true;
  return false;
}

int tdeb(unsigned rc, unsigned num) {
  return (channel_sum[rc][num] / channel_count[rc][num]) - channel_results[rc][num];
}

#define BLINKER_TIMER TIM14

HardwareTimer timCharlie(BLINKER_TIMER);

static bool leds[17];

struct CharlieProgram {
  // Which pin to set to positive voltage.
  int pin_hi_;
  // Which pin to set to negative voltage.
  int pin_lo_;
  // Which pin to set to input.
  int pin_z_;
  // Which led to output.
  int led_no_;
};

const CharlieProgram pg1[6] = {
  { R1_PIN, R2_PIN, R3_PIN, 0 },
  { R2_PIN, R1_PIN, R3_PIN, 1 },
  { R1_PIN, R3_PIN, R2_PIN, 2 },
  { R3_PIN, R1_PIN, R2_PIN, 3 },
  { R2_PIN, R3_PIN, R1_PIN, 4 },
  { R3_PIN, R2_PIN, R1_PIN, 5 },
};

const CharlieProgram pg2[6] = {
  { R5_PIN, R4_PIN, R6_PIN, 6 },
  { R4_PIN, R5_PIN, R6_PIN, 7 },
  { R6_PIN, R4_PIN, R5_PIN, 8 },
  { R4_PIN, R6_PIN, R5_PIN, 9 },
  { R6_PIN, R5_PIN, R4_PIN, 10 },
  { R5_PIN, R6_PIN, R4_PIN, 11 },
};

const CharlieProgram pg3[6] = {
  { R15_PIN, R16_PIN, DBG2_PIN, 14 },
  { R16_PIN, R15_PIN, DBG2_PIN, 15 },
  { DBG2_PIN, R16_PIN, R15_PIN, 16 },
  { DBG2_PIN, DBG2_PIN, R16_PIN, 16 },
  { DBG2_PIN, DBG2_PIN, DBG2_PIN, 16 },
  { DBG2_PIN, DBG2_PIN, DBG2_PIN, 16 },
};


void charlie_apply(const struct CharlieProgram* pgm, int idx) {
  if (idx >= 6) return;
  const auto& p = pgm[idx];
  pinMode(p.pin_z_, INPUT);
  pinMode(p.pin_lo_, INPUT);
  pinMode(p.pin_hi_, INPUT);
  pinMode(p.pin_lo_, OUTPUT);
  digitalWrite(p.pin_lo_, LOW);
  digitalWrite(p.pin_hi_, LOW);
  pinMode(p.pin_hi_, OUTPUT);
  digitalWrite(p.pin_hi_, leds[p.led_no_] ? HIGH : LOW);
}

void charliehandler(void) {
  static int i = 0;
  i++;
  if (i >= 6) i = 0;
  charlie_apply(pg1, i);
  charlie_apply(pg2, i);
  charlie_apply(pg3, i);
  return;
  /*
  pinMode(R6_PIN, INPUT);
  pinMode(R4_PIN, OUTPUT);
  pinMode(R5_PIN, OUTPUT);
  digitalWrite(R4_PIN, LOW);
  digitalWrite(R5_PIN, HIGH);
  return;
  */
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
      i = 0;
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

CAN_HandleTypeDef CanHandle;
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8];
uint8_t RxData[8];

void can_setup() {
  __HAL_RCC_CAN1_CLK_ENABLE();
  GPIO_InitTypeDef GPIO_InitStruct;
  memset(&GPIO_InitStruct, 0, sizeof(GPIO_InitStruct));

  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Alternate = GPIO_AF4_CAN;

  GPIO_InitStruct.Pin = GPIO_PIN_8;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  memset(&CanHandle, 0, sizeof(CanHandle));

  /*##-1- Configure the CAN peripheral #######################################*/
  CanHandle.Instance = CAN;

  CanHandle.Init.TimeTriggeredMode = DISABLE;
  CanHandle.Init.AutoBusOff = DISABLE;
  CanHandle.Init.AutoWakeUp = DISABLE;
  CanHandle.Init.AutoRetransmission = ENABLE;
  CanHandle.Init.ReceiveFifoLocked = DISABLE;
  CanHandle.Init.TransmitFifoPriority = DISABLE;

  /*  CanHandle.Init.TimeTriggeredMode = DISABLE;
  CanHandle.Init.AutoBusOff = ENABLE;
  CanHandle.Init.AutoWakeUp = ENABLE;
  CanHandle.Init.AutoRetransmission = ENABLE;
  CanHandle.Init.ReceiveFifoLocked = ENABLE;
  CanHandle.Init.TransmitFifoPriority = ENABLE;*/
  CanHandle.Init.Mode = CAN_MODE_NORMAL;
  CanHandle.Init.SyncJumpWidth = CAN_SJW_1TQ;
  CanHandle.Init.TimeSeg1 = CAN_BS1_5TQ;
  CanHandle.Init.TimeSeg2 = CAN_BS2_6TQ;
  CanHandle.Init.Prescaler = 48;

  if (HAL_CAN_Init(&CanHandle) != HAL_OK) {
    /* Initialization Error */
    Error_Handler();
  }

  /*##-2- Configure the CAN Filter ###########################################*/
  CAN_FilterTypeDef sFilterConfig;
  memset(&sFilterConfig, 0, sizeof(sFilterConfig));
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&CanHandle, &sFilterConfig) != HAL_OK) {
    /* Filter configuration Error */
    Error_Handler();
  }

  /* disable sleep, enter init mode */
  CAN->MCR = CAN_MCR_INRQ;

  /* Time triggered tranmission off
     * Bus off state is left automatically
     * Auto-Wakeup mode disabled
     * automatic re-transmission enabled
     * receive FIFO not locked on overrun
     * TX FIFO mode on
     */
  CAN->MCR |= (CAN_MCR_ABOM | CAN_MCR_TXFP);

  /* Setup timing.
     * 125,000 Kbps = 8 usec/bit
     */
  CAN->BTR = (CAN_BS1_5TQ | CAN_BS2_2TQ | CAN_SJW_1TQ | (48 - 1));

  /* enter normal mode */
  CAN->MCR &= ~CAN_MCR_INRQ;

  /* Enter filter initialization mode.  Filter 0 will be used as a single
     * 32-bit filter, ID Mask Mode, we accept everything, no mask.
     */
  CAN->FMR |= CAN_FMR_FINIT;
  CAN->FM1R = 0;
  CAN->FS1R = 0x000000001;
  CAN->FFA1R = 0;
  CAN->sFilterRegister[0].FR1 = 0;
  CAN->sFilterRegister[0].FR2 = 0;

  /* Activeate filter and exit initialization mode. */
  CAN->FA1R = 0x000000001;
  CAN->FMR &= ~CAN_FMR_FINIT;


  /*##-3- Configure Transmission process #####################################*/
  TxHeader.StdId = 0x321;
  TxHeader.ExtId = 0x195b4123;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_EXT;
  TxHeader.DLC = 8;
  TxHeader.TransmitGlobalTime = DISABLE;

  TxData[0] = 0x02;
  TxData[1] = 0x01;
  TxData[2] = 0x0D;
  uint32_t* puid = (uint32_t*)0x1FFFF7AC;
  uint32_t u = puid[0] ^ puid[1] ^ puid[2];
  memcpy(TxData + 3, &u, 4);
  TxData[7] = 0;
}

bool send_can_frame(uint32_t suffix) {
  TxData[7] = suffix & 0xff;
  uint32_t mailbox;
  int ret;
  if ((ret = HAL_CAN_AddTxMessage(&CanHandle, &TxHeader, TxData, &mailbox)) != HAL_OK) {
    SerialUSB.printf("can tx: error %d", ret);
    return false;
  }
  return true;
}

void loop_can() {
  if (HAL_CAN_GetRxMessage(&CanHandle, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
    uint32_t adr = 0;
    memcpy(&adr, RxData + 3, 4);
    SerialUSB.printf("CAN RX [%x]: %08x: 0x%02x\n", RxHeader.ExtId, adr, RxData[7]);
  }
}

void bus_setup() {
  serial_north.begin(9600);
  serial_south.begin(9600);
  serial_east.begin(9600);
  serial_west.begin(9600);
  for (auto pin : { LL_GPIO_PIN_0, LL_GPIO_PIN_1, LL_GPIO_PIN_2, LL_GPIO_PIN_3, LL_GPIO_PIN_9, LL_GPIO_PIN_10 }) {
    LL_GPIO_SetPinOutputType(GPIOA, pin, LL_GPIO_OUTPUT_OPENDRAIN);
    LL_GPIO_SetPinPull(GPIOA, pin, LL_GPIO_PULL_UP);
  }
  for (auto pin : { LL_GPIO_PIN_8, LL_GPIO_PIN_9, LL_GPIO_PIN_10, LL_GPIO_PIN_11 }) {
    LL_GPIO_SetPinOutputType(GPIOB, pin, LL_GPIO_OUTPUT_OPENDRAIN);
    LL_GPIO_SetPinPull(GPIOB, pin, LL_GPIO_PULL_UP);
  }
  can_setup();
}

uint32_t next_print = 0;


void setup() {
  pinMode(LED13_PIN, OUTPUT);
  pinMode(LED14_PIN, OUTPUT);
#ifdef HW_REV1  
  pinMode(LED15_PIN, OUTPUT);
  pinMode(LED16_PIN, OUTPUT);

  pinMode(MENU_PIN, INPUT_PULLUP);
#endif

  timCharlie.attachInterrupt(charliehandler);
  timCharlie.setMode(1, TIMER_DISABLED);
  timCharlie.setOverflow(600, HERTZ_FORMAT);
  timCharlie.resume();

  touch_setup();
  start_conv = true;

  next_print = HAL_GetTick() + 100;

  bus_setup();
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
  start_conv = true;
}

void serial_test() {
  static bool south_active = false;
  if (touch_eval(kChRow, 0)) {
    if (!south_active) {
      serial_south.write(0x01);
      SerialUSB.printf("Signaled south\n");
      south_active = true;
    }
  } else {
    south_active = false;
  }
  static bool west_active = false;
  if (touch_eval(kChRow, 1)) {
    if (!west_active) {
      serial_west.write(0x01);
      SerialUSB.printf("Signaled west\n");
      west_active = true;
    }
  } else {
    west_active = false;
  }
  static bool north_active = false;
  if (touch_eval(kChRow, 2)) {
    if (!north_active) {
      serial_north.write(0x83);
      SerialUSB.printf("Signaled north\n");
      north_active = true;
    }
  } else {
    north_active = false;
  }
  static bool east_active = false;
  if (touch_eval(kChRow, 3)) {
    if (!east_active) {
      serial_east.write(0x83);
      SerialUSB.printf("Signaled east\n");
      east_active = true;
    }
  } else {
    east_active = false;
  }

  // Handle input data
  int rd = -1;
  rd = serial_south.read();
  if (rd >= 0) {
    if ((rd & 0x80) == 0) {
      SerialUSB.printf("Echo south: %02x\n", rd);
    } else {
      SerialUSB.printf("Rcv south: %02x\n", rd);
      leds[13] = !leds[13];
    }
  }
  rd = serial_west.read();
  if (rd >= 0) {
    if ((rd & 0x80) == 0) {
      SerialUSB.printf("Echo west: %02x\n", rd);
    } else {
      SerialUSB.printf("Rcv west: %02x\n", rd);
      leds[13] = !leds[13];
    }
  }
  rd = serial_north.read();
  if (rd >= 0) {
    if ((rd & 0x80) != 0) {
      SerialUSB.printf("Echo north: %02x\n", rd);
    } else {
      SerialUSB.printf("Rcv north: %02x\n", rd);
      leds[12] = !leds[12];
    }
  }
  rd = serial_east.read();
  if (rd >= 0) {
    if ((rd & 0x80) != 0) {
      SerialUSB.printf("Echo east: %02x\n", rd);
    } else {
      SerialUSB.printf("Rcv east: %02x\n", rd);
      leds[12] = !leds[12];
    }
  }
}


static int last_press = -1;

void process_press(int btn);

void can_send_test() {
  int current_row = -1;
  int current_col = -1;

  int num_rows = 0;
  int num_cols = 0;


  for (unsigned i = 0; i <= 3; i++) {
    if (touch_eval(kChRow, i)) {
      ++num_rows;
      current_row = i;
    }
    if (touch_eval(kChCol, i)) {
      ++num_cols;
      current_col = i;
    }
  }

  //SerialUSB.printf("nr %d nc %d crow %d cc %d c3sum %ld c3curr %d\n", num_rows, num_cols, current_row, current_col, (int)channel_sum[kChCol][3], channel_results[kChCol][3]);
  if (num_rows == 1 && num_cols == 1) {
    int current_press = current_row * 4 + current_col;
    if (current_press != last_press) {
      last_press = current_press;
      process_press(current_press);
      if (!send_can_frame(current_press)) {
        SerialUSB.println("send failed.");
      }
      SerialUSB.printf("Button press %d\n", last_press);
    }
  } else {
    last_press = -1;
  }
}

void process_press(int btn) {
  if (btn == 0) {
    memset(leds, 1, sizeof(leds));
    leds[14]=leds[15] = 0;
  } else if (btn == 1) {
    memset(leds, 0, sizeof(leds));
  } else if (btn == 2) {
    leds[6] = 1;
    memset(leds + 7, 0, 5);
  } else if (btn == 4) {
    bool tmp = leds[11];
    memmove(leds + 7, leds + 6, 5);
    leds[6] = tmp;
  }
}



void loop() {
  leds[16] = 0;
  static int i = 0;
  i++;
  delay(1);
  loop_can();
  digitalWrite(LED13_PIN, leds[12] ? LOW : HIGH);
  digitalWrite(LED14_PIN, leds[13] ? LOW : HIGH);
#ifdef HW_REV1  
  digitalWrite(LED15_PIN, leds[14] ? LOW : HIGH);
  digitalWrite(LED16_PIN, leds[15] ? LOW : HIGH);
#endif  
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

  if (HAL_GetTick() >= next_print) {
    touch_calibrate();
    can_send_test();
    next_print = HAL_GetTick() + 100;
    static int kk = 0;

    //SerialUSB.printf("%d: R0 %5d R1 %5d R2 %5d R3 %5d| C0 %5d C1 %5d C2 %5d C3 %5d\n", kk, tdeb(0, 0), tdeb(0, 1), tdeb(0, 2), tdeb(0, 3), tdeb(1, 0), tdeb(1, 1), tdeb(1, 2), tdeb(1, 3));
    ++kk;
    //while (Serial.availableForWrite() < strlen(buf) + 2);
    //SerialUSB.println(buf);
    SerialUSB.print(".");
    if (kk % 80 == 0) {
      SerialUSB.print("\n");
    }

    serial_test();
  }

  if (start_conv) {
    start_conv = false;

    /*##-3- Discharge the touch-sensing IOs ####################################*/
    HAL_TSC_IODischarge(&TscHandle, ENABLE);
    delay(1); /* 1 ms is more than enough to discharge all capacitors */

    touch_iostart(++current_phase);
    /*##-4- Start the acquisition process ######################################*/
    if (HAL_TSC_Start_IT(&TscHandle) != HAL_OK) {
      /* Acquisition Error */
      Error_Handler();
    }
  }
}
