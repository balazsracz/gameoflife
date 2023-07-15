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

bool startConv = false;

/* Array used to store the three acquisition value (one per channel) */
__IO uint32_t uhTSCAcquisitionValue[3];

uint8_t IdxBank = 0;
TSC_IOConfigTypeDef IoConfig;

extern "C" {
  void TSC_IRQHandler(void) {
    HAL_TSC_IRQHandler(&TscHandle);
  }
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

  GPIO_InitStruct.Pin = GPIO_PIN_5;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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

  /*##-2- Configure the touch-sensing IOs ####################################*/
  IoConfig.ChannelIOs = TSC_GROUP2_IO2;  // | TSC_GROUP2_IO3 | TSC_GROUP2_IO4; /* Start with the first channel */
  IoConfig.SamplingIOs = (TSC_GROUP2_IO1);
  IoConfig.ShieldIOs = 0;

  if (HAL_TSC_IOConfig(&TscHandle, &IoConfig) != HAL_OK) {
    /* Initialization Error */
    Error_Handler();
  }

  startConv = true;
}


/**
  * @brief  Acquisition completed callback in non blocking mode 
  * @param  htsc: pointer to a TSC_HandleTypeDef structure that contains
  *         the configuration information for the specified TSC.
  * @retval None
  */
void HAL_TSC_ConvCpltCallback(TSC_HandleTypeDef* htsc) {
  /*##-5- Discharge the touch-sensing IOs ####################################*/
  HAL_TSC_IODischarge(&TscHandle, ENABLE);
  /* Note: a delay can be added here */

  /*##-6- Check if the First channel acquisition is correct (no max count) ###*/
  if (HAL_TSC_GroupGetStatus(&TscHandle, TSC_GROUP2_IDX) == TSC_GROUP_COMPLETED) {
    /*##-7- Read the acquisition value #######################################*/
    uhTSCAcquisitionValue[0] = HAL_TSC_GroupGetValue(&TscHandle, TSC_GROUP2_IDX);
    startConv = true;
    /* Note: Show the touch activity on LEDs.
       The threshold values are indicative and may need to be adjusted */
    if ((uhTSCAcquisitionValue[0] > 1000) && (uhTSCAcquisitionValue[0] < 1000))  // Channel 1
    {
    } else {
    }
  }

#if 0
  /*##-8- Check if the Second channel acquisition is correct (no max count) ##*/
  if (HAL_TSC_GroupGetStatus(&TscHandle, TSC_GROUP2_IDX) == TSC_GROUP_COMPLETED)
  {
    /*##-9- Read the acquisition value #######################################*/
    uhTSCAcquisitionValue[IdxBank] = HAL_TSC_GroupGetValue(&TscHandle, TSC_GROUP2_IDX);  
    /* Note: Show the touch activity on LEDs.
       The threshold values are indicative and may need to be adjusted */
    if ((uhTSCAcquisitionValue[1] > 1000) && (uhTSCAcquisitionValue[1] < TSCx_TS2_MAXTHRESHOLD)) // Channel 2
    {
      BSP_LED_On(LED6);
    }
    else
    {
      BSP_LED_Off(LED6);
    }
  }

  /*##-10- Check if the Third channel acquisition is correct (no max count) ###*/
  if (HAL_TSC_GroupGetStatus(&TscHandle, TSC_GROUP3_IDX) == TSC_GROUP_COMPLETED)
  {
    /*##-11- Read the acquisition value #######################################*/
    uhTSCAcquisitionValue[IdxBank] = HAL_TSC_GroupGetValue(&TscHandle, TSC_GROUP3_IDX);  
    /* Note: Show the touch activity on LEDs.
       The threshold values are indicative and may need to be adjusted */
    if ((uhTSCAcquisitionValue[2] > 1000) && (uhTSCAcquisitionValue[2] < TSCx_TS3_MAXTHRESHOLD)) // Channel 3
    {
      BSP_LED_On(LED5);
    }
    else
    {
      BSP_LED_Off(LED5);
    }
  }

  /*##-12- Configure the next channels to be acquired #########################*/
  switch (IdxBank)
  {
    case 0:
      IoConfig.ChannelIOs = TSC_GROUP2_IO3; /* Second channel */
      IdxBank = 1;
      break;
    case 1:
      IoConfig.ChannelIOs = TSC_GROUP3_IO2; /* Third channel */
      IdxBank = 2;
      break;
    case 2:
      IoConfig.ChannelIOs = TSC_GROUP1_IO3; /* First channel */
      IdxBank = 0;
      break;
    default:
      break;
  }
#endif
  if (HAL_TSC_IOConfig(&TscHandle, &IoConfig) != HAL_OK) {
    /* Initialization Error */
    Error_Handler();
  }

#if 0 
  /*##-13- Re-start the acquisition process ###################################*/
  if (HAL_TSC_Start_IT(&TscHandle) != HAL_OK)
  {
    /* Acquisition Error */
    Error_Handler();
  }
#endif
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

  if (startConv) {
    static int kk = 0;
    ++kk;
    Serial.print("conv ");
    Serial.print(kk);
    Serial.print(" ");
    Serial.println(uhTSCAcquisitionValue[0]);
    startConv = false;
    /*##-3- Discharge the touch-sensing IOs ####################################*/
    HAL_TSC_IODischarge(&TscHandle, ENABLE);
    delay(1); /* 1 ms is more than enough to discharge all capacitors */

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