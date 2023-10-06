#include "USBSerial.h"


static constexpr uint64_t kEventPrefix = UINT64_C(0x09000D0000000000);

// Implement this function for handling events coming from the local bus.
extern void OnGlobalEvent(uint64_t event);

// Call this function to send a broadcast event to the global bus.
extern void SendEvent(uint64_t event_id);

// Call this function once from setup().
extern void GlobalBusSetup();

// Call this function from the loop() handler.
extern void GlobalBusLoop();


// ============================ IMPLEMENTATION =========================

/** \copyright
 * Copyright (c) 2012-2023, Stuart W Baker, Balazs Racz
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *  - Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  - Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdint.h>

struct can_frame {
  union {
    uint32_t raw[4];
    struct
    {
      uint32_t can_id; /**< 11- or 29-bit ID (3-bits unsed) */

      uint8_t can_dlc : 4; /**< 4-bit DLC */
      uint8_t can_rtr : 1; /**< RTR indication */
      uint8_t can_eff : 1; /**< Extended ID indication */
      uint8_t can_err : 1; /**< @todo not supported by nuttx */
      uint8_t can_res : 1; /**< Unused */

      uint8_t pad;  /**< padding */
      uint8_t res0; /**< reserved */
      uint8_t res1; /**< reserved */

      union {
        /** CAN message data (64-bit) */
        uint64_t data64 __attribute__((aligned(8)));
        /** CAN message data (0-8 byte) */
        uint8_t data[8] __attribute__((aligned(8)));
      };
    };
  };
};

#define SET_CAN_FRAME_EFF(_frame) (_frame).can_eff = 1
#define SET_CAN_FRAME_RTR(_frame) (_frame).can_rtr = 1
#define SET_CAN_FRAME_ERR(_frame) (_frame).can_err = 1
#define CLR_CAN_FRAME_EFF(_frame) (_frame).can_eff = 0
#define CLR_CAN_FRAME_RTR(_frame) (_frame).can_rtr = 0
#define CLR_CAN_FRAME_ERR(_frame) (_frame).can_err = 0
#define IS_CAN_FRAME_EFF(_frame) ((_frame).can_eff)
#define IS_CAN_FRAME_RTR(_frame) ((_frame).can_rtr)
#define IS_CAN_FRAME_ERR(_frame) ((0))

#define GET_CAN_FRAME_ID_EFF(_frame) (_frame).can_id
#define GET_CAN_FRAME_ID(_frame) (_frame).can_id
#define SET_CAN_FRAME_ID_EFF(_frame, _value) (_frame).can_id = ((_value)&0x1FFFFFFFU)
#define SET_CAN_FRAME_ID(_frame, _value) (_frame).can_id = ((_value)&0x7ffU)


void GlobalBusSetup() {
  __HAL_RCC_GPIOB_CLK_ENABLE();
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

  /* Activate filter and exit initialization mode. */
  CAN->FA1R = 0x000000001;
  CAN->FMR &= ~CAN_FMR_FINIT;
}

bool try_send_can_frame(const struct can_frame &can_frame) {
  /* load the next message to transmit */
  volatile CAN_TxMailBox_TypeDef *mailbox;
  if (CAN->TSR & CAN_TSR_TME0) {
    mailbox = CAN->sTxMailBox + 0;
  } else if (CAN->TSR & CAN_TSR_TME1) {
    mailbox = CAN->sTxMailBox + 1;
  } else if (CAN->TSR & CAN_TSR_TME2) {
    mailbox = CAN->sTxMailBox + 2;
  } else {
    // no buffer available
    return false;
  }

  /* setup frame */
  if (can_frame.can_eff) {
    mailbox->TIR = (can_frame.can_id << 3) | CAN_TI0R_IDE;
  } else {
    mailbox->TIR = can_frame.can_id << 21;
  }
  if (can_frame.can_rtr) {
    mailbox->TIR |= CAN_TI0R_RTR;
  } else {
    mailbox->TDTR = can_frame.can_dlc;
    mailbox->TDLR = (can_frame.data[0] << 0) | (can_frame.data[1] << 8) | (can_frame.data[2] << 16) | (can_frame.data[3] << 24);
    mailbox->TDHR = (can_frame.data[4] << 0) | (can_frame.data[5] << 8) | (can_frame.data[6] << 16) | (can_frame.data[7] << 24);
  }

  /* request transmission */
  mailbox->TIR |= CAN_TI0R_TXRQ;

  return true;
}

bool read_can_frame(struct can_frame *can_frame) {
  if (!(CAN->RF0R & CAN_RF0R_FMP0)) {
    return false;
  }

  /* Read a message from CAN and clear the interrupt source */
  if (CAN->sFIFOMailBox[0].RIR & CAN_RI0R_IDE) {
    /* extended frame */
    can_frame->can_id = CAN->sFIFOMailBox[0].RIR >> 3;
    can_frame->can_eff = 1;
  } else {
    /* standard frame */
    can_frame->can_id = CAN->sFIFOMailBox[0].RIR >> 21;
    can_frame->can_eff = 0;
  }
  if (CAN->sFIFOMailBox[0].RIR & CAN_RI0R_RTR) {
    /* remote frame */
    can_frame->can_rtr = 1;
    can_frame->can_dlc = 0;
  } else {
    /* data frame */
    can_frame->can_rtr = 0;
    can_frame->can_dlc = CAN->sFIFOMailBox[0].RDTR & CAN_RDT0R_DLC;
    can_frame->data[0] = (CAN->sFIFOMailBox[0].RDLR >> 0) & 0xFF;
    can_frame->data[1] = (CAN->sFIFOMailBox[0].RDLR >> 8) & 0xFF;
    can_frame->data[2] = (CAN->sFIFOMailBox[0].RDLR >> 16) & 0xFF;
    can_frame->data[3] = (CAN->sFIFOMailBox[0].RDLR >> 24) & 0xFF;
    can_frame->data[4] = (CAN->sFIFOMailBox[0].RDHR >> 0) & 0xFF;
    can_frame->data[5] = (CAN->sFIFOMailBox[0].RDHR >> 8) & 0xFF;
    can_frame->data[6] = (CAN->sFIFOMailBox[0].RDHR >> 16) & 0xFF;
    can_frame->data[7] = (CAN->sFIFOMailBox[0].RDHR >> 24) & 0xFF;
  }
  /* release FIFO */;
  CAN->RF0R |= CAN_RF0R_RFOM0;
  return true;
}

void SendEvent(uint64_t ev) {
  struct can_frame f;
  SET_CAN_FRAME_ID_EFF(f, 0x195b4123);
  ev = __builtin_bswap64(ev);
  memcpy(f.data, &ev, 8);
  f.can_dlc = 8;
  if (!try_send_can_frame(f)) {
    SerialUSB.printf("error sending CAN frame.\n");
  }
}

void GlobalBusLoop() {
  struct can_frame f;
  if (read_can_frame(&f) && IS_CAN_FRAME_EFF(f) && f.can_dlc == 8) {
    uint32_t id = GET_CAN_FRAME_ID_EFF(f);
    if ((id & ~0xFFF) == 0x195b4000) {
      // Event arrived.
      uint64_t ev = 0;
      memcpy(&ev, f.data, 8);
      ev = __builtin_bswap64(ev);
      OnGlobalEvent(ev);
    }
  }
}
