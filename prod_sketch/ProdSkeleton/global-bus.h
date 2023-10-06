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
#include <string>

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

extern uint64_t nmranet_nodeid(void);

namespace openlcb {

using std::string;

/** 48-bit NMRAnet Node ID type */
typedef uint64_t NodeID;

/** Alias to a 48-bit NMRAnet Node ID type */
typedef uint16_t NodeAlias;

/// Container that carries the data bytes in an NMRAnet message.
typedef string Payload;

/// Guard value put into the the internal node alias maps when a node ID could
/// not be translated to a valid alias.
static const NodeAlias NOT_RESPONDING = 0xF000;

/** Container of both a NodeID and NodeAlias */
struct NodeHandle
{
    NodeID id; /**< 48-bit NMRAnet Node ID */
    NodeAlias alias; /**< alias to NMRAnet Node ID */

    explicit NodeHandle(NodeID _id) : id(_id), alias(0) {}
    explicit NodeHandle(NodeAlias _alias) : id(0), alias(_alias) {}
    NodeHandle(NodeID _id, NodeAlias _alias) : id(_id), alias(_alias) {}
    NodeHandle() : id(0), alias(0) {}

    /** Resets node handle to global (broadcast) handle. */
    void clear()
    {
        id = 0;
        alias = 0;
    }

    /** Compares two NodeHandle instances.
     * @param o object to compare to
     * @return boolean result of compare
     */
    bool operator==(const NodeHandle& o) const
    {
        return id == o.id && alias == o.alias;
    }
};

/** Allowed states of producers and consumers. */
enum class EventState
{
    VALID = 0,
    INVALID = 1,
    RESERVED = 2,
    UNKNOWN = 3
};

/** The generic interface for NMRAnet network interfaces
 */
struct Defs
{
    /** Known Message type indicators.
     */
    enum MTI
    {
        MTI_EXACT                     = 0xFFFF, /**< match mask for a single MTI */
        MTI_NONE                      = 0x0000, /**< invalid MTI */
        MTI_INITIALIZATION_COMPLETE   = 0x0100, /**< initialization complete */
        MTI_VERIFY_NODE_ID_ADDRESSED  = 0x0488, /**< verify a Node ID */
        MTI_VERIFY_NODE_ID_GLOBAL     = 0x0490, /**< verify a Node ID globally */
        MTI_VERIFIED_NODE_ID_NUMBER   = 0x0170, /**< respond to a verify Node ID request */
        MTI_OPTIONAL_INTERACTION_REJECTED = 0x0068, /**< rejected request */
        MTI_TERMINATE_DUE_TO_ERROR    = 0x00A8, /**< terminate due to some error */
        MTI_PROTOCOL_SUPPORT_INQUIRY  = 0x0828, /**< inquire on supported protocols */
        MTI_PROTOCOL_SUPPORT_REPLY    = 0x0668, /**< reply with supported protocols */
        MTI_CONSUMER_IDENTIFY         = 0x08F4, /**< query about consumers */
        MTI_CONSUMER_IDENTIFIED_RANGE   = 0x04A4, /**< consumer broadcast about a range of consumers */
        MTI_CONSUMER_IDENTIFIED_UNKNOWN = 0x04C7, /**< consumer broadcast, validity unknown */
        MTI_CONSUMER_IDENTIFIED_VALID   = 0x04C4, /**< consumer broadcast, valid state */
        MTI_CONSUMER_IDENTIFIED_INVALID = 0x04C5, /**< consumer broadcast, invalid state */
        MTI_CONSUMER_IDENTIFIED_RESERVED = 0x04C6, /**< reserved for future use */
        MTI_PRODUCER_IDENTIFY         = 0x0914, /**< query about producers */
        MTI_PRODUCER_IDENTIFIED_RANGE   = 0x0524, /**< producer broadcast about a range of producers */
        MTI_PRODUCER_IDENTIFIED_UNKNOWN = 0x0547, /**< producer broadcast, validity unknown */
        MTI_PRODUCER_IDENTIFIED_VALID   = 0x0544, /**< producer broadcast, valid state */
        MTI_PRODUCER_IDENTIFIED_INVALID = 0x0545, /**< producer broadcast, invalid state */
        MTI_PRODUCER_IDENTIFIED_RESERVED = 0x0546, /**< reserved for future use */
        MTI_EVENTS_IDENTIFY_ADDRESSED = 0x0968, /**< request identify all of a node's events */
        MTI_EVENTS_IDENTIFY_GLOBAL    = 0x0970, /**< request identify all of every node's events */
        MTI_LEARN_EVENT               = 0x0594, /**< */
        MTI_EVENT_REPORT              = 0x05B4, /**< */
        MTI_TRACTION_CONTROL_COMMAND  = 0x05EB,
        MTI_TRACTION_CONTROL_REPLY    = 0x01E9,
        MTI_TRACTION_PROXY_COMMAND    = 0x05EA,
        MTI_TRACTION_PROXY_REPLY      = 0x01E8,
        MTI_XPRESSNET                 = 0x09C0, /**< */
        MTI_IDENT_INFO_REQUEST        = 0x0DE8, /**< request node identity */
        MTI_IDENT_INFO_REPLY          = 0x0A08, /**< node identity reply */
        MTI_DATAGRAM                  = 0x1C48, /**< datagram */
        MTI_DATAGRAM_OK               = 0x0A28, /**< datagram received okay */
        MTI_DATAGRAM_REJECTED         = 0x0A48, /**< datagram rejected by receiver */
        MTI_STREAM_INITIATE_REQUEST   = 0x0CC8, /**< Stream initiate request */
        MTI_STREAM_INITIATE_REPLY     = 0x0868, /**< Stream initiate reply */
        MTI_STREAM_DATA               = 0x1F88, /**< stream data */
        MTI_STREAM_PROCEED            = 0x0888, /**< stream flow control */
        MTI_STREAM_COMPLETE           = 0x08A8, /**< stream terminate connection */
        
        MTI_MODIFIER_MASK = 0x0003, /**< modifier within Priority/Type mask */
        MTI_EVENT_MASK    = 0x0004, /**< event number present mask */
        MTI_ADDRESS_MASK  = 0x0008, /**< Address present mask */
        MTI_SIMPLE_MASK   = 0x0010, /**< simple protocol mask */
        MTI_TYPE_MASK     = 0x03e0, /**< type within priority mask */
        MTI_PRIORITY_MASK = 0x0c00, /**< priority mask */
        MTI_DATAGRAM_MASK = 0x1000, /**< stream or datagram mask */
        MTI_SPECIAL_MASK  = 0x2000, /**< special mask */
        MTI_RESERVED_MASK = 0xc000, /**< reserved mask */

        MTI_MODIFIER_SHIFT =  0, /**< modifier within Priority/Type shift */
        MTI_EVENT_SHIFT    =  2, /**< event number present shift */
        MTI_ADDRESS_SHIFT  =  3, /**< Address present shift */
        MTI_SIMPLE_SHIFT   =  4, /**< simple protocol shift */
        MTI_TYPE_SHIFT     =  5, /**< type within priority shift */
        MTI_PRIORITY_SHIFT = 10, /**< priority shift */
        MTI_DATAGRAM_SHIFT = 12, /**< stream or datagram shift */
        MTI_SPECIAL_SHIFT  = 13, /**< special shift */
        MTI_RESERVED_SHIFT = 14  /**< reserved shift */

    };

    enum ErrorCodes
    {
        ERROR_CODE_OK = 0,

        ERROR_PERMANENT = 0x1000,
        ERROR_TEMPORARY = 0x2000,

        ERROR_SRC_NOT_PERMITTED = 0x1020,
        ERROR_OPENMRN_NOT_FOUND = 0x1030,
        ERROR_UNIMPLEMENTED = 0x1040,
        ERROR_INVALID_ARGS = 0x1080,

        ERROR_OPENLCB_TIMEOUT = 0x2030,
        ERROR_OUT_OF_ORDER = 0x2040,

        ERROR_INVALID_ARGS_MESSAGE_TOO_SHORT = ERROR_INVALID_ARGS | 1,

        ERROR_UNIMPLEMENTED_MTI = ERROR_UNIMPLEMENTED | 3,
        ERROR_UNIMPLEMENTED_CMD = ERROR_UNIMPLEMENTED | 2,
        ERROR_UNIMPLEMENTED_SUBCMD = ERROR_UNIMPLEMENTED | 1,

        ERROR_OPENMRN_ALREADY_EXISTS = ERROR_OPENMRN_NOT_FOUND | 2,

        // Codes defined by the firmware upgrade standard

        /// The firmware data is incompatible with this hardware node.
        ERROR_FIRMWARE_INCOMPATIBLE = ERROR_INVALID_ARGS | 8,
        /// The firmware data is invalid or corrupted.
        ERROR_FIRMWARE_CORRUPTED = ERROR_INVALID_ARGS | 9,
        /// The firmware written has failed checksum (temporary error).
        ERROR_FIRMWARE_CSUM = 0x2088,

        // Internal error codes generated by the stack.
        ERROR_DST_NOT_FOUND = 0x40000, //< on CAN. Permanent error code.
        // There is a conflict with MinGW macros here.
        OPENMRN_TIMEOUT = 0x80000,       //< Timeout waiting for ack/nack.
        ERROR_DST_REBOOT = 0x100000,   //< Target node has rebooted.
        ERROR_REJECTED = 0x200000,     //< Target node has rejectedthe request.
    };

    /** Bitmask for all potentially supported NMRAnet protocols.
     */
    enum Protocols
    {
        SIMPLE_PROTOCOL_SUBSET  = 0x800000000000,
        DATAGRAM                = 0x400000000000,
        STREAM                  = 0x200000000000,
        MEMORY_CONFIGURATION    = 0x100000000000,
        RESERVATION             = 0x080000000000,
        EVENT_EXCHANGE          = 0x040000000000,
        IDENTIFICATION          = 0x020000000000,
        LEARN_CONFIGURATION     = 0x010000000000,
        REMOTE_BUTTON           = 0x008000000000,
        ABBREVIATED_DEFAULT_CDI = 0x004000000000,
        DISPLAY_PROTOCOL        = 0x002000000000,
        SIMPLE_NODE_INFORMATION = 0x001000000000,
        CDI                     = 0x000800000000,
        TRACTION_CONTROL        = 0x000400000000,
        TRACTION_FDI            = 0x000200000000,
        TRACTION_PROXY          = 0x000100000000,
        TRACTION_SIMPLE_TRAIN_INFO
                                = 0x000080000000,
        FUNCTION_CONFIGURATION  = 0x000040000000,
        FIRMWARE_UPGRADE        = 0x000020000000,
        FIRMWARE_UPGRADE_ACTIVE = 0x000010000000,
        RESERVED_MASK           = 0x00000FFFFFFF
    };

    /// "Emergency off (de-energize)"
    /// Producing this event causes an Emergency Off (de-energize).  For
    /// example, a DCC command station or booster may react to this by turning
    /// off the command station or booster power output.
    static constexpr uint64_t EMERGENCY_OFF_EVENT = 0x010000000000FFFFULL;

    /// "Clear emergency off (energize)"
    /// Producing this event clears an Emergency Off (energize).  For example,
    /// a DCC command station or booster mauy react to this by restoring
    /// track power.
    static constexpr uint64_t CLEAR_EMERGENCY_OFF_EVENT = 0x010000000000FFFEULL;

    /// "Emergency stop of all operations"
    /// The Emergency Stop Event is a request for a node to command all of its
    /// outputs to a safe state. A node receiving this event is not required
    /// to de-energize any of its outputs. The meaning of “safe state” is not
    /// prescribed for any given node, it is up to the node manufacturer and/or
    /// user to prescribe what, if anything, should happen in the node if it
    /// receives this event.
    /// For example, a DCC command station may react to this by sending a DCC
    /// emergency stop command packet to the track outputs.
    static constexpr uint64_t EMERGENCY_STOP_EVENT = 0x010000000000FFFDULL;

    /// "Clear Emergency stop of all operations"
    /// Producing this event clears an Emergency Stop. For example, a DCC command
    /// station may react to this by restoring locomotive speed settings.
    static constexpr uint64_t CLEAR_EMERGENCY_STOP_EVENT = 0x010000000000FFFCULL;

    /// "Power supply brownout detected below minimum required by standard"
    /// This event can be generated when a node detects that the CAN bus power
    /// has dropped below the minimum declared in the standard.
    static constexpr uint64_t POWER_STANDARD_BROWNOUT_EVENT = 0x010000000000FFF0ULL;

    /// "Power supply brownout detected below minimum required by node"
    /// This event can be generated when a node detects that it has
    /// insufficient power for normal operations.
    static constexpr uint64_t NODE_POWER_BROWNOUT_EVENT = 0x010000000000FFF1ULL;

    /// "Ident button combination pressed"
    /// This event can be generated by a node when it is instructed to generate
    /// an identification event.
    static constexpr uint64_t NODE_IDENT_BUTTON_EVENT = 0x010000000000FE00ULL;

    /** Status of the pysical layer link */
    enum LinkStatus
    {
        LINK_UP,  /**< link is up and ready for transmit */
        LINK_DOWN /**< link is down and unable to transmit */
    };
    
    /** Get the MTI address present value field.
     * @param mti MTI to extract field value from
     * @return true if MTI is an addressed message, else false
     */
    static bool get_mti_address(MTI mti)
    {
        return (mti & MTI_ADDRESS_MASK);
    }

    /** Get the MTI event present value field.
     * @param mti MTI to extract field value from
     * @return true if MTI is a message carrying an event as payload, else false
     */
    static bool get_mti_event(MTI mti)
    {
        return (mti & MTI_EVENT_MASK);
    }

    /** Get the MTI datagram or stream value field.
     * @param mti MTI to extract field value from
     * @return true if MTI is a datagram or stream, else false
     */
    static bool get_mti_datagram(MTI mti)
    {
        return (mti & MTI_DATAGRAM_MASK);
    }

    /** Get the MTI priority (value 0 through 3).
     * @param mti MTI to extract field value from
     * @return priority value 0 through 3
     */
    static unsigned int mti_priority(MTI mti)
    {
        return (mti & MTI_PRIORITY_MASK) >> MTI_PRIORITY_SHIFT;
    }

    /** Maximum size of a static addressed message payload */
    static const size_t MAX_ADDRESSED_SIZE = 14;

private:
    /** This struct should not be instantiated. */
    Defs();
};  // struct Defs

/// Static values and helper functions for CAN-bus interfaces.
struct CanDefs {
    /** Status value for an alias pool item.
     */
    enum AliasStatus
    {
        UNDER_TEST, /**< this is an alias we are trying to claim */
        RESERVED,   /**< the alias has been reserved for use */
        CONFLICT,   /**< we discovered someone else already is using this alias */
        FREE        /**< the alias is free for another request */
    };


    /** CAN ID bit fields for most CAN frames. */
    enum ID
    {
        SRC_MASK            = 0x00000fff, /**< mask for source field of CAN ID */
        MTI_MASK            = 0x00fff000, /**< mask for MTI field of CAN ID */
        DST_MASK            = 0x00fff000, /**< mask for MTI field of CAN ID */
        CAN_FRAME_TYPE_MASK = 0x07000000, /**< mask for can frame type field of CAN ID */
        FRAME_TYPE_MASK     = 0x08000000, /**< mask for frame type field of CAN ID */
        PRIORITY_MASK       = 0x10000000, /**< mask for priority field of CAN ID */
        PADDING_MASK        = 0xe0000000, /**< mask for padding field of CAN ID */
        STREAM_DG_RECV_MASK = 0x0fffffff, /**< mask for receiving datagram and stream frames. */

        SRC_SHIFT            =  0, /**< shift for source field of CAN ID */
        MTI_SHIFT            = 12, /**< shift for MTI field of CAN ID */
        DST_SHIFT            = 12, /**< shift for MTI field of CAN ID */
        CAN_FRAME_TYPE_SHIFT = 24, /**< shift for can frame type field of CAN ID */
        FRAME_TYPE_SHIFT     = 27, /**< shift for frame type field of CAN ID */
        PRIORITY_SHIFT       = 28, /**< shift for priority field of CAN ID */
        PADDING_SHIFT        = 29, /**< shift for padding field of CAN ID */
        
        CONTROL_SRC_MASK      = 0x00000fff, /**< source alias mask */
        CONTROL_FIELD_MASK    = 0x07fff000, /**< control field data mask */
        CONTROL_SEQUENCE_MASK = 0x07000000, /**< frame sequence number mask */
        CONTROL_TYPE_MASK     = 0x08000000, /**< value of '0' means control frame mask */
        CONTROL_PRIORITY_MASK = 0x10000000, /**< priority mask */
        CONTROL_PADDING_MASK  = 0xe0000000, /**< pad out to a full 32-bit word */

        CONTROL_SRC_SHIFT      =  0, /**< source alias shift */
        CONTROL_FIELD_SHIFT    = 12, /**< control field data shift */
        CONTROL_SEQUENCE_SHIFT = 24, /**< frame sequence number shift */
        CONTROL_TYPE_SHIFT     = 27, /**< value of '0' means control frame shift */
        CONTROL_PRIORITY_SHIFT = 28, /**< priority shift */
        CONTROL_PADDING_SHIFT  = 29  /**< pad out to a full 32-bit word */
    };

    // @TODO(balazs.racz) do we need this?
    typedef uint16_t CanMTI;

    /** CAN Frame Types. */
    enum CanFrameType
    {
        GLOBAL_ADDRESSED      = 1, /**< most CAN frame types fall in this category */
        DATAGRAM_ONE_FRAME    = 2, /**< a single frame datagram */
        DATAGRAM_FIRST_FRAME  = 3, /**< first frame of multi-frame datagram */
        DATAGRAM_MIDDLE_FRAME = 4, /**< middle frame of multi-frame datagram */
        DATAGRAM_FINAL_FRAME  = 5, /**< last frame of multi-frame datagram */
        STREAM_DATA           = 7, /**< stream data frame */
    };
    
    /** Frame Types, Control or normal NMRAnet message. */
    enum FrameType
    {
        CONTROL_MSG = 0, /**< CAN control frame message */
        NMRANET_MSG = 1  /**< normal NMRAnet message */
    };
    
    /** Highest order priority of a CAN message.  Most messages fall into
     * the NORMAL_PRIORITY category.
     */
    enum Priority
    {
        HIGH_PRIORITY   = 0, /**< high priority CAN message */
        NORMAL_PRIORITY = 1  /**< normal priority CAN message */
    };

    enum ControlField
    {
        RID_FRAME = 0x0700, /**< Reserve ID Frame */
        AMD_FRAME = 0x0701, /**< Alias Map Definition frame */
        AME_FRAME = 0x0702, /**< Alias Mapping Enquiry */
        AMR_FRAME = 0x0703  /**< Alias Map Reset */
    };

    enum AddressedPayloadFlags
    {
        NOT_FIRST_FRAME = 0x20,
        NOT_LAST_FRAME = 0x10,
    };

    /// Constants used in the LocalAliasCache for reserved but not used
    /// aliases.
    enum ReservedAliasNodeId
    {
        /// To mark a reserved alias in the local alias cache, we use this as a
        /// node ID and add the alias to the lowest 12 bits. Since this value
        /// starts with a zero MSB byte, it is not a valid node ID.
        RESERVED_ALIAS_NODE_BITS = 0xF000,
        /// Mask for the reserved aliases.
        RESERVED_ALIAS_NODE_MASK = 0xFFFFFFFFF000
    };

    /** Get the source field value of the CAN ID.
     * @param can_id identifier to act upon
     * @return source field
     */
    static NodeAlias get_src(uint32_t can_id)
    {
        return (can_id & SRC_MASK) >> SRC_SHIFT;
    }

    /** Get the MTI field value of the CAN ID.
     * @param can_id identifier to act upon
     * @return MTI field value
     */
    static CanMTI get_mti(uint32_t can_id)
    {
        return ((can_id & MTI_MASK) >> MTI_SHIFT);
    }

    /** Get the destination field value of the CAN ID.
     * @param can_id identifier to act upon
     * @return destination field value
     */
    static NodeAlias get_dst(uint32_t can_id)
    {
        return (can_id & DST_MASK) >> DST_SHIFT;
    }

    /** Get the CAN frame type field value of the CAN ID.
     * @param can_id identifier to act upon
     * @return CAN frame type field value
     */
    static CanFrameType get_can_frame_type(uint32_t can_id)
    {
        return (CanFrameType)((can_id & CAN_FRAME_TYPE_MASK) >> CAN_FRAME_TYPE_SHIFT);
    }

    /** Get the frame type field value of the CAN ID.
     * @param can_id identifier to act upon
     * @return frame type field value
     */
    static FrameType get_frame_type(uint32_t can_id)
    {
        return (FrameType)((can_id & FRAME_TYPE_MASK) >> FRAME_TYPE_SHIFT);
    }

    /** Get the priority field value of the CAN ID.
     * @param can_id identifier to act upon
     * @return priority field value
     */
    static Priority get_priority(uint32_t can_id)
    {
        return (Priority)((can_id & PRIORITY_MASK) >> PRIORITY_SHIFT);
    }

    /** Tests if the incoming frame is a CID frame.
     * @param can_id identifier to act upon
     * @return true for CID frame, false for any other frame.
     */
    static bool is_cid_frame(uint32_t can_id)
    {
        return ((can_id >> CAN_FRAME_TYPE_SHIFT) & 0x1C) == 0x14;
    }

    /** Tests if the incoming frame is a stream data send frame.
     * @param can_id identifier to act upon
     * @return true for Stream Data frame, false for any other frame.
     */
    static bool is_stream_frame(uint32_t can_id)
    {
        return ((can_id >> CAN_FRAME_TYPE_SHIFT) & 0xF) == 0xF;
    }

    /** Set the MTI field value of the CAN ID.
     * @param can_id identifier to act upon, passed by reference
     * @param mti MTI field value
     */
    static void set_mti(uint32_t *can_id, CanMTI mti)
    {
        *can_id &= ~MTI_MASK;
        *can_id |= mti << MTI_SHIFT;
    }

    /** Set the source field value of the CAN ID.
     * @param can_id identifier to act upon, passed by pointer
     * @param src source field value
     */
    static void set_src(uint32_t *can_id, NodeAlias src)
    {
        *can_id &= ~SRC_MASK;
        *can_id |= src << SRC_SHIFT;
    }

    /** Set the destination field value of the CAN ID.
     * @param can_id identifier to act upon, passed by reference
     * @param dst destination field value
     */
    static void set_dst(uint32_t *can_id, NodeAlias dst)
    {
        *can_id &= ~DST_MASK;
        *can_id |= dst << DST_SHIFT;
    }

    /** Set the CAN frame type field value of the CAN ID.
     * @param can_id identifier to act upon, passed by reference
     * @param type CAN frame type field value
     */
    static void set_can_frame_type(uint32_t *can_id, CanFrameType type)
    {
        *can_id &= ~CAN_FRAME_TYPE_MASK;
        *can_id |= type << CAN_FRAME_TYPE_SHIFT;
    }

    /** Set the frame type field value of the CAN ID.
     * @param can_id identifier to act upon, passed by reference
     * @param type frame type field value
     */
    static void set_frame_type(uint32_t *can_id, FrameType type)
    {
        *can_id &= ~FRAME_TYPE_MASK;
        *can_id |= type << FRAME_TYPE_SHIFT;
    }

    /** Set the priority field value of the CAN ID.
     * @param can_id identifier to act upon, passed by reference
     * @param priority pryority field value
     */
    static void set_priority(uint32_t *can_id, Priority priority)
    {
        *can_id &= ~PRIORITY_MASK;
        *can_id |= priority << PRIORITY_SHIFT;
    }

    /** Set all the CAN ID fields.
     * @param can_id identifier to act upon
     * @param source source field value
     * @param mti MTI field value
     * @param can_type CAN frame type field value
     * @param type frame type field value
     * @param priority priority field value
     */
    static void set_fields(uint32_t *can_id, NodeAlias src, Defs::MTI mti, CanFrameType can_type, FrameType type, Priority priority)
    {
        *can_id = (src      << SRC_SHIFT           ) +
                  (mti      << MTI_SHIFT           ) +
                  (can_type << CAN_FRAME_TYPE_SHIFT) +
                  (type     << FRAME_TYPE_SHIFT    ) +
                  (priority << PRIORITY_SHIFT      );
    }

    /** Set all the CAN ID fields for datagram or stream message.
     * @param can_id identifier to act upon
     * @param source source alias
     * @param dst desitnation alias
     * @param can_type CAN frame type field value
     */
    static void set_datagram_fields(uint32_t *can_id, NodeAlias src,
                                    NodeAlias dst, CanFrameType can_type)
    {
        *can_id = (src      << SRC_SHIFT           ) +
                  (dst      << DST_SHIFT           ) +
                  (can_type << CAN_FRAME_TYPE_SHIFT) +
                  (NMRANET_MSG << FRAME_TYPE_SHIFT ) +
                  (NORMAL_PRIORITY << PRIORITY_SHIFT);
    }

    /** Get the NMRAnet MTI from a can identifier.
     * @param can_id CAN identifider
     * @return NMRAnet MTI
     */
    static Defs::MTI nmranet_mti(uint32_t can_id);

    /** Get the CAN identifier from an NMRAnet mti and source alias.
     * @param mti NMRAnet MTI
     * @param src Source node alias
     * @return CAN identifier
     */
    static uint32_t can_identifier(Defs::MTI mti, NodeAlias src);

    /** Get the control field of a can control frame. This includes the
     * sequence number and the variable field.

     * @param can_id CAN ID of the control frame
     * @return value of the control field
     */
    static ControlField get_control_field(uint32_t can_id)
    {
        return (ControlField)((can_id & CONTROL_FIELD_MASK) >> CONTROL_FIELD_SHIFT);
    }

#if 0
    /** Get the source field of the a can control frame.
     * @param can_id CAN ID of the control frame
     * @return value of the source field
     */
    static NodeAlias get_control_src(uint32_t can_id)
    {
        return (can_id & CONTROL_SRC_MASK) >> CONTROL_SRC_SHIFT;
    }

    /** Get the sequence field of the a can control frame.
     * @param can_id CAN ID of the control frame
     * @return value of the sequence field
     */
    static unsigned int get_control_sequence(uint32_t can_id)
    {
        return (can_id & CONTROL_SEQUENCE_MASK) >> CONTROL_SEQUENCE_SHIFT;
    }

#endif

    /** Initialize a control frame CAN ID and set DLC to 0.
     * @param src source node alias
     * @param field control field data (e.g. AME_FRAME)
     * @param sequence sequence number or zero if not CID frame
     */
    static uint32_t set_control_fields(
        NodeAlias src, uint16_t field, int sequence)
    {
        return (src << CONTROL_SRC_SHIFT) | (field << CONTROL_FIELD_SHIFT) |
            (sequence << CONTROL_SEQUENCE_SHIFT) | ((0) << CONTROL_TYPE_SHIFT) |
            ((1) << CONTROL_PRIORITY_SHIFT);
    }

    /** Initialize a control frame CAN ID and set DLC to 0.
     * @param _frame control frame to initialize
     * @param _source source data
     * @param _field field data
     * @param _sequence sequence data
     */
    static void control_init(struct can_frame &frame, NodeAlias src, uint16_t field, int sequence)
    {
        SET_CAN_FRAME_ID_EFF(frame, set_control_fields(src, field, sequence));
        frame.can_dlc = 0;
    }

    /** Computes a reserved alias node ID for the local alias cache map.
     * @param alias the alias to reserve
     * @return Node ID to use in the alias map as a key.
     */
    static NodeID get_reserved_alias_node_id(NodeAlias alias)
    {
        return RESERVED_ALIAS_NODE_BITS | alias;
    }

    /** Tests if a node ID is a reserved alias Node ID.
     * @param id node id to test
     * @return true if this is a reserved alias node ID. */
    static bool is_reserved_alias_node_id(NodeID id)
    {
        return (id & RESERVED_ALIAS_NODE_MASK) == RESERVED_ALIAS_NODE_BITS;
    }

private:
    /** This class should not be instantiated. */
    CanDefs();
};  // struct CanDefs

/** Static constants and functions related to the Datagram protocol. */
struct DatagramDefs
{
    /** All known datagram protocols */
    enum Protocol
    {
        LOG_REQUEST   = 0x01, /**< request a placement into the log */
        LOG_REPLY     = 0x02, /**< reply to a @ref LOG_REQUEST */
        CONFIGURATION = 0x20, /**< configuration message */
        REMOTE_BUTTON = 0x21, /**< remote button input */
        DISPLAY_PROTOCOL = 0x28, /**< place on display */
        TRAIN_CONTROL = 0x30, /**< operation of mobile nodes */
    };

    /** Various public datagram constants. */
    enum
    {
        MAX_SIZE = 72, /**< maximum size in bytes of a datagram */

    };
    
    /** Possible flags for a successful receipt (received okay) of a Datagram.
     */
    enum Flag
    {
        FLAGS_NONE    = 0x00, /**< no flags set */
        REPLY_PENDING = 0x80, /**< A reply is pending */
        
        TIMEOUT_NONE  = 0x00, /**< no timeout */
        TIMEOUT_2     = 0x01, /**< 2 second timeout */
        TIMEOUT_4     = 0x02, /**< 4 second timeout */
        TIMEOUT_8     = 0x03, /**< 8 second timeout */
        TIMEOUT_16    = 0x04, /**< 16 second timeout */
        TIMEOUT_32    = 0x05, /**< 32 second timeout */
        TIMEOUT_64    = 0x06, /**< 64 second timeout */
        TIMEOUT_128   = 0x07, /**< 128 second timeout */
        TIMEOUT_256   = 0x08, /**< 256 second timeout */
        TIMEOUT_512   = 0x09, /**< 512 second timeout */
        TIMEOUT_1024  = 0x0A, /**< 1024 second timeout */
        TIMEOUT_2048  = 0x0B, /**< 2048 second timeout */
        TIMEOUT_4096  = 0x0C, /**< 4096 second timeout */
        TIMEOUT_8192  = 0x0D, /**< 8192 second timeout */
        TIMEOUT_16384 = 0x0E, /**< 16384 second timeout */
        TIMEOUT_32768 = 0x0F, /**< 32768 second timeout */
        
        TIMEOUT_MASK  = 0x0F, /**< Mask for reply timeout */
    };
    
    /** Possible error codes for a rejected datagram.
     */
    enum Error
    {
        RESEND_OK          = 0x2000, /**< We can try to resend the datagram. */
        TRANSPORT_ERROR    = 0x6000, /**< Transport error occurred. */
        BUFFER_UNAVAILABLE = 0x2020, /**< Buffer unavailable error occurred. */
        OUT_OF_ORDER       = 0x2040, /**< Out of order error occurred. */
        PERMANENT_ERROR    = 0x1000, /**< Permanent error occurred. */
        SRC_NOT_PERMITTED  = 0x1020, /**< Source not permitted error occurred. */
        NOT_ACCEPTED       = 0x1040, /**< Destination node does not accept datagrams of any kind. */
        UNIMPLEMENTED      = 0x1080, /**< NON_STANDARD The feature or command requested is not implemented by the target node. */
        INVALID_ARGUMENTS  = 0x1010, /**< NON_STANDARD Invalid or unparseable arguments. */
    };

    /** We can try to resend the datagram.
     * @param error error number
     * @return true or false
     */
    static bool resend_ok(uint16_t error)
    {
        return error & RESEND_OK;
    }

    /** Transport error occurred.
     * @param error error number
     * @return true or false
     */
    static bool transport_error(uint16_t error)
    {
        return error & TRANSPORT_ERROR;
    }

    /** Buffer unavailable error occurred.
     * @param error error number
     * @return true or false
     */
    static bool buffer_unavailable(uint16_t error)
    {
        return error & BUFFER_UNAVAILABLE;
    }
    
    /** Determine if the protocol ID is represented by one, two, or six bytes.
     * @param _protocol protocol ID to interrogate
     * @return number of bytes representing the protocol
     */
    static unsigned int protocol_size(uint64_t protocol)
    {
        return (((protocol & PROTOCOL_SIZE_MASK) == PROTOCOL_SIZE_6) ? 6 :
                ((protocol & PROTOCOL_SIZE_MASK) == PROTOCOL_SIZE_2) ? 2 : 1);
    }

private:
    /** Constants used by the protocol_size function. */
    enum
    {
        PROTOCOL_SIZE_2    = 0xE0, /**< possible return value for @ref protocol_size */
        PROTOCOL_SIZE_6    = 0xF0, /**< possible return value for @ref protocol_size */
        PROTOCOL_SIZE_MASK = 0xF0, /**< mask used when determining protocol size */
    };

    /** Do not instantiate this class, ever. */
    DatagramDefs();
}; // struct DatagramDefs

/** Returns the inverted event state, switching valid and invalid, but not
 * changing unknown and reserved. */
inline EventState invert_event_state(EventState state)
{
    switch (state)
    {
        case EventState::VALID:
            return EventState::INVALID;
        case EventState::INVALID:
            return EventState::VALID;
        default:
            return state;
    }
}

/** Returns the inverted event state, switching valid and invalid, but not
 * changing unknown and reserved. */
inline EventState to_event_state(bool state)
{
    return state ? EventState::VALID : EventState::INVALID;
}

/** Allows of setting the producer/consumer identified MTI with the event state
 * to set the low bits. */
inline Defs::MTI operator+(const Defs::MTI &value, EventState state)
{
    int code = static_cast<int>(value);
    code += static_cast<int>(state);
    return static_cast<Defs::MTI>(code);
}

/** Operator overload for post increment */
inline Defs::MTI operator ++ (Defs::MTI &value, int)
{
    Defs::MTI result = value;
    value = static_cast<Defs::MTI>(static_cast<int>(value) + 1);
    return result;
}

/** Operator overload for pre increment */
inline Defs::MTI& operator ++ (Defs::MTI &value)
{
    value = static_cast<Defs::MTI>(static_cast<int>(value) + 1);
    return value;
}

/** Operator overload for post decrement */
inline Defs::MTI operator -- (Defs::MTI &value, int)
{
    Defs::MTI result = value;
    value = static_cast<Defs::MTI>(static_cast<int>(value) - 1);
    return result;
}

/** Operator overload for pre decrement */
inline Defs::MTI& operator -- (Defs::MTI &value)
{
    value = static_cast<Defs::MTI>(static_cast<int>(value) - 1); 
    return value;
}

/// State machine states for initializing the bootloader node. Represents what
/// the next outgoing packet should be.
enum InitState
{
    PICK_ALIAS = 0,
    SEND_CID_7,
    SEND_CID_6,
    SEND_CID_5,
    SEND_CID_4,
    WAIT_RID,
    SEND_RID,
    SEND_AMD,
    SEND_NMRANET_INIT,
    INITIALIZED,
};

/// Internal state of the bootloader stack.
struct BootloaderState
{
    struct can_frame input_frame;
    struct can_frame output_frame;
    unsigned input_frame_full : 1;
    unsigned output_frame_full : 1;
    unsigned request_reset : 1;
#ifdef BOOTLOADER_STREAM
    unsigned stream_pending : 1;
    unsigned stream_open : 1;
    unsigned stream_proceed_pending : 1;
#endif
#ifdef BOOTLOADER_DATAGRAM
    unsigned incoming_datagram_pending : 1;
    unsigned datagram_write_pending : 1;
#endif
    // 1 if the datagram buffer is busy
    unsigned datagram_output_pending : 1;
    // 1 if we are waiting for an incoming reply to a sent datagram
    unsigned datagram_reply_waiting : 1;

    NodeAlias alias;
    InitState init_state;

    // response datagram
    NodeAlias datagram_dst;
    uint8_t datagram_dlc;
    uint8_t datagram_offset;
    uint8_t datagram_payload[14];

    // Node that is sending us the stream of data.
    NodeAlias write_src_alias;

#ifdef BOOTLOADER_STREAM
    // stream source ID of incoming data.
    uint8_t stream_src_id;
    // What's the total length of the negotiated stream buffer.
    uint16_t stream_buffer_size;
    // How many bytes are left of the strem buffer before a continue needs to
    // be sent.
    int stream_buffer_remaining;
#endif

    // Offset of the beginning of the write buffer.
    uintptr_t write_buffer_offset;
    // Offset inside the write buffer for the next incoming data.
    unsigned write_buffer_index;
};

/// Global state variables.
extern BootloaderState state_;
BootloaderState state_;

/// Protocol support bitmask that the node should export.
#define PIP_REPLY_VALUE                                                        \
    (Defs::SIMPLE_PROTOCOL_SUBSET | Defs::EVENT_EXCHANGE)

/// We manually convert to big-endian to store this value in .rodata.
static const uint64_t PIP_REPLY =         //
    ((PIP_REPLY_VALUE >> 40) & 0xff) |    //
    ((PIP_REPLY_VALUE >> 24) & 0xff00) |  //
    ((PIP_REPLY_VALUE >> 8) & 0xff0000) | //
    ((PIP_REPLY_VALUE & 0xff0000) << 8) | //
    ((PIP_REPLY_VALUE & 0xff00) << 24) |  //
    ((PIP_REPLY_VALUE & 0xff) << 40);


/// Prepares the outgoing frame buffer for a frame to be sent.
void setup_can_frame()
{
    CLR_CAN_FRAME_RTR(state_.output_frame);
    CLR_CAN_FRAME_ERR(state_.output_frame);
    SET_CAN_FRAME_EFF(state_.output_frame);
    state_.output_frame.can_dlc = 0;
    state_.output_frame_full = 1;
}

/// Sets up the outgoing frame buffer for a global OpenLCB packet.
///
/// @param mti message transmit indicator to set for the outgoing message.
///
void set_can_frame_global(Defs::MTI mti)
{
    setup_can_frame();
    uint32_t id;
    CanDefs::set_fields(&id, state_.alias, mti, CanDefs::GLOBAL_ADDRESSED,
        CanDefs::NMRANET_MSG, CanDefs::NORMAL_PRIORITY);
    SET_CAN_FRAME_ID_EFF(state_.output_frame, id);
}

/** Sets the outgoing CAN frame to addressed, destination taken from the source
 * field of the incoming message or the given alias. 
 * @param mti the MTI of the outgoing value.
 * @param alias the destination node alias; defaults to taking the alias of the
 * incoming frame.
*/
void set_can_frame_addressed(
    Defs::MTI mti, NodeAlias alias = CanDefs::get_src(
                       GET_CAN_FRAME_ID_EFF(state_.input_frame)))
{
    setup_can_frame();
    uint32_t id;
    CanDefs::set_fields(&id, state_.alias, mti, CanDefs::GLOBAL_ADDRESSED,
        CanDefs::NMRANET_MSG, CanDefs::NORMAL_PRIORITY);
    SET_CAN_FRAME_ID_EFF(state_.output_frame, id);
    state_.output_frame.can_dlc = 2;
    state_.output_frame.data[0] = (alias >> 8) & 0xf;
    state_.output_frame.data[1] = alias & 0xff;
}

/// Adds the node id ad the data payload of the outgoing can frame.
void set_can_frame_nodeid()
{
    uint64_t node_id = nmranet_nodeid();
    for (int i = 5; i >= 0; --i)
    {
        state_.output_frame.data[i] = node_id & 0xff;
        node_id >>= 8;
    }
    state_.output_frame.can_dlc = 6;
}

/// Checks whether the incoming frame contains the current (bootloader) node's
/// node_id as the data payload.
///
///
/// @return true if the current node's address is in the payload.
///
bool is_can_frame_nodeid()
{
    if (state_.input_frame.can_dlc != 6)
        return false;
    uint64_t node_id = nmranet_nodeid();
    for (int i = 5; i >= 0; --i)
    {
        if (state_.input_frame.data[i] != (node_id & 0xff))
            return false;
        node_id >>= 8;
    }
    return true;
}

/** Sets output frame dlc to 4; adds the given error code to bytes 2 and 3.
 * @param error_code send this in bytes 2 and 3 of the reply message */
void set_error_code(uint16_t error_code)
{
    state_.output_frame.can_dlc = 4;
    state_.output_frame.data[2] = error_code >> 8;
    state_.output_frame.data[3] = error_code & 0xff;
}

/// Kills the input frame and sends back a datagram rejected error message with
/// permanent error.
void reject_datagram()
{
    set_can_frame_addressed(Defs::MTI_DATAGRAM_REJECTED);
    set_error_code(Defs::ERROR_PERMANENT);
    state_.input_frame_full = 0;
}

/** Loads an unaligned 32-bit value that is network-endian. 
    @param ptr is an unaligned pointer to ram.
    @return the big-endian interpreted value at the pointed value converted to
    host endian.
 */
uint32_t load_uint32_be(const uint8_t *ptr)
{
    return (ptr[0] << 24) | (ptr[1] << 16) | (ptr[2] << 8) | ptr[3];
}

/// turns an already prepared memory config response datagram into an error
/// response.
///
/// @param error_code 16-bit OpenLCB error code to report in the response
/// datagram.
///
void add_memory_config_error_response(uint16_t error_code)
{
    state_.datagram_payload[1] |= 0x08; // Turns success into error reply.
    state_.datagram_payload[state_.datagram_dlc++] = error_code >> 8;
    state_.datagram_payload[state_.datagram_dlc++] = error_code & 0xff;
}

/// Handles incoming addressed message (non-datagram).
void handle_addressed_message(Defs::MTI mti)
{
    switch (mti)
    {
        case Defs::MTI_PROTOCOL_SUPPORT_INQUIRY:
        {
            if (state_.output_frame_full)
            {
                // No buffer. Re-try next round.
                return;
            }
            set_can_frame_addressed(Defs::MTI_PROTOCOL_SUPPORT_REPLY);
            state_.output_frame.can_dlc = 8;
            memcpy(state_.output_frame.data + 2, &PIP_REPLY, 6);
            break;
        }
        case Defs::MTI_VERIFY_NODE_ID_ADDRESSED:
        {
            if (state_.output_frame_full)
            {
                // No buffer. Re-try next round.
                return;
            }
            set_can_frame_global(Defs::MTI_VERIFIED_NODE_ID_NUMBER);
            set_can_frame_nodeid();
            state_.input_frame_full = 0;
            return;
        }
        case Defs::MTI_DATAGRAM_OK:
        {
            if (state_.datagram_reply_waiting)
            {
                state_.datagram_reply_waiting = 0;
                state_.datagram_output_pending = 0;
            }
            break;
        }
#ifdef BOOTLOADER_STREAM
        case Defs::MTI_STREAM_INITIATE_REQUEST:
        {
            if (state_.output_frame_full)
            {
                // No buffer. Re-try next round.
                return;
            }
            if (state_.input_frame.can_dlc < 7)
            {
                set_can_frame_addressed(Defs::MTI_TERMINATE_DUE_TO_ERROR);
                set_error_code(DatagramDefs::INVALID_ARGUMENTS);
                break;
            }
            set_can_frame_addressed(Defs::MTI_STREAM_INITIATE_REPLY);
            state_.output_frame.can_dlc = 8;
            state_.output_frame.data[6] = state_.input_frame.data[6];
            state_.output_frame.data[7] = STREAM_ID;
            if (!state_.stream_pending ||
                state_.input_frame.data[6] != state_.stream_src_id)
            {
                // Request out of the blue. Reject.
                state_.output_frame.data[2] = 0;
                state_.output_frame.data[3] = 0;
                state_.output_frame.data[4] =
                    0b01000010; // permanent error, "should not happen"
                // invalid stream request
                state_.output_frame.data[5] = 0b00100000;
            }
            else
            {
                uint16_t proposed_size = (state_.input_frame.data[2] << 8) |
                    state_.input_frame.data[3];
                uint16_t final_size = WRITE_BUFFER_SIZE;
                while (final_size > proposed_size)
                {
                    final_size >>= 1;
                }
                state_.stream_buffer_size = final_size;
                state_.stream_buffer_remaining = state_.stream_buffer_size;
                state_.output_frame.data[2] = state_.stream_buffer_size >> 8;
                state_.output_frame.data[3] = state_.stream_buffer_size & 0xff;
                state_.output_frame.data[4] = 0x80; // accept, no type id.
                state_.output_frame.data[5] = 0x00;

                state_.stream_pending = 0;
                state_.stream_open = 1;
            }
            break;
        }
        case Defs::MTI_STREAM_COMPLETE:
        {
            return handle_stream_complete();
        }
#endif
        default:
        {
            // Send reject.
            if (state_.output_frame_full)
            {
                // No buffer. Re-try next round.
                return;
            }
            set_can_frame_addressed(Defs::MTI_OPTIONAL_INTERACTION_REJECTED);
            set_error_code(DatagramDefs::UNIMPLEMENTED);
            break;
        }
    }
    state_.input_frame_full = 0;
    return;
}

/// Handles incoming global message.
void handle_global_message(Defs::MTI mti)
{
    switch (mti)
    {
        case Defs::MTI_VERIFY_NODE_ID_GLOBAL:
        {
            if (state_.output_frame_full)
            {
                // No buffer. Re-try next round.
                return;
            }
            set_can_frame_global(Defs::MTI_VERIFIED_NODE_ID_NUMBER);
            set_can_frame_nodeid();
            state_.input_frame_full = 0;
            return;
        }
    default:
        break;
    }
    // Drop to the floor.
    state_.input_frame_full = 0;
    /// @todo(balazs.racz) what about global identify messages?
    return;
}



}  // namespace openlcb


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
