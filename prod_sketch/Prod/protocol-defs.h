#ifndef _GOL_PROTOCOL_DEFS_H_
#define _GOL_PROTOCOL_DEFS_H_

enum Direction : uint8_t {
  kNorth = 0,
  kEast = 1,
  kSouth = 2,
  kWest = 3,
  kNorthWest = 4,
  kNorthEast = 5,
  kSouthEast = 6,
  kSouthWest = 7
};

static constexpr int kMaxDirection = (int)kWest;

struct ProtocolDefs {
public:
  // How frequently we should emit evolution ticks.
  static constexpr unsigned kDefaultEvolutionSpeedMsec = 250;

  static constexpr uint64_t kEventPrefix = UINT64_C(0x09000D0000000000);
  static constexpr uint64_t kEventMask   = UINT64_C(0xFFFFFF0000000000);
  // Shift a byte this many bits to get the command byte.
  static constexpr unsigned kCmdShift = 32;
  static constexpr uint64_t kCmdMask = UINT64_C(0xFF) << kCmdShift;

  static constexpr unsigned kXShift = 24;
  static constexpr uint64_t kXMask = UINT64_C(0xFF) << kXShift;
  static constexpr unsigned kYShift = 16;
  static constexpr uint64_t kYMask = UINT64_C(0xFF) << kYShift;
  static constexpr unsigned kArgXShift = 8;
  static constexpr uint64_t kArgXMask = UINT64_C(0xFF) << kArgXShift;
  static constexpr unsigned kArgYShift = 0;
  static constexpr uint64_t kArgYMask = UINT64_C(0xFF) << kArgYShift;
  static constexpr unsigned kArgShift = 0;
  static constexpr uint64_t kArgMask = UINT64_C(0xFFFF) << kArgYShift;


  enum Command {
    // Global command without any arguments. no x,y, arg is the command.
    kGlobalCmd = 0xFF,
    // Reports the current state of a node. x,y defines the node. arg is 16-bit
    // for the node's state. Sent by each node.
    kStateReport = 0xF0,
    // Overrides the state of a node. x,y defines the node. arg is 16-bit for
    // the node's state. Sent by an external configuring agent.
    kStateSet = 0xF4,
    // Modifies the state of a node by adding certain cells (with an OR
    // operator). x,y defines the node. arg is 16-bit for the bits to set in
    // the node's state. Sent by an external configuring agent.
    kStateOr = 0xF5,
    // Listen for a local signal and set the coordinates. x,y are the new
    // coordinates. arg unused. Sent by the leader.
    kLocalAssign = 0xF2,
    // Reports that the local signal was received, and coordinates are now
    // assigned. x,y are the new coordinates. Arg unused. Sent by the
    // node. This might be not matching the coordinates in kLocalAssign if
    // there is a coordinate conflict.
    kLocalFound = 0xF3,
    // Reports that there are conflicting coordinates from a local assign. x,y
    // are the old coordinates (which stay in effect). ArgX, ArgY are the new
    // coordinates (that were not assigned). Sent by the node.
    kLocalConflict = 0xF6,
    // Reports who the leader is (according to the sender as an observer). x,y
    // are unused, arg is the alias of the leader.
    kDeclareLeader = 0xF7,
    // Found an unannounced local signal. x,y are the source node that found
    // this, arg unused.
    kLocalSpurious = 0xF8,
    // Firmware upgrade data. There are four bytes of firmware payload in this
    // message (in x, y and args fields).
    kFirmwareData = 0xF9,
    // Reports that a button was pressed. Button number is arg, x, y are the
    // location of the cell. Button 0..15 are leds, 16 is the menu button.
    kButtonPressed = 0xFA,
    // Reports the current addresses. x, y are the local address. Arg is the
    // node ID's last 16 bits.
    kCurrentAddressReport = 0xFB,
    // Requests a local signal to be toggled. dir is the lowest two bits of the
    // command. x,y are the target coordinate. Arg unused. Sent by the leader.
    kToggleLocalSignal = 0xEC,
    // Reports who is our neighbor. x,y, are the source (reporting)
    // node. argx-argy are the neighbor node. dir of the reporting node is the
    // lowest two bits of the command. The neighbor's direction is the bits 2-3
    // of the command. Sent by the node.
    kReportNeighbor = 0xD0,
    // Sets who is our neighbor. x,y, are the target node (to
    // change). argx-argy are the neighbor node. dir of the target node is the
    // lowest two bits of the command. The neighbor's direction is the bits 2-3
    // of the command. Sent by the leader.
    kSetNeighbor = 0xC0,

    // These commands have no argument in the cmd byte.
    kNoArgFilter = 0xF0,
    kNoArgMask = 0xF0,
    // These commands have 2 bits of argument in the cmd byte.
    k2bArgFilter = 0xE0,
    k2bArgMask = 0xF0,
    // These commands have 4 bits of argument in the cmd byte.
    k4bArgFilter = 0xC0,
    k4bArgMask = 0xE0,
  };

  // These are the arguments of the kGlobalCmd events.
  enum GlobalCommand {
    // Drop all state and reinitialize all connections.
    kReInit = 1,
    // Hard-reboot all nodes.
    kReboot = 2,
    // Report all neighbors.
    kReportNeighbors = 3,
    // Reports a freshly initialized node.
    kInitDone = 4,
    // Reports that the sender is the leader.
    kIAmLeader = 5,
    // Start leader election; I can be leader.
    kProposeLeader = 6,
    // Triggers all unassigned nodes to flip all their local lines.
    kLocalToggleUnassigned = 8,
    // When this command comes, each node should evolve the current state and report the state.
    kEvolveAndReport = 9,
    // Requests all devices to report the current state.
    kReportState = 10,
    // Stop iteration (acted upon by the leader only)
    kStopIteration = 11,
    // Stop iteration (acted upon by the leader only)
    kStartIteration = 12,
    // Set state to a random bitset.
    kSetStateRandom = 13,
    // Set the state to all off.
    kClearState = 19,
    // Enter firmware upgrade mode.
    kEnterFirmware = 14,
    // Finsh a firmware upgrade.
    kStartFirmware = 15,
    // Finsh a firmware upgrade.
    kFinishFirmware = 16,
    // Requests to report calibration values for touch sensor
    kReportCalibration = 17,
    // Data from touch sensor calibration in X/Y arguments.
    kCalibrationData = 18,
    // Reports x-y and address low byte.
    kReportAddress = 20,
    // Reports x-y and address low byte.
    kPrintNeighborDebug = 21,
    // Leader should reply with IAmLeader.
    kFindLeader = 22,

    // next = 23.
  };

  // Checks if an event belong to this protocol.
  static constexpr bool IsProtocolEvent(uint64_t ev) {
    return (ev & kEventMask) == kEventPrefix;
  }

  // Extract the command from an event.
  static Command GetCommand(uint64_t ev) {
    uint8_t raw_value = (ev & kCmdMask) >> kCmdShift;
    if ((raw_value & kNoArgMask) == kNoArgFilter) {
      return (Command)raw_value;
    } else if ((raw_value & k2bArgMask) == k2bArgFilter) {
      return (Command)(raw_value & ~3);
    } else if ((raw_value & k4bArgMask) == k4bArgFilter) {
      return (Command)(raw_value & ~0xf);
    }
    return (Command)0;
  }

  // @return the first direction field in the event encoding (bottom two bits of command byte)
  static Direction GetDir(uint64_t ev) {
    uint8_t raw_value = (ev & kCmdMask) >> kCmdShift;
    return (Direction)(raw_value & 3);
  }
  // @return the second direction field in the event encoding (second two bits of the command byte).
  static Direction GetDir2(uint64_t ev) {
    uint8_t raw_value = (ev & kCmdMask) >> (kCmdShift + 2);
    return (Direction)(raw_value & 3);
  }
  // Extracts the X parameter from an event.
  static uint8_t GetX(uint64_t ev) {
    return (ev & kXMask) >> kXShift;
  }
  // Extracts the Y parameter from an event.
  static uint8_t GetY(uint64_t ev) {
    return (ev & kYMask) >> kYShift;
  }
  // Extracts the Arg parameter from an event.
  static uint16_t GetArg(uint64_t ev) {
    return (ev & kArgMask) >> kArgShift;
  }
  // Extracts the ArgX parameter from an event.
  static uint8_t GetArgX(uint64_t ev) {
    return (ev & kArgXMask) >> kArgXShift;
  }
  // Extracts the ArgY parameter from an event.
  static uint8_t GetArgY(uint64_t ev) {
    return (ev & kArgYMask) >> kArgYShift;
  }

  // Helper functions for creating encoded events.

  static constexpr uint64_t CreateEvent(Command cmd, uint8_t x, uint8_t y, uint16_t arg = 0) {
    return kEventPrefix | (uint64_t(cmd) << kCmdShift) |          //
           (uint64_t(x) << kXShift) | (uint64_t(y) << kYShift) |  //
           (uint64_t(arg) << kArgShift);
  }

  static constexpr uint64_t CreateEvent(Command cmd, uint8_t x, uint8_t y, uint8_t argx, uint8_t argy) {
    return kEventPrefix | (uint64_t(cmd) << kCmdShift) |          //
           (uint64_t(x) << kXShift) | (uint64_t(y) << kYShift) |  //
           (uint64_t(argx) << kArgXShift) | (uint64_t(argy) << kArgYShift);
  }

  static constexpr uint64_t CreateEvent(Command cmd, uint8_t x, uint8_t y, uint8_t argx, uint8_t argy, Direction d1) {
    return CreateEvent(cmd, x, y, argx, argy) |  //
           (uint64_t(d1) << kCmdShift);
  }

  static constexpr uint64_t CreateEvent(Command cmd, uint8_t x, uint8_t y, uint8_t argx, uint8_t argy, Direction d1, Direction d2) {
    return CreateEvent(cmd, x, y, argx, argy) |  //
           (uint64_t(d1) << kCmdShift) | (uint64_t(d2) << (kCmdShift + 2));
  }

  static constexpr uint64_t CreateGlobalCmd(GlobalCommand arg) {
    return CreateEvent(kGlobalCmd, 0, 0, (uint16_t)arg);
  }


  // A segment is a sequence of 4 bits in the 16-bit argument describing the
  // current state of a node. Rows can be segments, but also edges.
  struct Segment {
    // Bit number of the first bit in the segment. This means (1u << bit_num)
    // is the bit that needs to be set in the uint16_t arg in order to indicate
    // that the first bit in the segment is lit.
    int8_t bit_num;
    // The remaining bits are at bit_num + bit_stride, bit_num + 2 * bit_stride
    // and bit_num + 3 * bit_stride.
    int8_t bit_stride;
  };

  // These segments are indexed by rows from top to bottom, and describe the
  // positions of the lights in the uint16_t encoding. Each row is
  // left-to-right.
  static constexpr const Segment kRowSegments[4] = {
    // Row 0 (north; left-to-right)
    { .bit_num = 0, .bit_stride = 1 },
    // Row 1 (upper middle row)
    { .bit_num = 4, .bit_stride = 1 },
    // Row 2 (lower middle row)
    { .bit_num = 8, .bit_stride = 1 },
    // Row 3 (south, left-to-right)
    { .bit_num = 12, .bit_stride = 1 },
  };

  // These segments are indexed by a Direction, and describe the respective
  // edge, in a CLOCKWISE manner. This means that North is left-to-right, East
  // is top-down, South is right-to-left, West is bottom-up.
  static constexpr const Segment kEdgeSegments[4] = {
    // North; left-to-right
    { .bit_num = 0, .bit_stride = 1 },
    // East; top-down
    { .bit_num = 3, .bit_stride = 4 },
    // South; right-to-left
    { .bit_num = 15, .bit_stride = -1 },
    // West; bottom-up
    { .bit_num = 12, .bit_stride = -4 },
  };


private:
  // do not instantiate this class.
  ProtocolDefs();
};  // struct ProtocolDefs




#endif  //_GOL_PROTOCOL_DEFS_H_
