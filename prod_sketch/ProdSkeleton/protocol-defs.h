#ifndef _GOL_PROTOCOL_DEFS_H_
#define _GOL_PROTOCOL_DEFS_H_

enum Direction : uint8_t {
  kNorth = 0,
  kEast = 1,
  kSouth = 2,
  kWest = 3
};

struct ProtocolDefs {
public:
  static constexpr uint64_t kEventPrefix = UINT64_C(0x09000D0000000000);
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
    // Reports the current state of a node. x,y defines the node. arg is 16-bit for the node's state. Sent by each node.
    kStateReport = 0xF0,
    // Overrides the state of a node. x,y defines the node. arg is 16-bit for the node's state. Sent by an external configuring agent.
    kStateSet = 0xF4,
    // Modifies the state of a node by adding certain cells (with an OR operator). x,y defines the node. arg is 16-bit for the bits to set in the node's state. Sent by an external configuring agent.
    kStateOr = 0xF5,
    // Listen for a local signal and set the coordinates. x,y are the new coordinates. arg unused. Sent by the master.
    kLocalAssign = 0xF2,
    // Reports that the local signal was received, and coordinates are now assigned. x,y are the new coordinates. Arg unused. Sent by the node.
    kLocalFound = 0xF3,
    // Requests a local signal to be toggled. dir is the lowest two bits of the command. x,y are the target coordinate. Arg unused. Sent by the master.
    kToggleLocalSignal = 0xEC,
    // Reports who is our neighbor. x,y, are the source (reporting) node. argx-argy are the neighbor node. dir of the reporting node is the lowest two bits of the command. The neighbor's direction is the bits 2-3 of the command. Sent by the node.
    kReportNeighbor = 0xD0,
    // Sets who is our neighbor. x,y, are the target node (to change). argx-argy are the neighbor node. dir of the target node is the lowest two bits of the command. The neighbor's direction is the bits 2-3 of the command. Sent by the master.
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

private:
  // do not instantiate this class.
  ProtocolDefs();
};  // struct ProtocolDefs




#endif  //_GOL_PROTOCOL_DEFS_H_