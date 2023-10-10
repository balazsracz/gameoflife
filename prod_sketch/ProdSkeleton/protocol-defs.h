#ifndef _GOL_PROTOCOL_DEFS_H_
#define _GOL_PROTOCOL_DEFS_H_

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

    // Direction bit values.
    kDirNorth = 0,
    kDirEast = 1,
    kDirSouth = 2,
    kDirWest = 3,
  };

private:
  // do not instantiate this class.
  ProtocolDefs();
};  // struct ProtocolDefs




#endif  //_GOL_PROTOCOL_DEFS_H_