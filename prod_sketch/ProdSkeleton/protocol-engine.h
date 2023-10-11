#include "protocol-defs.h"

class ProtocolEngineInterface {
public:
  // Sends an event to the bus.
  // @param event_id the 64-bit payload of the event to send.
  virtual void SendEvent(uint64_t event_id) = 0;

  // @return the currently used alias of the local node.
  virtual uint16_t GetAlias() = 0;

  virtual void LocalBusSignal(Direction dir, bool active) = 0;
  virtual bool LocalBusIsActive(Direction dir) = 0;

  // @return the current time.
  virtual uint32_t millis() = 0;
};  // class ProtocolEngineInterface

class ProtocolEngine {
public:
  ProtocolEngine() {
    InitState();
  }

  // Call this function once from setup().
  // @param iface implementation object proxying a variety of functions to the global state machines. Ownership is NOT transferred.
  void Setup(ProtocolEngineInterface* iface) {
    iface_ = iface;
  }

  // Call this function from the loop() handler.
  void Loop() {
    if (iface_->millis() >= timeout_) {
      HandleTimeout();
    }
    if (pending_x_ != 0xff && pending_y_ != 0xff) {
      LookForLocalSignal();
    }
  }

  // Call this function when a global event arrives.
  // @param ev the 64-bit event payload.
  // @param src the source node alias that sent this event
  void OnGlobalEvent(int64_t ev, uint16_t src) {
    Defs::Command cmd = Defs::GetCommand(ev);
    switch (cmd) {
      case Defs::kGlobalCmd:
        {
          return HandleGlobalCommand(Defs::GetArg(ev));
        }
      case Defs::kLocalAssign:
        {
          pending_x_ = Defs::GetX(ev);
          pending_y_ = Defs::GetY(ev);
          break;
        }
      case Defs::kLocalFound:
        {
          pending_x_ = 0xff;
          pending_y_ = 0xff;
          // If we are sending a signal right now.
          if (to_cancel_local_signal_) {
            CancelLocalSignal();
            timeout_ = INVALID_TIMEOUT;
            to_cancel_local_signal_ = false;
          }
          break;
        }
      case Defs::kToggleLocalSignal:
        {
          Direction dir = (Direction)(cmd & 0x3);
          if (ForMe(ev)) {
            timeout_ = iface_->millis() + 2;
            iface_->LocalBusSignal(dir, true);
            to_cancel_local_signal_ = true;
          } else {
            // Remembers who is being toggled.
            pending_neigh_dir_ = dir;
            pending_neigh_x_ = Defs::GetX(ev);
            pending_neigh_y_ = Defs::GetY(ev);
          }
          break;
        }
    }
  }


private:
  using Defs = ProtocolDefs;

  // Indexed with a Direction, delta x coordinate.
  static constexpr const int deltax[4] = { 0, 1, 0, -1 };
  // Indexed with a Direction, delta y coordinate.
  static constexpr const int deltay[4] = { 1, 0, -1, 0 };

  // Restores the internal state to a fresh boot.
  void InitState() {
    to_cancel_local_signal_ = false;
    my_x_ = my_y_ = pending_x_ = pending_y_ = pending_neigh_x_ = pending_neigh_y = 0xff;
    timeout_ = INVALID_TIMEOUT;
    neighbors_.clear();

  }

  // @return true if this event is targeting me in the x/y parameters.
  bool ForMe(uint64_t ev) {
    return Defs::GetX(ev) == my_x_ && Defs::GetY(ev) == my_y_;
  }

  // Called by the event handler when a global command event arrives.
  // @param arg the lowest bits of the event id (determine what to do).
  void HandleGlobalCommand(uint16_t arg) {}

  // Stops emitting a local signal to all directions.
  void CancelLocalSignal() {
    iface_->LocalBusSignal(Defs::kNorth, false);
    iface_->LocalBusSignal(Defs::kSouth, false);
    iface_->LocalBusSignal(Defs::kEast, false);
    iface_->LocalBusSignal(Defs::kWest, false);
  }

  // Called from the loop when the timeout expires.
  void HandleTimeout() {
    timeout_ = INVALID_TIMEOUT;
    if (cancel_local_signal_) {
      CancelLocalSignal();
      cancel_local_signal_ = false;
    }
  }

  // Called when there is an active neighbor search declared on the bus.
  void LookForLocalSignal() {
    if (iface_->LocalBusIsActive(Defs::kNorth)) {
    }
  }

  ProtocolEngineInterface* iface_;

  // Coordinates to be assigned via a local signal. This is already adjusted from the neighbor with deltaxy.
  uint8_t pending_x_{ 0xff };
  uint8_t pending_y_{ 0xff };

  // Which neighbor was recently instructed to do a local trigger.
  uint8_t pending_neigh_x_{ 0xff };
  uint8_t pending_neigh_y_{ 0xff };
  // Which direction on the neighbor that was triggered.
  Direction pending_neigh_dir_;

  // My assigned coordinates.
  uint8_t my_x_{ 0xff };
  uint8_t my_y_{ 0xff };

  struct Link {
    // Direction on my side
    Direction my_dir;
    // Coordinates of the neighbor
    uint8_t neigh_x;
    uint8_t neigh_y;
    // Direction on the neighbor side
    Direction neigh_dir;
  };
  // Neighbor's assigned coordinates.
  std::vector<Link> neighbors_;

  static constexpr uint32_t INVALID_TIMEOUT = (uint32_t)-1;
  // HAL tick when we should move away from from
  uint32_t timeout_{ INVALID_TIMEOUT };

  // what to do when we are done with the timeout.
  bool to_cancel_local_signal_ : 1;
};  // class ProtocolEngine
