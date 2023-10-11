#include "protocol-defs.h"

class ProtocolEngineInterface {
public:
  // Sends an event to the bus.
  // @param event_id the 64-bit payload of the event to send.
  // @return true if the message was sent, false if it was dropped.
  virtual bool SendEvent(uint64_t event_id) = 0;

  // @return the currently used alias of the local node.
  virtual uint16_t GetAlias() = 0;

  virtual void LocalBusSignal(Direction dir, bool active) = 0;
  virtual bool LocalBusIsActive(Direction dir) = 0;

  // @return the current time.
  virtual uint32_t millis() = 0;

  // @return true if there is a pending send.
  virtual bool TxPending() = 0;

  // Reboot the microcontroller. Does not return.
  virtual void Reboot() = 0;
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
    if (pending_x_ != INVALID_COORD && pending_y_ != INVALID_COORD) {
      LookForLocalSignal();
    }
    if (next_neighbor_report_ < INVALID_DIR && !iface_->TxPending()) {
      const auto& l = neighbors_[next_neighbor_report_];
      if (l.neigh_x != INVALID_COORD) {
        // there is a neighbor here.
        iface_->SendEvent(Defs::CreateEvent(Defs::kReportNeighbor, my_x_, my_y_, l.neigh_x, l.neigh_y, (Direction)next_neighbor_report_, l.neigh_dir));
      }
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
          pending_x_ = INVALID_COORD;
          pending_y_ = INVALID_COORD;
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
    next_neighbor_report_ = INVALID_DIR;
    my_x_ = my_y_ = pending_x_ = pending_y_ = pending_neigh_x_ = pending_neigh_y_ = INVALID_COORD;
    timeout_ = INVALID_TIMEOUT;
    neighbors_.clear();
    neighbors_.resize(4);  // number of directions
  }

  // @return true if this event is targeting me in the x/y parameters.
  bool ForMe(uint64_t ev) {
    return Defs::GetX(ev) == my_x_ && Defs::GetY(ev) == my_y_;
  }

  // Called by the event handler when a global command event arrives.
  // @param arg the lowest bits of the event id (determine what to do).
  void HandleGlobalCommand(uint16_t arg) {
    Defs::GlobalCommand gcmd = (Defs::GlobalCommand)arg;
    switch (gcmd) {
      case Defs::kReboot:
        iface_->Reboot();
        return;
      case Defs::kReInit:
        InitState();
        return;
      case Defs::kReportNeighbors:
        // This will start the state machine in the Loop().
        next_neighbor_report_ = 0;
        return;
    }
  }

  // Stops emitting a local signal to all directions.
  void CancelLocalSignal() {
    for (auto dir : { kNorth, kEast, kSouth, kWest }) {
      iface_->LocalBusSignal(dir, false);
    }
  }

  // Called from the loop when the timeout expires.
  void HandleTimeout() {
    timeout_ = INVALID_TIMEOUT;
    if (to_cancel_local_signal_) {
      CancelLocalSignal();
      to_cancel_local_signal_ = false;
    }
  }

  // Called from Loop() when there is an active neighbor search declared on the bus.
  void LookForLocalSignal() {
    bool found = false;
    for (auto dir : { kNorth, kEast, kSouth, kWest }) {
      if (iface_->LocalBusIsActive(dir)) {
        if (my_x_ == INVALID_COORD || my_y_ == INVALID_COORD) {
          my_x_ = pending_x_;
          my_y_ = pending_y_;
        }
        // We are the targeted neighbor. Report this.
        iface_->SendEvent(Defs::CreateEvent(Defs::kLocalFound, my_x_, my_y_));
        found = true;
        if (my_x_ != pending_x_ || my_y_ != pending_y_) {
          // Error: we have to different set of coordinates.
          iface_->SendEvent(Defs::CreateEvent(Defs::kLocalConflict, my_x_, my_y_, pending_x_, pending_y_));
        }
        // Records the neighbor in the links we know about.
        neighbors_[dir].neigh_x = pending_neigh_x_;
        neighbors_[dir].neigh_y = pending_neigh_y_;
        neighbors_[dir].neigh_dir = pending_neigh_dir_;
      }
    }
    if (found) {
      pending_x_ = INVALID_COORD;
      pending_y_ = INVALID_COORD;
    }
  }

  ProtocolEngineInterface* iface_;

  static constexpr uint8_t INVALID_COORD = 0xff;
  // Coordinates to be assigned via a local signal. This is already adjusted from the neighbor with deltaxy.
  uint8_t pending_x_{ INVALID_COORD };
  uint8_t pending_y_{ INVALID_COORD };

  // Which neighbor was recently instructed to do a local trigger.
  uint8_t pending_neigh_x_{ INVALID_COORD };
  uint8_t pending_neigh_y_{ INVALID_COORD };
  // Which direction on the neighbor that was triggered.
  Direction pending_neigh_dir_;

  // My assigned coordinates.
  uint8_t my_x_{ INVALID_COORD };
  uint8_t my_y_{ INVALID_COORD };

  struct Link {
    // Coordinates of the neighbor
    uint8_t neigh_x{ INVALID_COORD };
    uint8_t neigh_y{ INVALID_COORD };
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

  // This value in next_neighbor_report_ is invalid.
  static constexpr unsigned INVALID_DIR = 4;
  // If this is 0..3, then a neighbor report needs to be emitted.
  unsigned next_neighbor_report_ : 3;
};  // class ProtocolEngine
