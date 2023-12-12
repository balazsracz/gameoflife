#ifndef _PROTOCOL_ENGINE_H_
#define _PROTOCOL_ENGINE_H_

#include "protocol-defs.h"

#include <queue>
#include <map>

class ProtocolEngineInterface {
public:
  // Sends an event to the bus.
  // @param event_id the 64-bit payload of the event to send.
  // @return true if the message was sent, false if it was dropped.
  virtual bool SendEvent(uint64_t event_id) = 0;

  // @return the currently used alias of the local node,  or 0 if the node is not initialized yet.
  virtual uint16_t GetAlias() = 0;

  // @return the hardware's global address
  virtual uint64_t GetGlobalAddress() = 0;

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
  static constexpr unsigned kDeadBoardIterationThreshold = 10;

  ProtocolEngine() {
  }

  uint8_t GetX() {
    return my_x_;
  }
  uint8_t GetY() {
    return my_y_;
  }

  /// tells which LEDs the leader wants to light.
  /// @return a but mask for leds 1..16 to light up. The high bits specify
  /// which LEDs should be set to zero as well.
  uint32_t GetMenuLeds() {
    uint16_t ret = 0;
    if (disc_state_ != kNotRunning) {
      return 0xffff;
    }
    auto m = iface_->millis();
    unsigned blk = (m & 1023) / 128;  // 0..7
    if (is_leader_ && menu_active_ ) {
      static constexpr unsigned corners = 0b1001000000001001;
      // light up 4 corners.
      uint32_t ret = corners << 16;
      if (menu_second_page_) {
        if (blk & 1) ret |= corners;
      } else {
        if (blk & 2) ret |= corners;
      }
      return ret;
    }
    return 0;
  }

  // Call this function once from setup().
  // @param iface implementation object proxying a variety of functions to the global state machines. Ownership is NOT transferred.
  void Setup(ProtocolEngineInterface* iface) {
    iface_ = iface;
    InitState();
  }

  static constexpr unsigned kTestX = 0x85;
  static constexpr unsigned kTestY = 0x81;

  // Initializes the internal data structures with the following data:
  // X=85, Y=80
  // neighbors with +- 1.
  void SetupTest() {
    InitState();
    my_x_ = kTestX;
    my_y_ = kTestY;
    neighbors_[kNorth] = { .neigh_x = 0x85, .neigh_y = 0x80, .neigh_dir = kSouth };
    neighbors_[kSouth] = { .neigh_x = 0x85, .neigh_y = 0x82, .neigh_dir = kNorth };
    neighbors_[kEast] = { .neigh_x = 0x86, .neigh_y = 0x81, .neigh_dir = kWest };
    neighbors_[kWest] = { .neigh_x = 0x84, .neigh_y = 0x81, .neigh_dir = kEast };
    neighbors_[kNorthEast] = { .neigh_x = 0x86, .neigh_y = 0x80, .neigh_dir = kSouthWest, .pixel_offset = 12 };
    neighbors_[kNorthWest] = { .neigh_x = 0x84, .neigh_y = 0x80, .neigh_dir = kSouthEast, .pixel_offset = 15 };
    neighbors_[kSouthEast] = { .neigh_x = 0x86, .neigh_y = 0x82, .neigh_dir = kNorthWest, .pixel_offset = 0 };
    neighbors_[kSouthWest] = { .neigh_x = 0x84, .neigh_y = 0x82, .neigh_dir = kNorthEast, .pixel_offset = 3 };
  }

  // Call this function from the loop() handler.
  void Loop() {
    auto millis = iface_->millis();
    if (millis >= timeout_) {
      HandleTimeout();
    }
    //if (pending_x_ != INVALID_COORD && pending_y_ != INVALID_COORD) {
    {
      LookForLocalSignal();
    }
    if (next_neighbor_report_ < INVALID_DIR && !iface_->TxPending()) {
      if (my_x_ == INVALID_COORD) {
        iface_->SendEvent(Defs::CreateEvent(Defs::kGlobalCmd, 0, 0, Defs::kInitDone));
        next_neighbor_report_ = INVALID_DIR;
      } else {
        const auto& l = neighbors_[next_neighbor_report_];
        if (l.neigh_x != INVALID_COORD) {
          // there is a neighbor here.
          iface_->SendEvent(Defs::CreateEvent(Defs::kReportNeighbor, my_x_, my_y_, l.neigh_x, l.neigh_y, (Direction)next_neighbor_report_, l.neigh_dir));
        }
        ++next_neighbor_report_;
      }
    }
    if (need_init_done_ && iface_->GetAlias() != 0 && !iface_->TxPending()) {
      iface_->SendEvent(Defs::CreateEvent(Defs::kGlobalCmd, 0, 0, Defs::kInitDone));
      need_init_done_ = false;
      // Waits half a second before starting a leader election.
      election_start_timeout_ = millis + 500;
    }
    if (!need_init_done_ && !seen_leader_ && !to_leader_election_ && millis >= election_start_timeout_) {
      // Let's trigger a leader election.
      ParticipateLeaderElection();
    }
    if (to_leader_election_ && millis >= idle_timeout_) {
      TimeoutLeaderElection();
    }
    if (need_full_discovery_ && disc_state_ == kNotRunning) {
      disc_state_ = kStartFullDiscovery;
      need_full_discovery_ = false;
    }
    if (need_partial_discovery_ && disc_state_ == kNotRunning && millis >= idle_timeout_) {
      disc_state_ = kStartPartialDiscovery;
      need_partial_discovery_ = false;
    }
    DoDiscovery();
    if (run_tick_ && disc_state_ == kNotRunning && millis >= tick_timeout_) {
      auto ev = Defs::CreateGlobalCmd(Defs::kEvolveAndReport);
      iface_->SendEvent(ev);
      tick_timeout_ += evolution_speed_msec_;
    }
  }

  // Call this function when a global event arrives.
  // @param ev the 64-bit event payload.
  // @param src the source node alias that sent this event
  void OnGlobalEvent(int64_t ev, uint16_t src) {
    if (!Defs::IsProtocolEvent(ev)) return;
    idle_timeout_ = iface_->millis() + 10;
    if (to_leader_election_) {
      SetElectionTimeout();
    }
    Defs::Command cmd = Defs::GetCommand(ev);
    switch (cmd) {
      case Defs::kGlobalCmd:
        {
          return HandleGlobalCommand(Defs::GetArg(ev), src);
        }
      case Defs::kLocalAssign:
        {
          pending_x_ = Defs::GetX(ev);
          pending_y_ = Defs::GetY(ev);
          break;
        }
      case Defs::kLocalFound:
        {
          // If we are sending a signal right now.
          if (to_cancel_local_signal_) {
            CancelLocalSignal();
            timeout_ = INVALID_TIMEOUT;
            to_cancel_local_signal_ = false;
          }
          // Discovery event found
          if (to_disc_neighbor_lookup_) {
            to_disc_neighbor_lookup_ = false;
            disc_timeout_ = INVALID_TIMEOUT;
            AddNodeToDiscovery(leader_pending_x_, leader_pending_y_, src);
          }
          pending_x_ = INVALID_COORD;
          pending_y_ = INVALID_COORD;
          break;
        }
      case Defs::kToggleLocalSignal:
        {
          Direction dir = Defs::GetDir(ev);
          if (ForMe(ev)) {
            timeout_ = iface_->millis() + kLocalBusSignalTimeoutMsec;
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
      case Defs::kLocalSpurious:
        {
          if (!disc_catchup_pending_) return;
          auto x = Defs::GetX(ev);
          auto y = Defs::GetY(ev);
          disq_.push(GetCoord(x, y));
          return;
        }
      case Defs::kReportNeighbor:
        for (auto dir : { kNorth, kEast, kSouth, kWest }) {
          if (neighbors_[dir].neigh_x == Defs::GetX(ev) && neighbors_[dir].neigh_y == Defs::GetY(ev)) {
            ProcessNeighborReport(dir, ev);
          }
        }
        return;
      case Defs::kStateReport:
        {
          if (Defs::GetArg(ev) != 0) {
            seen_non_zero_ = true;
          }
          uint32_t report = ev & 0xffffffffu;
          uint64_t h = Murmur3(report, 0xa81f4f4c);
          h <<= 32;
          h |= Murmur3(report, 0x7d55d81a);
          curr_hash_ ^= h;
          return;
        }
      case Defs::kButtonPressed:
        if (!is_leader_) {
          return;
        }
        if (Defs::GetArg(ev) == 16) {
          // menu button
          menu_active_ = !menu_active_;
          menu_selected_ = kMenuNone;
        } else {
          ExecuteButton(Defs::GetX(ev), Defs::GetY(ev), Defs::GetArg(ev));
        }
        return;
    }
  }

private:
  // Called by the event handler when a global command event arrives.
  // @param arg the lowest bits of the event id (determine what to do).
  void HandleGlobalCommand(uint16_t arg, uint16_t src) {
    Defs::GlobalCommand gcmd = (Defs::GlobalCommand)arg;
    switch (gcmd) {
      case Defs::kReboot:
        if (src == iface_->GetAlias()) return;
        iface_->Reboot();
        return;
      case Defs::kReInit:
        if (src != iface_->GetAlias()) {
          InitState();
        }
        return;
      case Defs::kEnterFirmware:
        EnterBootloader();
        return;
      case Defs::kInitDone:
        if (src == iface_->GetAlias()) return;
        if (src == leader_alias_) {
          // Lost the leader.
          seen_leader_ = false;
          leader_alias_ |= 0xF000;
        } else if (is_leader_) {
          // Tell the new node that we are the leader.
          iface_->SendEvent(Defs::CreateEvent(Defs::kGlobalCmd, 0, 0, Defs::kIAmLeader));
          need_partial_discovery_ = true;
        }
        return;
      case Defs::kReportNeighbors:
        // Clears any remnants of a previous discovery.
        pending_x_ = INVALID_COORD;
        pending_y_ = INVALID_COORD;
        // This will start the state machine in the Loop().
        next_neighbor_report_ = 0;
        // Forgets all diagonal neighbors.
        neighbors_.resize(4);
        neighbors_.resize(8);
        return;
      case Defs::kIAmLeader:
        if (src == iface_->GetAlias()) return;
        if (is_leader_) {
          SerialUSB.printf("Seen conflicting leader me=%03x known=%03x other=%03x", iface_->GetAlias(), leader_alias_, src);
          is_leader_ = false;
          GlobalBusCancelPendingTx();
        }
        seen_leader_ = true;
        to_leader_election_ = false;
        leader_alias_ = src;
        return;
      case Defs::kProposeLeader:
        if (src == iface_->GetAlias()) return;
        if (is_leader_) {
          // debunk
          iface_->SendEvent(Defs::CreateEvent(Defs::kGlobalCmd, 0, 0, Defs::kIAmLeader));
          to_leader_election_ = false;
          return;
        }
        if (src < leader_alias_) {
          leader_alias_ = src;
        }
        if (!seen_leader_ && !to_leader_election_) {
          // Someone else started a leader election.
          ParticipateLeaderElection();
        }
        SetElectionTimeout();
        break;
      case Defs::kLocalToggleUnassigned:
        if (my_x_ == INVALID_COORD && my_y_ == INVALID_COORD) {
          timeout_ = iface_->millis() + kLocalBusSignalTimeoutMsec;
          for (auto dir : { kNorth, kEast, kSouth, kWest }) {
            iface_->LocalBusSignal(dir, true);
          }
          to_cancel_local_signal_ = true;
        }
        return;
      case Defs::kFindLeader:
        if (is_leader_) {
          iface_->SendEvent(Defs::CreateGlobalCmd(Defs::kIAmLeader));
        }
        return;
      case Defs::kStartIteration:
        if (is_leader_) {
          run_tick_ = true;
          tick_timeout_ = idle_timeout_;
        }
        return;
      case Defs::kStopIteration:
        run_tick_ = false;
        return;
      case Defs::kEvolveAndReport:
        if (is_leader_) {
          if (curr_hash_ == last_hash_[0] || curr_hash_ == last_hash_[1] || !seen_non_zero_) {
            // uninteresting state
            ++num_bad_states_;
            if (num_bad_states_ > kDeadBoardIterationThreshold && detect_steady_state_) {
              iface_->SendEvent(Defs::CreateGlobalCmd(Defs::kSetStateRandom));
            }
          } else {
            num_bad_states_ = 0;
          }
        }
        last_hash_[1] = last_hash_[0];
        last_hash_[0] = curr_hash_;
        curr_hash_ = 0;
        seen_non_zero_ = false;
        return;
      case Defs::kReportCalibration:
        {
          extern uint64_t channel_sum[2][4];
          extern unsigned channel_count[2][4];
          uint8_t x = (channel_sum[0][0] / channel_count[0][0]) >> 4;
          uint8_t y = (channel_sum[0][2] / channel_count[0][2]) >> 4;
          iface_->SendEvent(Defs::CreateEvent(Defs::kGlobalCmd, x, y, Defs::kCalibrationData));
          return;
        }
      case Defs::kReportAddress:
        {
          iface_->SendEvent(Defs::CreateEvent(Defs::kCurrentAddressReport, GetX(), GetY(), iface_->GetGlobalAddress() & 0xffffu));
          return;
        }
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

  void TimeoutLeaderElection() {
    // Leader election complete.
    to_leader_election_ = false;
    idle_timeout_ = iface_->millis() + 10;
    iface_->SendEvent(Defs::CreateEvent(Defs::kDeclareLeader, 0, 0, leader_alias_));
    if (leader_alias_ == iface_->GetAlias()) {
      is_leader_ = true;
      need_full_discovery_ = true;
      seen_leader_ = true;
      iface_->SendEvent(Defs::CreateEvent(Defs::kGlobalCmd, 0, 0, Defs::kIAmLeader));
      SerialUSB.printf("I am leader %03x\n", leader_alias_);
    } else {
      SerialUSB.printf("Leader is %03x\n", leader_alias_);
    }
  }

  using Defs = ProtocolDefs;

  // Indexed with a Direction (N,E,S,W), delta x coordinate.
  static constexpr const int deltax[4] = { 0, 1, 0, -1 };
  // Indexed with a Direction, delta y coordinate.
  static constexpr const int deltay[4] = { -1, 0, 1, 0 };

  static constexpr unsigned kLocalNeighborLookupTimeoutMsec = 8;
  static constexpr unsigned kLocalBusSignalTimeoutMsec = 4;

  // Restores the internal state to a fresh boot.
  void InitState() {
    GlobalBusCancelPendingTx();
    to_cancel_local_signal_ = false;
    to_leader_election_ = false;
    to_disc_neighbor_lookup_ = false;
    next_neighbor_report_ = INVALID_DIR;
    my_x_ = my_y_ = pending_x_ = pending_y_ = pending_neigh_x_ = pending_neigh_y_ = leader_pending_x_ = leader_pending_y_ = INVALID_COORD;
    timeout_ = INVALID_TIMEOUT;
    election_start_timeout_ = INVALID_TIMEOUT;
    idle_timeout_ = iface_->millis() + 10;
    neighbors_.clear();
    neighbors_.resize(8);  // number of directions
    need_init_done_ = true;
    seen_leader_ = false;
    is_leader_ = false;
    leader_alias_ = 0xF000;
    known_local_signal_ = 0;
    need_partial_discovery_ = false;
    need_full_discovery_ = false;
    disc_catchup_pending_ = false;
    disc_state_ = kNotRunning;
    evolution_speed_msec_ = Defs::kDefaultEvolutionSpeedMsec;
    tick_timeout_ = INVALID_TIMEOUT;
    run_tick_ = true;
    num_bad_states_ = kDeadBoardIterationThreshold - 2;
    detect_steady_state_ = true;
    menu_active_ = false;
    menu_second_page_ = false;
    menu_selected_ = kMenuNone;
  }

  // @return true if this event is targeting me in the x/y parameters.
  bool ForMe(uint64_t ev) {
    return Defs::GetX(ev) == my_x_ && Defs::GetY(ev) == my_y_;
  }

  // Sends an event to the bus to participate in leader election.
  void ParticipateLeaderElection() {
    iface_->SendEvent(Defs::CreateEvent(Defs::kGlobalCmd, 0, 0, Defs::kProposeLeader));
    leader_alias_ = std::min(iface_->GetAlias(), leader_alias_);
    SetElectionTimeout();
  }

  // Stops emitting a local signal to all directions.
  void CancelLocalSignal() {
    for (auto dir : { kNorth, kEast, kSouth, kWest }) {
      iface_->LocalBusSignal(dir, false);
    }
  }

  // Sets the timeout to the idle delay from this leader election message.
  void SetElectionTimeout() {
    to_leader_election_ = true;
    idle_timeout_ = iface_->millis() + (leader_alias_ == iface_->GetAlias() ? 10 : 14);
  }

  // Checks if a node is known or not. If not known, adds the node to known and enqueues it for neighbor discovery.
  void AddNodeToDiscovery(uint8_t x, uint8_t y, uint16_t alias) {
    NodeCoord nc = GetCoord(x, y);
    NodeInfo ni{ .alias_ = alias };
    if (known_nodes_.insert({ nc, ni }).second) {
      // new node
      disq_.push(nc);
    }
  }

  // State machine for running the discovery process after startup or after
  // seeing nodes rebooting.
  enum DiscoveryState : uint8_t {
    kNotRunning = 0,
    kWaitForSetup,
    kSendSetup,
    kWaitForSetupResponses,
    kAnnounceLocal,
    kWaitLocalAnnounce,
    // Iteration on the queue
    kSendLocalAssign,
    kWaitLocalTriggerSend,
    kWaitLocalFeedback,
    // Iteration end
    kSendNeighborReport,
    kWaitNeighborResponses,
    kDiscoveryDone,

    kPartialSendSetup,
    kPartialWaitSetupResponses,


    kIterationFirst = kSendLocalAssign,
    kStartFullDiscovery = kWaitForSetup,
    kStartPartialDiscovery = kPartialSendSetup,
  };

  void DoDiscovery() {
    switch (disc_state_) {
      case kNotRunning:
        return;
      case kWaitForSetup:
        //run_tick_ = false;
        if (iface_->GetAlias() == 0) { return; }
        if (iface_->TxPending()) { return; }
        need_partial_discovery_ = false;
        while (!disq_.empty()) { disq_.pop(); }
        known_nodes_.clear();
        to_disc_neighbor_lookup_ = false;

        disc_state_ = kSendSetup;
        return;
      case kSendSetup:
        // Requests everyone else to drop their state.
        
        // TODO: this reinit was here to ensure that the system starts up in a
        // definite state. However, it ended up causing infinite loops during
        // startup.

        //iface_->SendEvent(Defs::CreateGlobalCmd(Defs::kReInit));
        idle_timeout_ = iface_->millis() + 10;
        disc_state_ = kWaitForSetupResponses;
        return;

      case kWaitForSetupResponses:
        if (iface_->millis() < idle_timeout_) return;
        need_partial_discovery_ = false;
        disc_state_ = kAnnounceLocal;
        return;
      case kAnnounceLocal:
        // Setup local address.
        my_x_ = 0x80;
        my_y_ = 0x80;
        iface_->SendEvent(Defs::CreateEvent(Defs::kLocalFound, my_x_, my_y_));
        AddNodeToDiscovery(my_x_, my_y_, iface_->GetAlias());
        disc_neighbor_dir_ = -1;
        disc_state_ = kWaitLocalAnnounce;
        return;
      case kWaitLocalAnnounce:
        if (iface_->TxPending()) return;
        disc_state_ = kIterationFirst;
        return;
        // beginning of iteration
      case kSendLocalAssign:
        {
          ++disc_neighbor_dir_;
          if (disc_neighbor_dir_ > kMaxDirection) {
            // This entry is done.
            disq_.pop();
            disc_neighbor_dir_ = 0;
            if (disq_.empty()) {
              disc_state_ = kSendNeighborReport;
              return;
            }
          }
          uint8_t x = GetCoordX(disq_.front());
          uint8_t y = GetCoordY(disq_.front());
          uint8_t nx = x + deltax[disc_neighbor_dir_];
          uint8_t ny = y + deltay[disc_neighbor_dir_];
          to_disc_neighbor_lookup_ = true;
          leader_pending_x_ = nx;
          leader_pending_y_ = ny;
          disc_timeout_ = iface_->millis() + kLocalNeighborLookupTimeoutMsec;
          iface_->SendEvent(Defs::CreateEvent(Defs::kLocalAssign, nx, ny));
          iface_->SendEvent(Defs::CreateEvent(Defs::kToggleLocalSignal, x, y, 0, 0, (Direction)disc_neighbor_dir_));
          disc_state_ = kWaitLocalTriggerSend;
          return;
        }
      case kWaitLocalTriggerSend:
        if (iface_->TxPending()) return;
        disc_state_ = kWaitLocalFeedback;
        return;
      case kWaitLocalFeedback:
        if (to_disc_neighbor_lookup_ && (iface_->millis() < disc_timeout_)) return;
        to_disc_neighbor_lookup_ = false;
        disc_state_ = kIterationFirst;
        return;
      // end of iteration.
      case kSendNeighborReport:
        iface_->SendEvent(Defs::CreateGlobalCmd(Defs::kReportNeighbors));
        disc_state_ = kWaitNeighborResponses;
        return;
      case kWaitNeighborResponses:
        if (iface_->millis() < idle_timeout_) return;
        disc_state_ = kDiscoveryDone;
        return;

      case kDiscoveryDone:
        disc_state_ = kNotRunning;
        //run_tick_ = true;
        tick_timeout_ = iface_->millis();
        return;
      case kPartialSendSetup:
        //run_tick_ = false;
        iface_->SendEvent(Defs::CreateGlobalCmd(Defs::kLocalToggleUnassigned));
        disc_catchup_pending_ = true;
        need_partial_discovery_ = false;
        idle_timeout_ = iface_->millis() + 10;
        disc_state_ = kPartialWaitSetupResponses;
        return;
      case kPartialWaitSetupResponses:
        if (iface_->millis() < idle_timeout_) return;
        disc_catchup_pending_ = false;
        if (disq_.empty()) {
          // Did not get any neighbors reporting. Fall back to full discovery.
          disc_state_ = kStartFullDiscovery;
          return;
        }
        disc_neighbor_dir_ = -1;
        disc_state_ = kIterationFirst;
        return;
    }
  }

  // Called from Loop() when there is an active neighbor search declared on the bus.
  void LookForLocalSignal() {
    bool found = false;
    for (auto dir : { kNorth, kEast, kSouth, kWest }) {
      unsigned dir_bit = 1u << (unsigned)dir;
      if (iface_->LocalBusIsActive(dir)) {
        if (pending_x_ == INVALID_COORD && pending_y_ == INVALID_COORD) {
          if ((known_local_signal_ & dir_bit) == 0 && !to_cancel_local_signal_) {
            // Not sure what they want to assign.
            if (my_x_ != INVALID_COORD && my_y_ != INVALID_COORD) {
              auto ev = Defs::CreateEvent(Defs::kLocalSpurious, my_x_, my_y_);
              iface_->SendEvent(ev);
            }
          }
          known_local_signal_ |= dir_bit;
          continue;
        }
        known_local_signal_ |= dir_bit;
        if (to_cancel_local_signal_) {  // might be our signal
          continue;
        }
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
          my_x_ = pending_x_;
          my_y_ = pending_y_;
        } else if (to_disc_neighbor_lookup_) {
          // search for neighbors reached back to the master
          to_disc_neighbor_lookup_ = false;
        }
        // Records the neighbor in the links we know about.
        neighbors_[dir].neigh_x = pending_neigh_x_;
        neighbors_[dir].neigh_y = pending_neigh_y_;
        neighbors_[dir].neigh_dir = pending_neigh_dir_;
      } else {
        known_local_signal_ &= ~dir_bit;
      }
    }
    if (found) {
      pending_x_ = INVALID_COORD;
      pending_y_ = INVALID_COORD;
    }
  }

  // Called when `ev` is a neighbor report coming from a node that we remember is our actual neighbor in direction `dir`.
  void ProcessNeighborReport(Direction dir, uint64_t ev) {
    // This is the edge of the neighbor cell that looks towards us.
    Direction reverse_dir = neighbors_[dir].neigh_dir;
    // This is the edge if we go there and turn left.
    Direction left_dir = IncDir(reverse_dir);
    // This is the edge if we go there and turn right.
    Direction right_dir = DecDir(reverse_dir);
    bool found = false;
    Direction found_dir;
    unsigned num_stride = 0;
    if (left_dir == Defs::GetDir(ev)) {
      // We found a diagonal.
      found = true;
      // +4 turns north to northwest.
      found_dir = Direction(int(dir) + 4);
      // We turned left, so we have the first bit of that edge. Opposite that will be the last bit of the neighbor.
      num_stride = 3;
    }
    if (right_dir == Defs::GetDir(ev)) {
      // We found a diagonal.
      found = true;
      // +4 +1 turns north to northeast.
      found_dir = Direction(int(IncDir(dir)) + 4);
      // We turned right, so we have the last bit of that edge. Opposite that will be the first bit of the neighbor.
      num_stride = 0;
    }
    if (found) {
      auto& l = neighbors_[found_dir];
      l.neigh_x = Defs::GetArgX(ev);
      l.neigh_y = Defs::GetArgY(ev);
      // This is the direction on the diagonal neighbor that looks towards the neighbor.
      l.neigh_dir = Defs::GetDir2(ev);
      const auto& seg = Defs::kEdgeSegments[l.neigh_dir];
      l.pixel_offset = seg.bit_num + num_stride * seg.bit_stride;
    }
  }

  // Computes the clockwise next direction from a given direction. (N>E>S>W>N);
  static Direction IncDir(Direction dir) {
    return (Direction)((((int)dir) + 1) % 4);
  }
  // Computes the clockwise previous direction from a given direction. (N<E<S<W<N);
  static Direction DecDir(Direction dir) {
    return (Direction)((((int)dir) - 1) % 4);
  }

  // No menu entry selected.
  static constexpr unsigned kMenuNone = 31;

  // Turns off iteration.
  static constexpr unsigned kMenuStop = 0;
  // Turns on iteration.
  static constexpr unsigned kMenuStart = 1;
  // One iteration step.
  static constexpr unsigned kMenuStep = 2;
  // Random board
  static constexpr unsigned kMenuRandom = 3;

  // Evoution speed = 250 msec
  static constexpr unsigned kMenuSpeedFast = 4;
  // Evoution speed = 500 msec
  static constexpr unsigned kMenuSpeedMid = 5;
  // Evoution speed = 1000 msec
  static constexpr unsigned kMenuSpeedSlow = 6;
  // Evoution speed = 10000 msec
  static constexpr unsigned kMenuSpeedVerySlow = 7;

  // Pressing a button will add a glider in SE.
  static constexpr unsigned kAddGliderSE = 8;
  static constexpr unsigned kAddGliderNE = 9;
  static constexpr unsigned kAddGliderNW = 10;
  static constexpr unsigned kAddGliderSW = 11;

  // Pressing a button will add a glider in random direction.
  static constexpr unsigned kAddRandomGlider = 12;

  // Pressing a button will add three horizontal bits from the given place.
  static constexpr unsigned kAddTrio = 13;

  // After this: every button pressed will add that bit to the pattern.
  static constexpr unsigned kSetBit = 14;

  // Switch between first and second menu page.
  static constexpr unsigned kSecondMenu = 15;
  

  // SECOND MENU PAGE

  // Clear the entire board.
  //  static constexpr unsigned kMenuEmpty = 16;

  // Clear table and stop
  static constexpr unsigned kMenuClearAndStop = 16;
  // Put neighbor debug pattern up
  static constexpr unsigned kMenuDebugXY = 17;
  // Put neighbor debug pattern up
  static constexpr unsigned kMenuDebugNeighbor = 18;
  // Reinit all other nodes
  static constexpr unsigned kMenuRequestReInit = 19;

  

  void ExitMenu() {
    menu_active_ = 0;
    menu_selected_ = kMenuNone;
  }

  void ExecuteButton(int x, int y, unsigned btn) {
    if (menu_selected_ == kMenuNone) {
      menu_selected_ = btn;
      if (menu_second_page_) {
        menu_selected_ += 16;
      }
      switch (menu_selected_) {
        // These items are after the first press of the menu button.
        case kSecondMenu:
        case kSecondMenu + 16:
          menu_second_page_ ^= 1;
          // We want another menu button press.
          menu_selected_ = kMenuNone;
          return;
        case kMenuStop:
          iface_->SendEvent(Defs::CreateEvent(Defs::kGlobalCmd, 0, 0, Defs::kStopIteration));
          return ExitMenu();
        case kMenuStart:
          iface_->SendEvent(Defs::CreateEvent(Defs::kGlobalCmd, 0, 0, Defs::kStartIteration));
          return ExitMenu();
        case kMenuStep:
          iface_->SendEvent(Defs::CreateEvent(Defs::kGlobalCmd, 0, 0, Defs::kEvolveAndReport));
          return ExitMenu();
        case kMenuRandom:
          iface_->SendEvent(Defs::CreateEvent(Defs::kGlobalCmd, 0, 0, Defs::kSetStateRandom));
          return ExitMenu();
        case kMenuSpeedFast:
          evolution_speed_msec_ = 250;
          return ExitMenu();
        case kMenuSpeedMid:
          evolution_speed_msec_ = 500;
          return ExitMenu();
        case kMenuSpeedSlow:
          evolution_speed_msec_ = 1000;
          return ExitMenu();
        case kMenuSpeedVerySlow:
          evolution_speed_msec_ = 10000;
          return ExitMenu();
        case kMenuRequestReInit:
          iface_->SendEvent(Defs::CreateEvent(Defs::kGlobalCmd, 0, 0, Defs::kReInit));
          return ExitMenu();
        case kMenuClearAndStop:
          iface_->SendEvent(Defs::CreateEvent(Defs::kGlobalCmd, 0, 0, Defs::kClearState));
          iface_->SendEvent(Defs::CreateEvent(Defs::kGlobalCmd, 0, 0, Defs::kStopIteration));
          return ExitMenu();
        case kMenuDebugXY:
          iface_->SendEvent(Defs::CreateEvent(Defs::kGlobalCmd, 0, 0, Defs::kPrintXY));
          return ExitMenu();
        case kMenuDebugNeighbor:
          iface_->SendEvent(Defs::CreateEvent(Defs::kGlobalCmd, 0, 0, Defs::kClearState));
          iface_->SendEvent(Defs::CreateEvent(Defs::kGlobalCmd, 0, 0, Defs::kStopIteration));
          iface_->SendEvent(Defs::CreateEvent(Defs::kGlobalCmd, 0, 0, Defs::kPrintNeighborDebug));
          return ExitMenu();
      }
    } else {
      switch (menu_selected_) {
        // These items do something with the x-y after a given menu entry was
        // selected.
        case kSetBit:
          AddBit(x, y, 0, 0, btn);
          return;
        case kAddTrio:
          AddBit(x, y, 0, 0, btn);
          AddBit(x, y, 1, 0, btn);
          AddBit(x, y, 2, 0, btn);
          return;
        case kAddRandomGlider:
          AddGlider(rand() % 4, x, y, btn);
          return;
        case kAddGliderSE:
          AddGlider(0, x, y, btn);
          return;
        case kAddGliderNE:
          AddGlider(1, x, y, btn);
          return;
        case kAddGliderNW:
          AddGlider(2, x, y, btn);
          return;
        case kAddGliderSW:
          AddGlider(3, x, y, btn);
          return;
        default:
          return ExitMenu();
      }
    }
  }

  /// Sends out a message to add one bit to the board.
  /// @param x board address x
  /// @param y board address y
  /// @param btn what is the origin (0..15, row/col)
  /// @param dx where compared to the origin (plus: east)
  /// @param dy where compared to the origin (plus:south)
  void AddBit(uint8_t x, uint8_t y, int dx, int dy, unsigned btn) {
    int r = (btn / 4) + dy;
    int c = (btn % 4) + dx;
    while (r >= 4) {
      y++;
      r -= 4;
    }
    while (r < 0) {
      y--;
      r += 4;
    }
    while (c >= 4) {
      x++;
      c -= 4;
    }
    while (c < 0) {
      x--;
      c += 4;
    }
    iface_->SendEvent(Defs::CreateEvent(Defs::kStateOr, x, y, 1u << ((r * 4) + c)));
  }

  /// Adds a glider. x,y is a board, btn is a button where the glider
  /// originates.
  /// dir is a direction, 0 is SE.
  void AddGlider(uint8_t dir, uint8_t x, uint8_t y, unsigned btn) {
    AddBit(x, y, 0, 0, btn);
    switch (dir & 3) {
      case 0:
        AddBit(x, y, 1, 0, btn);
        AddBit(x, y, 2, 0, btn);
        AddBit(x, y, 2, -1, btn);
        AddBit(x, y, 1, -2, btn);
        return;
      case 1:
        AddBit(x, y, 0, -1, btn);
        AddBit(x, y, 0, -2, btn);
        AddBit(x, y, -1, -2, btn);
        AddBit(x, y, -2, -1, btn);
        return;
      case 2:
        AddBit(x, y, -1, 0, btn);
        AddBit(x, y, -2, 0, btn);
        AddBit(x, y, -2, 1, btn);
        AddBit(x, y, -1, 2, btn);
        return;
      case 3:
        AddBit(x, y, 0, 1, btn);
        AddBit(x, y, 0, 2, btn);
        AddBit(x, y, 1, 2, btn);
        AddBit(x, y, 2, 1, btn);
        return;
    }
  }

  static constexpr inline uint32_t rotl32(uint32_t x, uint32_t n) {
    return (x << n) | (x >> (32 - n));
  }

  // Fixed implementation of murmur3-32bit for 4 byte data length.
  uint32_t Murmur3(uint32_t data, uint32_t seed) {
    static constexpr uint32_t c1 = 0xcc9e2d51;
    static constexpr uint32_t c2 = 0x1b873593;
    static constexpr unsigned r1 = 15;
    static constexpr unsigned r2 = 13;
    static constexpr unsigned m = 5;
    static constexpr uint32_t n = 0xe6546b64;
    uint32_t hash = seed;

    uint32_t k = data;
    k *= c1;
    k = rotl32(k, r1);
    k *= c2;

    hash ^= k;
    hash = rotl32(hash, r2);
    hash = hash * m + n;

    hash = hash ^ 4;
    hash = hash ^ (hash >> 16);
    hash = hash * 0x85ebca6b;
    hash = hash ^ (hash >> 13);
    hash = hash * 0xc2b2ae35;
    hash = hash ^ (hash >> 16);

    return hash;
  }

  using NodeCoord = uint16_t;

  static constexpr uint8_t GetCoordX(const NodeCoord& c) {
    return c & 0xff;
  }
  static constexpr uint8_t GetCoordY(const NodeCoord& c) {
    return (c >> 8) & 0xff;
  }
  static constexpr NodeCoord GetCoord(uint8_t x, uint8_t y) {
    return (uint16_t(y) << 8) | x;
  }

  ProtocolEngineInterface* iface_;

  static constexpr uint8_t INVALID_COORD = 0xff;
  // Coordinates to be assigned via a local signal. This is already adjusted from the neighbor with deltaxy.
  uint8_t pending_x_{ INVALID_COORD };
  uint8_t pending_y_{ INVALID_COORD };
  // New coordinates being assigned by the leader.
  uint8_t leader_pending_x_{ INVALID_COORD };
  uint8_t leader_pending_y_{ INVALID_COORD };

  // Which neighbor was recently instructed to do a local trigger.
  uint8_t pending_neigh_x_{ INVALID_COORD };
  uint8_t pending_neigh_y_{ INVALID_COORD };
  // Which direction on the neighbor that was triggered.
  Direction pending_neigh_dir_;

  // My assigned coordinates.
  uint8_t my_x_{ INVALID_COORD };
  uint8_t my_y_{ INVALID_COORD };

public:
  struct Link {
    // Coordinates of the neighbor
    uint8_t neigh_x{ INVALID_COORD };
    uint8_t neigh_y{ INVALID_COORD };
    // Direction on the neighbor side
    Direction neigh_dir;
    // For corner links (NE, SE, SW, NW), this index pinpoints which bit we are
    // talking about. 0..15.
    uint8_t pixel_offset{ 16 };
    // @return true if this link is set up.
    bool valid() {
      return neigh_x != INVALID_COORD;
    }
  };
  // Neighbor's assigned coordinates.
  std::vector<Link> neighbors_;

private:
  // millis() tick when we start the leader election after startup
  uint32_t election_start_timeout_{ INVALID_TIMEOUT };

  // millis() tick when we claim the bus is idle.
  uint32_t idle_timeout_{ INVALID_TIMEOUT };

  static constexpr uint32_t INVALID_TIMEOUT = (uint32_t)-1;
  // millis() value when we should complete the current operation (marked by to_* variables)
  uint32_t timeout_{ INVALID_TIMEOUT };

  // Smallest alias that we know can be a leader.
  uint16_t leader_alias_;

  // This value in next_neighbor_report_ is invalid.
  static constexpr unsigned INVALID_DIR = 4;
  // If this is 0..3, then a neighbor report needs to be emitted.
  unsigned next_neighbor_report_ : 3;
  // Which directions have a local signal active right now. Used for edge detection.
  unsigned known_local_signal_ : 4;
  // true if we have not yet sent out the init done event.
  bool need_init_done_ : 1;
  // true if we've seen a leader node.
  bool seen_leader_ : 1;
  // true if this node is the leader.
  bool is_leader_ : 1;

  // to_*: what to do when we are done with the timeout.
  bool to_cancel_local_signal_ : 1;
  bool to_leader_election_ : 1;
  bool to_disc_neighbor_lookup_ : 1;

  // ==== Leader-only state ====

  // Ongoing discovery state.
  DiscoveryState disc_state_{ kNotRunning };

  // We've seen some nodes rebooting, do a discovery for them.
  bool need_partial_discovery_ : 1;
  // We need to do a discovery from the beginning.
  bool need_full_discovery_ : 1;
  // True if we sent out the toggle neighbors command at the beginning of a catchup discovery, but have not yet gotten all responses back.
  bool disc_catchup_pending_ : 1;
  // True if we should be emitting tick messages.
  bool run_tick_ : 1;
  // Whether we want steady state and dead-field detection.
  bool detect_steady_state_ : 1;
  // True if in the current iteration we've seen a nonzero state.
  bool seen_non_zero_ : 1;
  // how many consecutive states we've seen that seems uninteresting.
  unsigned num_bad_states_ : 4;

  // menu is active
  bool menu_active_ : 1;
  // true if we are on the second page of the menu.
  bool menu_second_page_ : 1;
  // What menu item is selected right now.
  uint8_t menu_selected_ : 6;

  // Last direction we queried for the node in the head of the queue. -1 .. 3.
  int disc_neighbor_dir_ : 4;
  // When should we emit the next evolution tick.
  uint32_t tick_timeout_;
  uint32_t evolution_speed_msec_{ Defs::kDefaultEvolutionSpeedMsec };

  // Timeout used for discovery operations.
  uint32_t disc_timeout_;

  // Nodes that we need to go through neightbor discovery with.
  std::queue<NodeCoord> disq_;
  struct NodeInfo {
    uint16_t alias_;
  };
  std::map<NodeCoord, NodeInfo> known_nodes_;

  // Hash values of recent history. Not implemented yet.
  //std::vector<uint64_t> hash_history_;
  uint64_t curr_hash_;
  uint64_t last_hash_[3];
};  // class ProtocolEngine

#endif  // _PROTOCOL_ENGINE_H_
