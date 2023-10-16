#ifndef _PROTOCOL_ENGINE_DEFAULT_PLUGIN_H_
#define _PROTOCOL_ENGINE_DEFAULT_PLUGIN_H_

#include "protocol-engine.h"
#include "global-bus.h"
#include "local-bus.h"

class ProtocolEngineIfImpl : public ProtocolEngineInterface {
  bool SendEvent(uint64_t event_id) override {
    return ::SendEvent(event_id);
  }

  // @return the currently used alias of the local node.
  uint16_t GetAlias() override {
    if (openlcb::state_.init_state != openlcb::INITIALIZED) {
      return 0;
    }
    return openlcb::state_.alias;
  }

  void LocalBusSignal(Direction dir, bool active) {
    ::LocalBusSignal(dir, active);
  }
  bool LocalBusIsActive(Direction dir) override {
    return ::LocalBusIsActive(dir);
  };

  uint32_t millis() override {
    return HAL_GetTick();
  }

  bool TxPending() override {
    return can_tx_busy();
  }

  void Reboot() override {
    HAL_NVIC_SystemReset();
  }

} global_impl;

#endif // _PROTOCOL_ENGINE_DEFAULT_PLUGIN_H_