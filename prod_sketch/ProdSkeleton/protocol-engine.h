
class ProtocolEngine {
public:
  // Call this function once from setup().
  void Setup(std::function<void(uint64_t)> event_send_fn) {
    event_send_fn_ = event_send_fn;
  }

  // Call this function from the loop() handler.
  void Loop() {}

  // Call this function when a global event arrives.
  void OnGlobalEvent(int64_t ev, uint16_t src) {}


private:
  std::function<void(uint64_t)> event_send_fn_;
};  // class ProtocolEngine



void ProtocolEngine::