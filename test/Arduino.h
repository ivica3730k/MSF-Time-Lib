#pragma once

// Minimal host-side mock of Arduino.h used to exercise MSFReceiver on a
// regular computer (no MCU). It only provides what MSF-Time-Lib.h touches.

#include <cstdint>
#include <cstring>

namespace msf_test {
// Mock clock backing millis(). Tests advance it explicitly or rely on
// auto-advance from millis()/delay()/delayMicroseconds() below.
extern uint32_t mock_millis_value;

// Value returned by mocked random(). Kept constant so behaviour is
// deterministic across runs.
extern uint32_t random_return_value;

// Resets mock state. Call between tests.
void reset_mock();
}  // namespace msf_test

// MSFReceiver's busy-wait loops call millis() with no delay in the body. To
// guarantee forward progress in tests, every call to millis() advances the
// mock clock by 1ms. Tests that need precise control set mock_millis_value
// directly and avoid reading millis() between writes.
inline uint32_t millis() {
  uint32_t v = msf_test::mock_millis_value;
  msf_test::mock_millis_value += 1;
  return v;
}

inline void delay(uint32_t ms) { msf_test::mock_millis_value += ms; }

inline void delayMicroseconds(uint32_t us) {
  // Round down to ms. Sub-millisecond delays do not advance the mock clock;
  // forward progress in tight loops comes from millis() auto-advance.
  if (us >= 1000) msf_test::mock_millis_value += us / 1000;
}

inline long random(long lower, long upper) {
  (void)lower;
  (void)upper;
  return static_cast<long>(msf_test::random_return_value);
}

// Arduino's F() wraps string literals in PROGMEM. On host we just pass through.
#define F(x) (x)

// Stubbed Serial so the library compiles if MSF_TIME_LIB_DEBUG is enabled.
struct SerialStub {
  template <typename T>
  void print(const T&) {}
  template <typename T>
  void println(const T&) {}
  void println() {}
};
extern SerialStub Serial;
