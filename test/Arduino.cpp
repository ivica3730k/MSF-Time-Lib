#include "Arduino.h"

namespace msf_test {
uint32_t mock_millis_value = 1000;
uint32_t random_return_value = 1000;

void reset_mock() {
  mock_millis_value = 1000;
  random_return_value = 1000;
}
}  // namespace msf_test

SerialStub Serial;
