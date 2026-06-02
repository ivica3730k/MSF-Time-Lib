// Host-side tests for MSF-Time-Lib. See README in this directory for how to
// build & run.
//
// The library is a header-only template that depends on Arduino.h. We compile
// against a local Arduino.h shim (test/Arduino.h) that provides a controllable
// mock clock. To exercise the private helpers directly we apply the standard
// "#define private public" idiom — gross but contained.

#include "Arduino.h"
#include "test_runner.h"

#define private public
#define protected public
#include "../src/MSF-Time-Lib.h"
#undef private
#undef protected

// =============================================================================
// Shared fixtures
// =============================================================================

namespace {

// Stub reader for unit tests that never actually exercise the carrier loop.
bool stub_reader() { return true; }

// Synthesized MSF signal used by the integration test. Bit A and B values are
// indexed by second-of-minute (0..59). The carrier reader generates an MSF
// waveform on the fly from these arrays plus the current mock clock value.
struct SignalSim {
  bool A[60];
  bool B[60];
};
SignalSim g_sim;

// Generates the carrier state for the current mock_millis. The broadcast is
// periodic with a 60-second cycle aligned to mock_millis % 60000.
//
//   - Second 0 (minute marker): 500ms silence, then 500ms carrier.
//   - Seconds 1..59 (standard): 100ms silence start mark, 100ms A bit,
//     100ms B bit, 700ms carrier. Carrier-off during the bit slot encodes a 1.
bool sim_reader() {
  uint32_t t = msf_test::mock_millis_value;
  uint32_t s = (t / 1000) % 60;
  uint32_t ms = t % 1000;
  if (s == 0) {
    return ms >= 500;
  }
  if (ms < 100) return false;
  if (ms < 200) return !g_sim.A[s];
  if (ms < 300) return !g_sim.B[s];
  return true;
}

// Populates g_sim with the encoded bits for 2024-12-31 23:59 (Tuesday).
// Returns the expected MSFData so tests can compare.
void encode_reference_signal() {
  std::memset(g_sim.A, 0, sizeof(g_sim.A));
  std::memset(g_sim.B, 0, sizeof(g_sim.B));

  // Year 24 (weights {80,40,20,10,8,4,2,1} at A bits 17..24): 24 = 20 + 4
  g_sim.A[19] = true;
  g_sim.A[22] = true;
  // Month 12 (weights {10,8,4,2,1} at A bits 25..29): 12 = 10 + 2
  g_sim.A[25] = true;
  g_sim.A[28] = true;
  // Day 31 (weights {20,10,8,4,2,1} at A bits 30..35): 31 = 20 + 8 + 2 + 1
  g_sim.A[30] = true;
  g_sim.A[32] = true;
  g_sim.A[34] = true;
  g_sim.A[35] = true;
  // Day of week raw = 2 (Tuesday in MSF's 0=Sunday encoding) at A bits 36..38
  g_sim.A[37] = true;
  // Hour 23 (weights {20,10,8,4,2,1} at A bits 39..44): 23 = 20 + 2 + 1
  g_sim.A[39] = true;
  g_sim.A[43] = true;
  g_sim.A[44] = true;
  // Minute 59 (weights {40,20,10,8,4,2,1} at A bits 45..51): 59 = 40+10+8+1
  g_sim.A[45] = true;
  g_sim.A[47] = true;
  g_sim.A[48] = true;
  g_sim.A[51] = true;

  // Odd parities: A_ones + B_parity_bit must be odd
  g_sim.B[54] = true;   // Year: 2 ones in A -> parity 1
  g_sim.B[55] = true;   // Date: 6 ones in A -> parity 1
  g_sim.B[56] = false;  // DOW: 1 one in A -> parity 0
  g_sim.B[57] = false;  // Time: 7 ones in A -> parity 0
}

}  // namespace

// =============================================================================
// Unit tests — pure bit manipulation
// =============================================================================

TEST_CASE(writeBit_sets_individual_bits_within_byte) {
  MSFReceiver<10> r(stub_reader);
  uint8_t arr[2] = {0, 0};
  r.writeBit(arr, 0, true);
  ASSERT_EQ(0x01, arr[0]);
  r.writeBit(arr, 7, true);
  ASSERT_EQ(0x81, arr[0]);
  r.writeBit(arr, 8, true);
  ASSERT_EQ(0x01, arr[1]);
}

TEST_CASE(writeBit_clears_bits_without_disturbing_neighbours) {
  MSFReceiver<10> r(stub_reader);
  uint8_t arr[1] = {0xFF};
  r.writeBit(arr, 3, false);
  ASSERT_EQ(0xF7, arr[0]);
  r.writeBit(arr, 0, false);
  ASSERT_EQ(0xF6, arr[0]);
}

TEST_CASE(readBit_returns_value_of_individual_bits) {
  MSFReceiver<10> r(stub_reader);
  const uint8_t arr[2] = {0xAA, 0x55};
  ASSERT_FALSE(r.readBit(arr, 0));
  ASSERT_TRUE(r.readBit(arr, 1));
  ASSERT_FALSE(r.readBit(arr, 2));
  ASSERT_TRUE(r.readBit(arr, 3));
  ASSERT_TRUE(r.readBit(arr, 8));
  ASSERT_FALSE(r.readBit(arr, 9));
}

TEST_CASE(writeBit_then_readBit_round_trips) {
  MSFReceiver<10> r(stub_reader);
  uint8_t arr[8] = {0};
  for (int i = 0; i < 60; i++) {
    r.writeBit(arr, i, (i % 3) == 0);
  }
  for (int i = 0; i < 60; i++) {
    ASSERT_EQ(((i % 3) == 0) ? 1 : 0, r.readBit(arr, i) ? 1 : 0);
  }
}

// =============================================================================
// Unit tests — BCD decoding
// =============================================================================

TEST_CASE(decodeBCD_decodes_year_value) {
  MSFReceiver<10> r(stub_reader);
  std::memset(r.packedABits, 0, sizeof(r.packedABits));
  // 24 = weights[2] (20) + weights[5] (4)
  r.writeBit(r.packedABits, 19, true);
  r.writeBit(r.packedABits, 22, true);
  static const int wYear[] = {80, 40, 20, 10, 8, 4, 2, 1};
  ASSERT_EQ(24, r.decodeBCD(17, 8, wYear));
}

TEST_CASE(decodeBCD_decodes_minute_value) {
  MSFReceiver<10> r(stub_reader);
  std::memset(r.packedABits, 0, sizeof(r.packedABits));
  // 59 = 40 + 10 + 8 + 1
  r.writeBit(r.packedABits, 45, true);
  r.writeBit(r.packedABits, 47, true);
  r.writeBit(r.packedABits, 48, true);
  r.writeBit(r.packedABits, 51, true);
  static const int wMin[] = {40, 20, 10, 8, 4, 2, 1};
  ASSERT_EQ(59, r.decodeBCD(45, 7, wMin));
}

TEST_CASE(decodeBCD_decodes_max_hour) {
  MSFReceiver<10> r(stub_reader);
  std::memset(r.packedABits, 0, sizeof(r.packedABits));
  // 23 = 20 + 2 + 1
  r.writeBit(r.packedABits, 39, true);
  r.writeBit(r.packedABits, 43, true);
  r.writeBit(r.packedABits, 44, true);
  static const int wHour[] = {20, 10, 8, 4, 2, 1};
  ASSERT_EQ(23, r.decodeBCD(39, 6, wHour));
}

TEST_CASE(decodeBCD_returns_zero_when_no_bits_set) {
  MSFReceiver<10> r(stub_reader);
  std::memset(r.packedABits, 0, sizeof(r.packedABits));
  static const int wHour[] = {20, 10, 8, 4, 2, 1};
  ASSERT_EQ(0, r.decodeBCD(39, 6, wHour));
}

// =============================================================================
// Unit tests — parity
// =============================================================================

TEST_CASE(checkParity_one_data_bit_zero_parity_passes) {
  MSFReceiver<10> r(stub_reader);
  std::memset(r.packedABits, 0, sizeof(r.packedABits));
  std::memset(r.packedBBits, 0, sizeof(r.packedBBits));
  r.writeBit(r.packedABits, 17, true);
  // 1 one + parity 0 = 1 (odd) -> passes
  ASSERT_TRUE(r.checkParity(17, 8, 54));
}

TEST_CASE(checkParity_two_data_bits_one_parity_passes) {
  MSFReceiver<10> r(stub_reader);
  std::memset(r.packedABits, 0, sizeof(r.packedABits));
  std::memset(r.packedBBits, 0, sizeof(r.packedBBits));
  r.writeBit(r.packedABits, 17, true);
  r.writeBit(r.packedABits, 19, true);
  r.writeBit(r.packedBBits, 54, true);
  // 2 ones + parity 1 = 3 (odd) -> passes
  ASSERT_TRUE(r.checkParity(17, 8, 54));
}

TEST_CASE(checkParity_two_data_bits_zero_parity_fails) {
  MSFReceiver<10> r(stub_reader);
  std::memset(r.packedABits, 0, sizeof(r.packedABits));
  std::memset(r.packedBBits, 0, sizeof(r.packedBBits));
  r.writeBit(r.packedABits, 17, true);
  r.writeBit(r.packedABits, 19, true);
  // 2 ones + parity 0 = 2 (even) -> fails
  ASSERT_FALSE(r.checkParity(17, 8, 54));
}

TEST_CASE(checkParity_all_zero_fails) {
  MSFReceiver<10> r(stub_reader);
  std::memset(r.packedABits, 0, sizeof(r.packedABits));
  std::memset(r.packedBBits, 0, sizeof(r.packedBBits));
  // 0 ones (even) -> fails (MSF spec mandates odd)
  ASSERT_FALSE(r.checkParity(17, 8, 54));
}

// =============================================================================
// Unit tests — rolling buffer state machine
// =============================================================================

TEST_CASE(rollingBufferSetupAndCleanup_initializes_state) {
  MSFReceiver<10> r(stub_reader);
  // Dirty the state first to make sure setup actually resets it.
  r.rollingBufferHead = 999;
  r.rollingBufferSilenceWindowScore = 999;
  r.rollingBufferCarrierWindowScore = -1;

  r.rollingBufferSetupAndCleanup();

  ASSERT_EQ(0, r.rollingBufferHead);
  ASSERT_EQ(0, r.rollingBufferSilenceWindowScore);
  // Buffer is filled with 0xFF (all carrier), so the initial carrier score
  // equals the full carrier window size.
  ASSERT_EQ(700 / 10, r.rollingBufferCarrierWindowScore);
}

TEST_CASE(updateRollingBuffer_peaks_on_minute_marker_pattern) {
  // SAMPLE_RATE_MS=10 -> carrier window 70 samples, silence window 50 samples.
  // Peak score = 70 + 50 = 120.
  MSFReceiver<10> r(stub_reader);
  r.rollingBufferSetupAndCleanup();

  // Flush the initial 0xFF-filled buffer so its state reflects real input.
  // Buffer holds 150 samples; 150 pushes of carrier guarantees a clean slate.
  for (int i = 0; i < 150; i++) {
    r.updateRollingBuffer(true);
  }

  // Push 70 carrier samples — these will land in the carrier window.
  for (int i = 0; i < 70; i++) {
    r.updateRollingBuffer(true);
  }

  // Push 50 silence samples — fills the silence window. Track peak.
  int peakSeen = 0;
  for (int i = 0; i < 50; i++) {
    int score = r.updateRollingBuffer(false);
    if (score > peakSeen) peakSeen = score;
  }

  ASSERT_EQ(120, peakSeen);
}

TEST_CASE(updateRollingBuffer_score_drops_after_marker_passes) {
  MSFReceiver<10> r(stub_reader);
  r.rollingBufferSetupAndCleanup();
  for (int i = 0; i < 150; i++) r.updateRollingBuffer(true);
  for (int i = 0; i < 70; i++) r.updateRollingBuffer(true);
  for (int i = 0; i < 50; i++) r.updateRollingBuffer(false);
  // Now push more carrier samples — silence window starts losing silence,
  // score should drop below the peak.
  int after = r.updateRollingBuffer(true);
  ASSERT(after < 120);
}

// =============================================================================
// Integration test — end-to-end get_time() against a synthesized signal
// =============================================================================

TEST_CASE(get_time_decodes_synthesized_signal) {
  msf_test::reset_mock();
  encode_reference_signal();

  MSFReceiver<10> msf(sim_reader);
  MSFData data = msf.get_time();

  ASSERT_EQ(2024, (int)data.year);
  ASSERT_EQ(12, data.month);
  ASSERT_EQ(31, data.day);
  ASSERT_EQ(23, data.hour);
  ASSERT_EQ(59, data.minute);
  ASSERT_EQ(0, data.second);
  ASSERT_EQ(3, data.dayOfTheWeek);  // MSF raw 2 (Tuesday) + 1 offset
  ASSERT_TRUE(data.checksumPassed);
}

TEST_CASE(get_time_reports_checksum_failure_when_year_parity_corrupted) {
  msf_test::reset_mock();
  encode_reference_signal();
  // Flip the year parity bit so parity becomes even (fails odd-parity check).
  g_sim.B[54] = false;

  MSFReceiver<10> msf(sim_reader);
  MSFData data = msf.get_time();

  // Time fields still decode correctly; only the checksum should fail.
  ASSERT_EQ(2024, (int)data.year);
  ASSERT_EQ(59, data.minute);
  ASSERT_FALSE(data.checksumPassed);
}

// =============================================================================
// main
// =============================================================================

int main() { return msf_test::run_all_tests(); }
