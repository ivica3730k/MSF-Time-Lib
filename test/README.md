# Host tests for MSF-Time-Lib

These tests run on a plain C++ compiler — no Arduino board required. They
exist mainly to give the upcoming refactoring work a safety net.

## Running

```
make test
```

That builds `run_tests` and executes it. `make clean` removes the binary.

## How it works

The library is a header-only template that includes `<Arduino.h>`. The test
binary compiles against:

- `Arduino.h` / `Arduino.cpp` — a tiny host-side shim that provides `millis()`,
  `delay()`, `delayMicroseconds()`, `random()`, and a stub `Serial`. The clock
  backing `millis()` is the global `msf_test::mock_millis_value`. Every call to
  `millis()` auto-advances it by 1ms so the library's busy-wait loops make
  forward progress.
- `test_runner.h` — a minimal `TEST_CASE` / `ASSERT*` harness (no external
  dependencies).
- `test_main.cpp` — the test cases themselves.

Private helpers (`writeBit`, `readBit`, `decodeBCD`, `checkParity`,
`updateRollingBuffer`) are accessed via the `#define private public` idiom
before including the library.

## What's covered

- **Bit packing** (`writeBit` / `readBit`): set/clear/round-trip across byte
  boundaries.
- **BCD decoding**: year, hour, minute decoded from known bit patterns.
- **Parity**: odd-parity passes/fails for the four MSF parity groups.
- **Rolling buffer**: initial state after setup; score peaks at the
  700ms-carrier + 500ms-silence minute-marker pattern and drops once it
  passes.
- **End-to-end `get_time()`**: a synthesized 60-second MSF waveform is fed to
  the receiver and the decoded `MSFData` is checked field-by-field. A
  follow-up test flips the year parity bit and confirms `checksumPassed`
  flips to `false`.

## CI

`.github/workflows/run-tests-on-pr.yaml` runs `make test` on every PR to
`main`.
