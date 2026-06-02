#pragma once

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <vector>

// Tiny self-contained test harness. Avoids pulling in an external framework
// so the test suite stays a single Make-and-run.
//
// Usage:
//   TEST_CASE(my_test) {
//     ASSERT(some_condition);
//     ASSERT_EQ(expected, actual);
//   }
//
// Each TEST_CASE auto-registers via a static initializer. Call
// msf_test::run_all_tests() in main().

namespace msf_test {

struct TestCase {
  const char* name;
  void (*fn)();
};

inline std::vector<TestCase>& registry() {
  static std::vector<TestCase> r;
  return r;
}

inline int& current_failure_count() {
  static int n = 0;
  return n;
}

inline int register_test(const char* name, void (*fn)()) {
  registry().push_back({name, fn});
  return 0;
}

inline int run_all_tests() {
  int passed = 0;
  int failed = 0;
  for (const auto& tc : registry()) {
    current_failure_count() = 0;
    std::printf("[RUN ] %s\n", tc.name);
    tc.fn();
    if (current_failure_count() == 0) {
      std::printf("[PASS] %s\n", tc.name);
      passed++;
    } else {
      std::printf("[FAIL] %s (%d assertion failures)\n", tc.name, current_failure_count());
      failed++;
    }
  }
  std::printf("\n%d passed, %d failed (out of %d)\n", passed, failed, passed + failed);
  return failed == 0 ? 0 : 1;
}

}  // namespace msf_test

#define MSF_TEST_CONCAT_INNER(a, b) a##b
#define MSF_TEST_CONCAT(a, b) MSF_TEST_CONCAT_INNER(a, b)

#define TEST_CASE(name)                                                  \
  static void MSF_TEST_CONCAT(test_fn_, name)();                         \
  static int MSF_TEST_CONCAT(test_reg_, name) =                          \
      ::msf_test::register_test(#name, MSF_TEST_CONCAT(test_fn_, name)); \
  static void MSF_TEST_CONCAT(test_fn_, name)()

#define ASSERT(cond)                                                          \
  do {                                                                        \
    if (!(cond)) {                                                            \
      std::printf("    FAIL %s:%d  ASSERT(%s)\n", __FILE__, __LINE__, #cond); \
      ::msf_test::current_failure_count()++;                                  \
      return;                                                                 \
    }                                                                         \
  } while (0)

#define ASSERT_EQ(expected, actual)                                           \
  do {                                                                        \
    auto _e = (expected);                                                     \
    auto _a = (actual);                                                       \
    if (!(_e == _a)) {                                                        \
      std::printf(                                                            \
          "    FAIL %s:%d  ASSERT_EQ(%s, %s)\n      expected=%lld\n"          \
          "      actual  =%lld\n",                                            \
          __FILE__, __LINE__, #expected, #actual, static_cast<long long>(_e), \
          static_cast<long long>(_a));                                        \
      ::msf_test::current_failure_count()++;                                  \
      return;                                                                 \
    }                                                                         \
  } while (0)

#define ASSERT_TRUE(cond) ASSERT(cond)
#define ASSERT_FALSE(cond) ASSERT(!(cond))
