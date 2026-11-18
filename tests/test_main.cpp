#include "test_framework.hpp"

int main() {
  int failed = 0;
  for (const auto& c : sdt::test::registry()) {
    const int before = sdt::test::failures();
    c.fn();
    const bool ok = sdt::test::failures() == before;
    std::printf("[%s] %s\n", ok ? " OK " : "FAIL", c.name);
    if (!ok) ++failed;
  }
  std::printf("\n%zu tests, %d failed\n", sdt::test::registry().size(), failed);
  return failed == 0 ? 0 : 1;
}
