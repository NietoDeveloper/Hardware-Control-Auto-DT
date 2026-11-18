#pragma once
// Tiny dependency-free test harness (keeps the repo hermetic: no network at configure time).
#include <cmath>
#include <cstdio>
#include <vector>

namespace sdt::test {
struct Case {
  const char* name;
  void (*fn)();
};
inline std::vector<Case>& registry() {
  static std::vector<Case> r;
  return r;
}
inline int& failures() {
  static int f = 0;
  return f;
}
struct Registrar {
  Registrar(const char* n, void (*f)()) { registry().push_back({n, f}); }
};
}  // namespace sdt::test

#define SDT_TEST(name)                                           \
  static void name();                                            \
  static ::sdt::test::Registrar registrar_##name(#name, &name);  \
  static void name()

#define CHECK(cond)                                                                      \
  do {                                                                                   \
    if (!(cond)) {                                                                       \
      std::fprintf(stderr, "  CHECK failed %s:%d: %s\n", __FILE__, __LINE__, #cond);     \
      ++::sdt::test::failures();                                                         \
    }                                                                                    \
  } while (0)

#define CHECK_NEAR(a, b, eps) CHECK(std::fabs(static_cast<double>(a) - static_cast<double>(b)) <= (eps))
