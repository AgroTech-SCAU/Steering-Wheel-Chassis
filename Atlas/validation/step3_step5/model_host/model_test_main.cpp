#include <gtest/gtest.h>
int main() {
  unsigned failed = 0;
  for (const auto & test : host_tests()) {
    try { test.second(); std::cout << "PASS " << test.first << '\n'; }
    catch (const std::exception & e) { ++failed; std::cerr << "FAIL " << test.first << " " << e.what() << '\n'; }
  }
  std::cout << host_tests().size() << " tests, " << failed << " failed\n";
  return failed ? 1 : 0;
}
