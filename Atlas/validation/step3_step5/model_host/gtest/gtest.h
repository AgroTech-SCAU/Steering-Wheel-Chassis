// Host-only assertion adapter for running existing model tests without ROS/gtest
#pragma once
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
inline std::vector<std::pair<std::string, std::function<void()>>> & host_tests() {
  static std::vector<std::pair<std::string, std::function<void()>>> tests;
  return tests;
}
#define TEST(suite, name) void suite##_##name(); \
  static const bool suite##_##name##_registered = [] { \
    host_tests().emplace_back(#suite "." #name, suite##_##name); return true; }(); \
  void suite##_##name()
#define HOST_CHECK(condition) do { if (!(condition)) \
  throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__) + " " #condition); } while (false)
#define ASSERT_TRUE(x) HOST_CHECK(x)
#define EXPECT_TRUE(x) HOST_CHECK(x)
#define EXPECT_FALSE(x) HOST_CHECK(!(x))
#define ASSERT_EQ(x,y) HOST_CHECK((x) == (y))
#define EXPECT_EQ(x,y) HOST_CHECK((x) == (y))
#define ASSERT_GT(x,y) HOST_CHECK((x) > (y))
#define ASSERT_LT(x,y) HOST_CHECK((x) < (y))
#define EXPECT_GE(x,y) HOST_CHECK((x) >= (y))
