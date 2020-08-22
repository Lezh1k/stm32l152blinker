#include <gtest/gtest.h>
#include <stdint.h>
#include <vector>
#include "commons.h"

TEST (commons, u16_to_str_test) {
  std::vector<std::pair<uint16_t, std::string>> lst_test_cases;
  lst_test_cases.push_back(std::make_pair<uint16_t, std::string>(1, "1"));
  lst_test_cases.push_back(std::make_pair<uint16_t, std::string>(12, "12"));
  lst_test_cases.push_back(std::make_pair<uint16_t, std::string>(123, "123"));
  lst_test_cases.push_back(std::make_pair<uint16_t, std::string>(1234, "1234"));
  lst_test_cases.push_back(std::make_pair<uint16_t, std::string>(12345, "12345"));
  lst_test_cases.push_back(std::make_pair<uint16_t, std::string>(UINT16_MAX, "65535"));

  for (auto t : lst_test_cases) {
    ASSERT_STREQ(u16_to_str(t.first), t.second.c_str());
  }
}
///////////////////////////////////////////////////////

