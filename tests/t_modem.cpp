#include <gtest/gtest.h>
#include <iostream>

TEST (modem, ms_open) {
  const char *rp = "\r\n+NETOPEN: ";
  const char *mb = "\r\n+NETOPEN: 0\r\n";

  while (*rp == *mb) {
    ++rp; ++mb;
  }

  std::cout << *mb << std::endl;
}
///////////////////////////////////////////////////////

