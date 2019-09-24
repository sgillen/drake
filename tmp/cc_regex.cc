#include <iostream>
#include <regex>

#include "cc_regex.h"

namespace drake {

std::string GetName() {
  // Construct a regex to try and repro `free(): invalid pointer`
  std::cerr
      << "_GLIBCXX_USE_CXX11_ABI: " << _GLIBCXX_USE_CXX11_ABI << std::endl;
  std::regex sub("\\b(class|struct|enum|union) ");
  std::cerr << "&sub: " << &sub << std::endl;
  return "nothing";
}

}
