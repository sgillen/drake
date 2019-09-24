#include <iostream>
#include <regex>

#include "cc_regex.h"

namespace drake {

std::string GetName() {
  // Construct a regex to try and repro `free(): invalid pointer`
  std::regex sub("\\b(class|struct|enum|union) ");
  std::cout << &sub << std::endl;
  return "nothing";
}

}
