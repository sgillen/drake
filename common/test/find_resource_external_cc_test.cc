#include "drake/common/find_resource.h"

#include <climits>
#include <cstdlib>
#include <string>

#include <gtest/gtest.h>
#include <spruce.hh>

#include "drake/common/drake_throw.h"
#include "drake/common/text_logging.h"

using std::string;

namespace drake {
namespace {

std::string simplify(std::string str) {
  spruce::path p;
  if (str[0] == '/') {
    p = str;
  } else {
    p.setAsCurrent();
    p.append(str);
  }
  std::vector<string> pieces;
  for (auto piece : p.split()) {
    if (piece == ".") {
      continue;
    } else if (piece == "..") {
      pieces.back() = piece;
    } else {
      pieces.push_back(piece);
    }
  }
  spruce::path out;
  for (auto piece : pieces) {
    out.append(piece);
  }
  return out.getStr();
}

GTEST_TEST(FindResourceTest, FoundDeclaredData) {
  drake::log()->set_level(spdlog::level::trace);
  const string relpath = "drake/common/test/find_resource_test_data.txt";
  const string resource_path = FindResourceOrThrow(relpath);
  EXPECT_EQ(simplify(resource_path), simplify("external/" + relpath));
}

}  // namespace
}  // namespace drake
