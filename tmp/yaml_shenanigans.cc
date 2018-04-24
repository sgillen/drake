#include <iostream>
#include <string>

#include <Eigen/Dense>
#include "yaml-cpp/yaml.h"

int main() {
  std::string contents = R"""(
key:
- [1, 2, 3]
- [4, 5, 6]
)""";
  YAML::Node node = YAML::Load(contents);

  auto cur = node["key"];
  int rows = cur.size();
  int cols = -1;
  int row = 0;
  Eigen::MatrixXd out;
  for (auto cur_row : cur) {
    if (cols == -1) {
      cols = cur_row.size();
      out.resize(rows, cols);
    }
    else
      assert(cols == cur_row.size());
    int col = 0;
    for (auto cur_item : cur_row) {
      out(row, col) = cur_item.as<double>();
      col++;
    }
    row++;
  }

  std::cout
      << "Results:" << std::endl
      << out << std::endl;

  return 0;
}

/**
Output:

Results:
1 2 3
4 5 6
**/
