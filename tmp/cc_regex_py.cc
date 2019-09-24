#include "pybind11/pybind11.h"

#include "cc_regex.h"

namespace drake {
namespace py = pybind11;

PYBIND11_MODULE(cc_regex, m) {
  m.def("get_name", &GetName);
  m.def("dl_iterate", &DlIterate);
}

}  // namespace drake
