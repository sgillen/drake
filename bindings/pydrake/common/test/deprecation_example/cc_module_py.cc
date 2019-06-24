#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/common/deprecation_example/example_class_documentation.h"  // NOLINT
#include "drake/bindings/pydrake/common/deprecation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/common/drake_deprecated.h"

namespace drake {
namespace pydrake {
namespace {

PYBIND11_MODULE(cc_module, m) {
  constexpr auto& doc = pydrake_doc.drake.example_class;

  m.def("emit_deprecation", []() {
    // N.B. This is only for coverage. You generally do not need this.
    WarnDeprecated("Example emitting of deprecation");
  });

  {
    constexpr auto& cls_doc = doc.ExampleCpp;
    py::class_<ExampleCppClass> cls(m, "ExampleCppClass", cls_doc.doc);
    cls  // BR
        .def(py::init(), cls_doc.ctor.doc_0args)
        .def(py_init_deprecated<ExampleCppClass, int>(cls_doc.ctor.doc_1args_depcated), cls_doc.ctor.doc_1args_depcated)
        .def(py_init_deprecated(cls_doc.ctor.doc_1args_deprecated,
            [](double arg) { return ExampleCppClass(arg); }), cls_doc.ctor.doc_1args_depcated)
        .def("overload", py::overload_cast<>(&ExampleCppClass::overload), cls_doc.overload.doc_0args);

    // Add deprecated method.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    cls.def("DeprecatedMethod", &ExampleCppClass::DeprecatedMethod, cls_doc.DeprecatedMethod.doc);
#pragma GCC diagnostic pop
    DeprecateAttribute(cls, "DeprecatedMethod", cls_doc.DeprecatedMetohd.doc);

    // Add deprecated property.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    cls.def_readwrite("deprecated_prop", &ExampleCppClass::deprecated_prop, cls_doc.deprecated_prop.doc);
#pragma GCC diagnostic pop
    DeprecateAttribute(cls, "deprecated_prop", cls_doc.deprecated_prop.doc);

    // Add deprecated overload.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    cls.def("overload", WrapDeprecated(cls_doc.overload.doc_1args,
                            py::overload_cast<int>(&ExampleCppClass::overload)), cls_doc.overload.doc_1args);
#pragma GCC diagnostic pop
  }
}

}  // namespace
}  // namespace pydrake
}  // namespace drake
