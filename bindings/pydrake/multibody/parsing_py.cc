#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/multibody/parsing/package_map.h"
#include "drake/multibody/parsing/parser.h"

using drake::geometry::SceneGraph;
using drake::multibody::MultibodyPlant;
using std::string;

namespace drake {
namespace pydrake {

void init_deprecated(py::module m);

PYBIND11_MODULE(parsing, m) {
  m.doc() = "SDF and URDF parsing for MultibodyPlant and SceneGraph.";

  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::multibody;
  constexpr auto& doc = pydrake_doc.drake.multibody;

  // PackageMap
  {
    using Class = PackageMap;
    constexpr auto& cls_doc = doc.PackageMap;
    py::class_<Class>(m, "PackageMap", cls_doc.doc)
        .def(py::init<>(), cls_doc.ctor.doc_0args)
        .def("Add", &Class::Add, cls_doc.Add.doc)
        .def("Contains", &Class::Contains, cls_doc.Contains.doc)
        .def("size", &Class::size, cls_doc.size.doc)
        .def("GetPath", &Class::GetPath, cls_doc.GetPath.doc)
        .def("PopulateFromFolder", &Class::PopulateFromFolder,
            cls_doc.PopulateFromFolder.doc)
        .def("PopulateFromEnvironment", &Class::PopulateFromEnvironment,
            cls_doc.PopulateFromEnvironment.doc)
        .def("PopulateUpstreamToDrake", &Class::PopulateUpstreamToDrake,
            cls_doc.PopulateUpstreamToDrake.doc);
  }

  // Parser
  {
    using Class = Parser;
    constexpr auto& cls_doc = doc.Parser;
    py::class_<Class>(m, "Parser", cls_doc.doc)
        .def(py::init<MultibodyPlant<double>*, SceneGraph<double>*>(),
            py::arg("plant"), py::arg("scene_graph") = nullptr,
            cls_doc.ctor.doc_2args)
        .def("AddAllModelsFromFile", &Class::AddAllModelsFromFile,
            py::arg("file_name"), cls_doc.AddAllModelsFromFile.doc)
        .def("AddModelFromFile", &Class::AddModelFromFile, py::arg("file_name"),
            py::arg("model_name") = "", cls_doc.AddModelFromFile.doc);
  }

  init_deprecated(m.def_submodule("_deprecated"));
}

void init_deprecated(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::multibody;
  constexpr auto& doc = pydrake_doc.drake.multibody;

  // Bind the deprecated free functions.
  // TODO(jwnimmer-tri) Remove these stubs on or about 2019-03-01.
  m.def("AddModelFromSdfFile",
      [](const string& file_name, const string& model_name,
          MultibodyPlant<double>* plant, SceneGraph<double>* scene_graph) {
        WarnDeprecated(
            "AddModelFromSdfFile is deprecated; please use the class "
            "pydrake.multibody.parsing.Parser instead.");
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
        return parsing::AddModelFromSdfFile(
            file_name, model_name, plant, scene_graph);
#pragma GCC diagnostic pop
      },
      py::arg("file_name"), py::arg("model_name"), py::arg("plant"),
      py::arg("scene_graph") = nullptr, doc.AddModelFromSdfFile.doc_4args);
  m.def("AddModelFromSdfFile",
      [](const string& file_name, MultibodyPlant<double>* plant,
          SceneGraph<double>* scene_graph) {
        WarnDeprecated(
            "AddModelFromSdfFile is deprecated; please use the class "
            "pydrake.multibody.parsing.Parser instead.");
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
        return parsing::AddModelFromSdfFile(file_name, plant, scene_graph);
#pragma GCC diagnostic pop
      },
      py::arg("file_name"), py::arg("plant"), py::arg("scene_graph") = nullptr,
      doc.AddModelFromSdfFile.doc_3args);
}

}  // namespace pydrake
}  // namespace drake
