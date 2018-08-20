using T = double;

PYBIND11_MODULE(_module_py, m) {
  py::class_<SceneGraph, LeafSystem<T>>(m, "SceneGraph");
}
