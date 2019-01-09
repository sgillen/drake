#include "drake/tmp/scene_graph_parser.h"

#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {

using multibody::MultibodyPlant;
using multibody::Parser;
using multibody::PackageMap;

namespace geometry {

struct SceneGraphParser::Impl {
  SceneGraph<double>* scene_graph{};
  MultibodyPlant<double> plant;
  Parser parser;

  Impl(SceneGraph<double>* scene_graph_in)
      : scene_graph(scene_graph_in),
        plant(),
        parser(&plant) {
    plant.RegisterAsSourceForSceneGraph(scene_graph);
  }
};

SceneGraphParser::SceneGraphParser(SceneGraph<double>* scene_graph)
  : impl_(new Impl(scene_graph)) {}

SceneGraphParser::~SceneGraphParser() {}

PackageMap& SceneGraphParser::package_map() {
  DRAKE_DEMAND(impl_ != nullptr);
  return impl_->parser.package_map();
}

void SceneGraphParser::AddModelFromFile(
    const std::string& file_name, const std::string& model_name) {
  DRAKE_DEMAND(impl_ != nullptr);
  impl_->parser.AddModelFromFile(file_name, model_name);
}

SourceId SceneGraphParser::Finalize() {
  DRAKE_DEMAND(impl_ != nullptr);
  impl_->plant.Finalize();
  auto id = impl_->plant.get_source_id();
  DRAKE_DEMAND(id.has_value());
  impl_.reset();
  return *id;
}

}
}
