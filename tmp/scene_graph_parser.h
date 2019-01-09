#pragma once

#include <memory>

#include "drake/geometry/scene_graph.h"
#include "drake/multibody/parsing/package_map.h"

namespace drake {
namespace geometry {

/// Proxy for `drake::multibody::Parser`, but seals off MBP deps.
/// Mode of operation:
/// - Internally creates MBP while loading, permits adding multiple things.
/// - Exposes subset of `Parser`s API.
/// - At `Finalize()`, returns constructed `SceneGraph`, discards plant.
class SceneGraphParser {
 public:
  explicit SceneGraphParser(SceneGraph<double>* scene_graph);
  ~SceneGraphParser();

  multibody::PackageMap& package_map();

  /// See Parser method.
  // Dunno what to return... Also, `model_name` only matters if you need to
  // segregate your queries, or add multiple of the same model.
  void AddModelFromFile(
      const std::string& file_name,
      const std::string& model_name = {});

  /// Finalizes MBP, flushes info to SceneGraph, returns SourceId for MBP, and
  /// invalidates this parser (never to be used again).
  SourceId Finalize();

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}
}
