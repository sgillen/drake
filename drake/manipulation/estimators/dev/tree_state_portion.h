#pragma once

#include <Eigen/Dense>
#include <type_traits>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

#include "drake/systems/framework/vector_base.h"

#include "drake/manipulation/estimators/dev/stamped_value.h"

namespace drake {
namespace manipulation {

/**
 * Check that two 2D Eigen matrices have the same shape.
 * @note This ignores type.
 */
template <typename DerivedA, typename DerivedB>
bool IsSameDim(const Eigen::MatrixBase<DerivedA>& A,
               const Eigen::MatrixBase<DerivedB>& B) {
  return A.rows() == B.rows() && A.cols() == B.cols();
}

template <typename Integral>
VectorX<Integral> CardinalIndices(Integral size) {
  VectorX<Integral> indices(size);
  auto begin = indices.data();
  auto end = begin + size;
  std::iota(begin, end, Integral(0));
  return indices;
}

// Rather than this:
//  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(VectorPortion);
// Permit move construction only (no assignment).
#define DRAKE_MOVE_ONLY_NO_COPY_NO_ASSIGN(Classname)    \
  Classname(const Classname&) = delete;         \
  void operator=(const Classname&) = delete;        \
  Classname(Classname&&) = default;             \
  void operator=(Classname&&) = delete;

class VectorSlice {
 public:
  typedef VectorX<T> Vector;
  typedef std::vector<int> Indices;

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(VectorSlice);

  explicit VectorSlice(const Indices& indices, int super_size = -1)
        : indices_(indices) {
    // TODO(eric.cousineau): Check for uniqueness
    if (indices_.size() > 0) {
      min_index_ = *std::min_element(indices.begin(), indices.end());
      DRAKE_ASSERT(min_index_ >= 0);
      max_index_ = *std::max_element(indices.begin(), indices.end());
    }
    if (super_size == -1) {
      super_size_ = max_index_;
    } else {
      super_size_ = super_size;
      DRAKE_ASSERT(max_index_ <= super_size_);
    }
  }

  int size() const { return indices_.size(); }
  int super_size() const { return super_size_; }
  int min_index() const { return min_index_; }
  int max_index() const { return max_index_; }
  const Indices& indices() const { return indices_; }

  template <typename Container>
  bool is_valid_subset_of(const Container& other) const {
    return super_size() == other.size();
  }

  void ReadFromSuperset(const Vector& super, Eigen::Ref<Vector> values) const {
    DRAKE_ASSERT(is_valid_subset_of(super));
    for (int i = 0; i < this->size(); ++i) {
      int index = this->indices_[i];
      values[i] = super[index];
    }
  }

  void WriteToSuperset(const Vector& values, Eigen::Ref<Vector> super) const {
    DRAKE_ASSERT(is_valid_subset_of(super));
    for (int i = 0; i < this->size(); ++i) {
      int index = this->indices_[i];
      super[index] = values[i];
    }
  }

 protected:
  int super_size_{};
  int max_index_{};
  int min_index_{};
  const Indices indices_;
};

/**
 * Simple mechanism to handle (ragged) slices of a vector.
 */
// TODO(eric.cousineau): Consider storing an immutable reference to the parent
// vector, for assured validity?
// TODO(eric.cousineau): If storing a reference to the parent, could use
// symbolic variables in order to sync up with the parent type. Perhaps something
// like: IndexedVectorPortion (no knowledge of parent), and then
// SemanticVectorPortion / SymbolicVectorPortion (knowledge of parent... possibly
// knowledge of children?
// Would need:
//   VectorXV = VectorX<symbolic::Variable>
//   VectorXV full_variables;
//   VectorXV vp = map_strings_to_variables(VectorXV v, vector<string> names);
// Could also just store values within - symbolic::ValuePair<double, Varaible>
// And follow suite with StampedValue<> stuff.
// TODO(eric.cousineau): Consider storing the size of the superset? This would
// prevent some minor misidentifications.
template <typename T>
class VectorPortion : public VectorSlice<T> {
 public:
  // Helpers
  typedef VectorX<T> Vector;
  typedef VectorSlice<T> Base;
  typedef typename Base::Indices Indices;

//  DRAKE_MOVE_ONLY_NO_COPY_NO_ASSIGN(VectorPortion);
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(VectorPortion);  // Permit use as abstract value

  VectorPortion(const Indices& sub_indices, int super_size = -1)
      : Base(sub_indices, super_size), values_(this->size()) {}

  static VectorPortion Make(int size) {
    return std::move(VectorPortion(CardinalIndices(size)));
  }
  template <typename Derived>
  static VectorPortion Make(const Eigen::MatrixBase<Derived>& values) {
    DRAKE_ASSERT(values.cols() == 1);
    auto out{Make(values.size())};
    out.values() = values;
    return std::move(out);
  }

  /**
   * Aggregate unique vector portions into one. They should all have the same
   * frame of reference, and they should all be disjoint.
   */
  // TODO(eric.cousineau): As mentioned above, construction should check for
  // uniqueness of indices.
  static VectorPortion Aggregate(
      const std::vector<const VectorPortion*>& portions) {
    // Use const pointers so an initializer list can be used.
    DRAKE_ASSERT(portions.size() > 0);
    int size = 0;
    for (auto portion : portions) {
      size += portion->size();
    }
    Indices indices(size);
    Vector values(values);
    // Use Eigen's CommanInitializer.
    auto indices_init = (indices << portions[0]->indices());
    auto values_init = (values << portions[0]->values());
    for (int i = 1; i < portions.size(); ++i) {
      // Sigh... Only exposed via a comma operator.
      indices_init, portions[i]->indices();
      values_init, portions[i]->values();
    }
    VectorPortion out(indices);
    out.values() = values;
    return out;
  }

  const Vector& values() const { return values_; }
  // Only offer a reference, such that the shape cannot be changed.
  Eigen::Ref<Vector> values() { return values_; }

  void ReadFromSuperset(const VectorPortion& super) {
    this->ReadFromSuperset(super.values(), values());
  }

  void ReadFromSubset(const VectorPortion& sub) {
    DRAKE_ASSERT(sub.is_valid_subset_of(*this));
    const auto& sub_indices = sub.indices();
    const auto& sub_values = sub.values();
    for (int i = 0; i < sub.size(); ++i) {
      int index = sub_indices[i];
      values_[index] = sub_values[i];
    }
    return *this;
  }

 protected:
  Vector values_;
};

// There is Subvector<T> in the systems framework.
// However, this only handles contiguous slices.

//template <typename T>
//class SubVectorBase : public systems::VectorBase<T> {
// public:
//  typedef VectorPortion<T> Portion;
//  typedef typename Portion::Indices Indices;
//  SubVectorBase(const Indices &indices)
//      : portion_(indices) {}
//  int size() const override {
//    return portion_.size();
//  }
//  const T& GetAtIndex(int index) const override {
//    return portion_.values()(index);
//  }
//  T& GetAtIndex(int index) override {
//    return portion_.values()(index);
//  }
//  const Portion& portion() const { return portion_; }
//  Portion& portion() { return portion_; }
// private:
//  Portion portion_;
//};

// Portion of vector that records timestamps.
template <typename T>
using VectorStampedPortion = VectorPortion<StampedValue<T>>;

/**
 * Store a portion of the kinematic state of a tree (world).
 *
 * If the full set of a tree is x[k] = { x_r[k] }_{r ∊ R} at frame k, where R is
 * the indexing state for the total number of robots, and each robot's state is
 * x_r[k] = { x_r_i[l] }_{i ∊ O_r}, where O_r is the indexing set for the state
 * vector (q and v), then this represents the portion:
 *  xp[k] = {xp_r[k]}_{r ∊ Rp} ⊆ x[k], where Rp ⊆ R, and
 *  xp_r[k] = {xp_r_i[k]}_{i ∊ Op_r} ⊆ x_r[k], where Op_r ⊆ O_r.
 *
 * Note that this can be used to represent (a) a subset of the robots in a tree,
 * and/or (b) a portion of the joints of a subset of the robots.
 *
 * This class itself does not distinguish among different robots.
 * Rather, different state portions should be stored for different robots.
 *
 * The indices stored for mapping are local to only the immediate superset, e.g.
 * If you have xpp ⊆ xp ⊆ x, then max(Opp) is most likely NOT size(O).
 *
 * All indices must be UNIQUE, and position and velocity indices should relate
 * to the same physical states.
 *
 * @note Generalization of structure within: RobotStateLcmMessageTranslator.
 * @note Can templatize on a something akin to a StampedCache, such that the
 * vectors could be implicitly casted to doubles, and could be set element-wise
 * with ease. (For state estimation, if coordinates come from other sources at
 * different rates.)
 */
//template <typename T>
//class KinematicStatePortion {
// public:
//  typedef VectorX<T> Vector;
//  typedef VectorX<int> Indices;
//  typedef VectorPortion<T> Portion;

//  DRAKE_MOVE_ONLY_NO_COPY_NO_ASSIGN(KinematicStatePortion);

//  KinematicStatePortion(const Indices& joint_indices)
//      : positions_(joint_indices), velocities_(joint_indices) {}
//  KinematicStatePortion(const Indices& position_indices,
//                        const Indices& velocity_indices)
//      : positions_(position_indices), velocities_(velocity_indices) {}

//  // Get lazy.
//  template <typename Arg>
//  static KinematicStatePortion Make(Arg&& arg) {
//    return std::move(
//          KinematicStatePortion(Portion::Make(arg), Portion::Make(arg)));
//  }
//  template <typename PosArg, typename VelArg>
//  static KinematicStatePortion Make(PosArg&& pos, VelArg&& vel) {
//    return std::move(
//          KinematicStatePortion(Portion::Make(pos), Portion::Make(vel)));
//  }

//  Portion& positions() { return positions_; }
//  const Portion& positions() const { return positions_; }

//  Portion& velocities() { return velocities_; }
//  const Portion& velocities() const { return velocities_; }

//  void ReadFromSuperset(const KinematicStatePortion& super) {
//    positions_.ReadFromSuperset(super.positions());
//    velocities_.ReadFromSuperset(super.velocities());
//  }

//  void ReadFromSubset(const KinematicStatePortion& sub) {
//    positions_.ReadFromSubset(sub.positions());
//    velocities_.ReadFromSubset(sub.velocities());
//  }

// private:
//  KinematicStatePortion(Portion&& positions, Portion&& velocities)
//      : positions_(std::move(positions)), velocities_(std::move(velocities)) {}
//  Portion positions_;
//  Portion velocities_;
//};

//template <typename T>
//using KinematicStateStampedPortion = KinematicStatePortion<StampedValue<T>>;

}  // manipulation
}  // drake
