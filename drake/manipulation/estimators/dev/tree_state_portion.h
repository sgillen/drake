#include <Eigen/Dense>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace manipulation {

/**
 * Check that two 2D Eigen matrices have the same shape.
 * @note This ignores type.
 */
template <typename DerivedA, typename DerivedB>
bool is_same_dim(const Eigen::MatrixBase<DerivedA>& A,
                  const Eigen::MatrixBase<DerivedB>& B) {
  return A.rows() == B.rows() && A.cols() == B.cols();
}

typedef VectorX<int> Indices;

static Indices cardinal_indices(int size) {
  Indices indices(size);
  auto begin = indices.data();
  auto end = begin + size;
  std::iota(begin, end, 0);
  return indices;
}

/**
 *
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
 */
template <typename T>
class TreeStatePortion {
 public:
  // Helpers
  typedef VectorX<T> Vector;
  typedef VectorX<int> Indices;
  template <typename Derived>
  using MatrixBase = Eigen::MatrixBase<Derived>;

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(TreeStatePortion);
  TreeStatePortion() {}
  explicit TreeStatePortion(const Indices& joint_indices)
      : TreeStatePortion(joint_indices, joint_indices) {}
  TreeStatePortion(const Indices& position_indices,
                   const Indices& velocity_indices)
        : position_indices_(position_indices),
          velocity_indices_(velocity_indices) {
    positions_.resize(num_positions());
    positions_.setZero();
    velocities_.resize(num_velocities());
    velocities_.setZero();
  }

  static TreeStatePortion Make(int size) {
    return TreeStatePortion(cardinal_indices(size));
  }
  static TreeStatePortion Make(const Vector& positions) {
    auto value = Make(positions.size());
    value.set_positions(positions);
    return value;
  }
  static TreeStatePortion Make(int num_positions, int num_velocities) {
    return TreeStatePortion(cardinal_indices(num_positions),
                            cardinal_indices(num_velocities));
  }
  static TreeStatePortion Make(const Vector& positions,
                               const Vector& velocities) {
    auto value = TreeStatePortion::Make(positions.size(), velocities.size());
    value.set_positions(positions);
    value.set_velocities(velocities);
  }

  int num_positions() const { return position_indices_.size(); }
  int num_velocities() const { return velocity_indices_.size(); }
  const Vector& positions() const { return positions_; }

  template <typename Derived>
  void set_positions(const MatrixBase<Derived>& value) {
    DRAKE_ASSERT(is_same_dim(positions_, value));
    positions_ = value;
  }

  // TODO(eric.cousineau): Add ability to set using unordered map to guide
  // indexing.
  template <typename Derived>
  void set_velocities(const MatrixBase<Derived>& value) {
    DRAKE_ASSERT(is_same_dim(velocities_, value));
    velocities_ = value;
  }

 private:
  Indices position_indices_;
  Vector positions_;
  Indices velocity_indices_;
  Vector velocities_;
};


}  // manipulation
}  // drake
