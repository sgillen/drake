#pragma once

#include <Eigen/Dense>
#include <type_traits>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

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

template <typename T>
std::vector<T> CardinalIndices(T size) {
  std::vector<T> indices(size);
  std::iota(indices.begin(), indices.end(), T{});
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

// This permits repeated indices.
class VectorSlice {
 public:
  typedef std::vector<int> Indices;

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(VectorSlice);

  VectorSlice() {}
  explicit VectorSlice(const Indices& indices, int super_size)
        : indices_(indices) {
    // TODO(eric.cousineau): Check for uniqueness
    if (indices_.size() > 0) {
      min_index_ = *std::min_element(indices.begin(), indices.end());
      DRAKE_DEMAND(min_index_ >= 0);
      max_index_ = *std::max_element(indices.begin(), indices.end());
    }
    super_size_ = super_size;
    DRAKE_DEMAND(max_index_ <= super_size_);
  }

  int size() const { return indices_.size(); }
  int super_size() const { return super_size_; }
  int min_index() const { return min_index_; }
  int max_index() const { return max_index_; }
  const Indices& indices() const { return indices_; }

  VectorSlice Inverse() const {
    // TODO(eric.cousineau): Simplify logic.
    Indices indices_sorted = indices_;
    std::sort(indices_sorted.begin(), indices_sorted.end());
    Indices inverse_indices;
    int final_index = super_size();
    int index_i = 0;
    int index = size() == 0 ? final_index : indices_sorted[index_i];
    for (int i = 0; i < super_size(); ++i) {
      if (i < index) {
        // Add to inverse list.
        inverse_indices.push_back(i);
      } else {
        // Increment index counter.
        index_i += 1;
        if (index_i == size()) {
          index = final_index;
        } else {
          index = indices_sorted[index_i];
        }
      }
    }
    return VectorSlice(inverse_indices, super_size());
  }

  template <typename Container>
  bool is_valid_subset_of(const Container& other) const {
    return super_size() == other.size();
  }

  template <typename VectorIn, typename VectorOut>
  void ReadFromSuperset(const VectorIn& super, VectorOut&& values) const {
    DRAKE_DEMAND(is_valid_subset_of(super));
    values.resize(this->size());
    for (int i = 0; i < this->size(); ++i) {
      int index = this->indices_[i];
      values[i] = super[index];
    }
  }

  template <typename MatrixIn, typename MatrixOut>
  void ReadFromSupersetMatrix(const MatrixIn& super, MatrixOut&& values) const {
    // Select column-wise (for speed on storage order), one-by-one.
    DRAKE_DEMAND(super.cols() == super_size());
    values.resize(this->size(), this->size());
    for (int i = 0; i < this->size(); ++i) {
      int index = this->indices_[i];
      ReadFromSuperset(super.col(index), values.col(i));
    }
  }

  template <typename VectorIn, typename VectorOut>
  void WriteToSuperset(const VectorIn& values, VectorOut&& super) const {
    DRAKE_DEMAND(is_valid_subset_of(super));  // Do not attepmt to resize.
    for (int i = 0; i < this->size(); ++i) {
      int index = this->indices_[i];
      super[index] = values[i];
    }
  }

  template <typename Container>
  Container CreateFromSuperset(const Container& super) {
    Container values(super.size());
    ReadFromSuperset(super, values);
    return values;
  }

 protected:
  int super_size_{};
  int max_index_{-1};
  int min_index_{-1};
  Indices indices_;  // const discards default assignment (as expected).
};


// NOTE: XprType should be `const T&` or `T&` if not using a view.
// Use perfect forwarding when able.
// This will ONLY work if .row(int) returns a reference object that does
// not need to be forwarded.
// Be wary of scoping.
template <typename XprType>
class RowView {
 public:
  RowView(XprType xpr)
      : xpr_(xpr) {}

  int size() const {
    return xpr_.rows();
  }

  void resize(int row_count) {
    xpr_.resize(row_count, xpr_.cols());
  }

  template <typename Other>
  void resizeLike(const RowView<Other>& other, int row_count = -1) const {
    int cols = other.xpr().cols();
    int rows = row_count == -1 ? other.xpr().rows() : row_count;
    xpr_.resize(rows, cols);
  }

  auto xpr() { return xpr_; }
  auto xpr() const { return xpr_; }

  auto segment(int index, int row_count) {
    return xpr_.middleRows(index, row_count);
  }

  // Note: Attempting perfect forwarding does not play well with temporary
  // row view objects.
  auto operator[](int index) {
    return xpr_.row(index);
  }
  auto operator()(int index) {
    return operator[](index);
  }

  auto operator[](int index) const {
    return xpr_.row(index);
  }
  auto operator()(int index) const {
    return operator[](index);
  }

 private:
  // TODO(eric.cousineau): Add static_assertion.
  XprType xpr_;
};

template<typename XprType>
auto MakeRowView(XprType&& xpr) {
  return RowView<XprType>(std::forward<XprType>(xpr));
}


template <typename XprType>
class ColView {
 public:
  ColView(XprType xpr)
      : xpr_(xpr) {}

  int size() const {
    return xpr_.cols();
  }

  void resize(int col_count) {
    xpr_.resize(col_count, xpr_.cols());
  }

  template <typename Other>
  void resizeLike(const ColView<Other>& other, int col_count = -1) const {
    int cols = col_count == -1 ? other.xpr().cols() : col_count;
    int rows = other.xpr().rows();
    xpr_.resize(rows, cols);
  }

  auto xpr() { return xpr_; }
  auto xpr() const { return xpr_; }

  auto segment(int index, int col_count) {
    return xpr_.middleCols(index, col_count);
  }

  auto operator[](int index) {
    return xpr_.col(index);
  }
  auto operator()(int index) {
    return operator[](index);
  }

  auto operator[](int index) const {
    return xpr_.col(index);
  }
  auto operator()(int index) const {
    return operator[](index);
  }

 private:
  // TODO(eric.cousineau): Add static_assertion.
  XprType xpr_;
};

template<typename XprType>
auto MakeColView(XprType&& xpr) {
  return ColView<XprType>(std::forward<XprType>(xpr));
}


// TODO(eric.cousineau): Do full implementation for more robust access
// (especially for views!!!). Make static_assert for invalid types.
// TODO(eric.cousineau): Name this IndexableIterableMatrix or something.
template <typename XprType>
class IterableMatrix {
 public:
  IterableMatrix(XprType&& xpr)
      : xpr_(xpr) {
    int size = xpr.size();
    auto* back_ptr = &xpr.coeffRef(size - 1);
    if (end() - 1 != back_ptr) {
      throw std::runtime_error("Not a usable storage format");
    }
  }

  auto& operator[](int i) { return xpr_.coeffReff(i); }
  const auto& operator[](int i) const { return xpr_.coeffRef(i); }
  int size() const { return xpr_.size(); }
  auto begin() const { return xpr_.data(); }
  auto end() const { return xpr_.data() + xpr_.size(); }

 private:
  XprType xpr_;
};

template <typename XprType>
auto MakeIterableMatrix(XprType&& xpr) {
  return IterableMatrix<XprType>(std::forward<XprType>(xpr));
}

}  // manipulation
}  // drake
