#pragma once

#include <mutex>
#include <string>

#include <Eigen/Core>

#include "drake/common/drake_copyable.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {

/*
 * Permit a scoped lock of a MOSEK license.
 * This will attempt to obtain a MOSEK license session if it is the first one.
 * If it fails, it will throw a runtime_error.
 * Once there are no locks in scope, the MOSEK license session will be
 * released.
 */
class MosekLicenseLock {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MosekLicenseLock)

  MosekLicenseLock();
  ~MosekLicenseLock();

  void* get_env() const;

 private:
  struct License {
    // This is a void* to avoid needing to refer to types from the Mosek
    // API if we're building without Mosek.  Note that both of these are
    // mutable to allow latching the allocation of mosek_env_ during the
    // first call so Solve() (which avoids grabbing a Mosek license
    // before we know that we actually want one).
    mutable void* mosek_env_{nullptr};
    mutable int ref_count_{0};
    mutable std::mutex env_mutex_;

    void Acquire();
    void Release();
  };
  static License license_;
};

class MosekSolver : public MathematicalProgramSolverInterface {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MosekSolver)

  MosekSolver() : MathematicalProgramSolverInterface(SolverType::kMosek) {}
  ~MosekSolver();

  /**
   * Defined true if Mosek was included during compilation, false otherwise.
   */
  bool available() const override;

  SolutionResult Solve(MathematicalProgram& prog) const override;

 private:
  std::unique_ptr<MosekLicenseLock> license_;
};

}  // namespace solvers
}  // namespace drake
