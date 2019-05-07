#pragma once

#include <functional>
#include <iostream>
#include <memory>
#include <utility>

#include "drake/common/drake_assert.h"

namespace drake {

/**
 Permits binding either a class method pointer (const-only) or
 `std::function<>` as an argument via implicit conversion. Provides inheritance
 checks when binding for derived classes. Intended to consolidate overloads.
 */
template <typename Base, bool PermitMutable, typename Return, typename... Args>
class FunctionOrMethod {
 public:
  using Function = std::function<Return (Args...)>;

  template <typename Derived>
  using ConstMethod = Return (Derived::*)(Args...) const;

  template <typename Derived>
  using MutableMethod = Return (Derived::*)(Args...);

  /** Constructs from lambda. **/
  template <typename Lambda>
  FunctionOrMethod(Lambda lambda)  // NOLINT
    : function_(lambda) {}

  /** Constructs from const class method. */
  template <typename Derived>
  FunctionOrMethod(ConstMethod<Derived> const_method)  // NOLINT
    : method_(new MethodImpl<const Derived, ConstMethod<Derived>>(
          const_method)) {}

  /** Constructs from mutable class method (if enabled). */
  template <
      typename Derived,
      typename SFINAE = typename std::enable_if<PermitMutable, Derived>::type>
  FunctionOrMethod(MutableMethod<Derived> method)  // NOLINT
    : method_(new MethodImpl<Derived, MutableMethod<Derived>>(method)) {}

  /** Retrieves the stored function, or binds and return the stored method. */
  Function get(Base* ptr) const {
    if (method_) {
      return method_->Bind(ptr);
    } else {
      return function_;
    }
  }

 private:
  // Permit erasing methods (either const or mutable).
  class MethodErasure {
   public:
    virtual ~MethodErasure() {}
    // Bind to a given instance.
    virtual Function Bind(Base* ptr) = 0;
  };

  // Implementation of method erasure for any derived type, for either const or
  // immutable methods.
  template <typename Derived, typename Method>
  class MethodImpl : public MethodErasure {
   public:
    static_assert(std::is_base_of<Base, Derived>::value, "Invalid inheritance");
    explicit MethodImpl(Method method) : method_(method) {}

    Function Bind(Base* base) override {
      Derived* ptr = dynamic_cast<Derived*>(base);
      DRAKE_DEMAND(ptr != nullptr);
      auto is_void = std::is_same<Return, void>{};
      return BindImpl(is_void, ptr, method_);
    }

   private:
    // For void return type.
    auto BindImpl(
        std::true_type /* is_void */, Derived* ptr, Method method) {
      return [ptr, method](Args... args) {
        (ptr->*method)(std::forward<Args>(args)...);
      };
    }

    // For non-void return type.
    auto BindImpl(
        std::false_type /* is_void */, Derived* ptr, Method method) {
      return [ptr, method](Args... args) -> Return {
        return (ptr->*method)(std::forward<Args>(args)...);
      };
    }

    Method method_;
  };

  Function function_;
  std::unique_ptr<MethodErasure> method_;
};

}  // namespace drake
