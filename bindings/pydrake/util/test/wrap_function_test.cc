#include <functional>
#include <type_traits>

#include <gtest/gtest.h>

#include "drake/bindings/pydrake/util/wrap_function.h"

using namespace std;

namespace drake {

// N.B. Anonymous namespace not used as it makes failure messages
// (static_assert) harder to interpret.

// Will keep argument / return types the same, but homogenize method signatures.
template <typename Func>
auto WrapIdentity(Func&& func) {
  return WrapFunction<wrap_arg_default>(std::forward<Func>(func));
}

#define WTEST(name) GTEST_TEST(WrapFunction, name)

// Function with `void` return type, `int` by value.
void FunctionPtr(int value) {}

WTEST(FunctionPtr) {
  int value = 0;
  WrapIdentity(FunctionPtr)(value);
  EXPECT_EQ(value, 0);
}

// Lambdas / basic functors.
WTEST(Lambda) {
  int value = 0;
  auto func_1_lambda = [](int value) {};
  WrapIdentity(func_1_lambda)(value);
  EXPECT_EQ(value, 0);

  std::function<void (int)> func_1_func = func_1_lambda;
  WrapIdentity(func_1_func)(value);
  EXPECT_EQ(value, 0);
}

// Class methods.
class MyClass {
 public:
  static int MethodStatic(int value) { return value; }
  int MethodMutable(int value) { return value + value_; }
  int MethodConst(int value) const { return value * value_; }

 private:
  int value_{10};
};

WTEST(Methods) {
  int value = 2;

  MyClass c;
  const MyClass& c_const{c};

  // Wrapped signature: Unchanged.
  EXPECT_EQ(WrapIdentity(&MyClass::MethodStatic)(value), 2);
  // Wrapped signature: int (MyClass*, int)
  auto method_mutable = WrapIdentity(&MyClass::MethodMutable);
  EXPECT_EQ(method_mutable(&c, value), 12);
  // method_mutable(&c_const, value);  // Should fail.
  // Wrapped signature: int (const MyClass*, int)
  EXPECT_EQ(WrapIdentity(&MyClass::MethodConst)(&c_const, value), 20);
}

// Move-only arguments.
struct MoveOnlyValue {
  MoveOnlyValue() = default;
  MoveOnlyValue(const MoveOnlyValue&) = delete;
  MoveOnlyValue& operator=(const MoveOnlyValue&) = delete;
  MoveOnlyValue(MoveOnlyValue&&) = default;
  MoveOnlyValue& operator=(MoveOnlyValue&&) = default;
  int value{};
};

void ArgMoveOnly(MoveOnlyValue arg) {
  EXPECT_EQ(arg.value, 1);
}

const int& ArgMoveOnlyConst(const MoveOnlyValue& arg) {
  return arg.value;
}

int& ArgMoveOnlyMutable(MoveOnlyValue& arg) {
  arg.value += 1;
  return arg.value;
}

WTEST(ArgMoveOnly) {
  WrapIdentity(ArgMoveOnly)(MoveOnlyValue{1});
  MoveOnlyValue v{10};
  
  auto& out_const = WrapIdentity(ArgMoveOnlyConst)(v);
  v.value += 10;
  EXPECT_EQ(out_const, 20);

  auto& out_mutable = WrapIdentity(ArgMoveOnlyMutable)(v);
  EXPECT_EQ(out_mutable, 21);
  out_mutable += 1;
  EXPECT_EQ(v.value, 22);
}


// Test a slightly complicated conversion mechanism.

// Constructible only via brackets (no implicit conversions).
template <typename T>
struct const_ptr {
  static_assert(!std::is_const<T>::value, "Bad (redundant) inference");
  const T* value{};
};

template <typename T>
struct ptr {
  // Ensure that this is not being used in lieu of `ptr` (ensure that
  // our specializiation delegate correctly).
  static_assert(
      !std::is_const<T>::value, "Should be using `const_ptr`");
  T* value{};
};


// Base case: Pass though.
template <typename T, typename = void>
struct wrap_change : public wrap_arg_default<T> {
  // For `wrap_change<const T&>`.
  using Wrapped = T;
};

// Wraps any `const T*` with `const_ptr`, except for `int`.
// SFINAE. Could be achieved with specialization, but using to uphold SFINAE
// contract provided by `WrapFunction`.
template <typename T>
struct wrap_change<const T*, std::enable_if_t<!std::is_same<T, int>::value>> {
  // For `wrap_change<const T&>`.
  using Wrapped = const_ptr<T>;

  static Wrapped wrap(const T* arg) {
    return {arg};
  }

  static const T* unwrap(Wrapped arg_wrapped) {
    return arg_wrapped.value;
  }
};

// Wraps any `const T&` with `const_ptr`, except for `int`.
// Leverage `const T&` logic, such that we'd get the default template when
// SFINAE prevents matching.
template <typename T>
struct wrap_change<const T&> : public wrap_change<const T*> {
  using base = wrap_change<const T*>;
  using Wrapped = typename base::Wrapped;

  static Wrapped wrap(const T& arg) { return base::wrap(&arg); }

  static const T& unwrap(Wrapped arg_wrapped) {
    return *base::unwrap(arg_wrapped);
  }
};

// Wraps any mutable `T*` with `ptr`.
// N.B. Prevent `const T*` from binding here, since it may be rejected from
// SFINAE.
template <typename T>
struct wrap_change<T*, std::enable_if_t<!std::is_const<T>::value>> {
  static ptr<T> wrap(T* arg) { return {arg}; }
  static T* unwrap(ptr<T> arg_wrapped) { return arg_wrapped.value; }
};

// Wraps any mutable `T&` with `ptr`.
template <typename T>
struct wrap_change<T&> {
  static ptr<T> wrap(T& arg) { return {&arg}; }
  static T& unwrap(ptr<T> arg_wrapped) { return *arg_wrapped.value; }
};

// Test case to exercise `WrapFunction`.
// Mappings:
//   `T*`         -> `ptr<T>` (always)
//   `T&`         -> `ptr<T>` (always).
//   `const T*`   -> `const_ptr<T>`, if `T` is not `int`.
//   `const int*` -> `const int*`
//   `const T&`   -> `const_ptr<T>`, if `T` is not `int`.
//   `const int&` -> `const int*`
template <typename Func>
auto WrapChange(Func&& func) {
  return WrapFunction<wrap_change>(std::forward<Func>(func));
}

template <typename T>
using wrap_arg_t =
    detail::wrap_function_impl<wrap_change>::wrap_arg_t<T>;

template <typename Expected, typename Actual>
void check_type() {
  // Use this function to inspect types when failure is encountered.
  static_assert(std::is_same<Actual, Expected>::value, "Mismatch");
}

GTEST_TEST(WrapFunction, ChangeTypeCheck) {
  // Codify rules above.

  // Use arbitrary T that is not constrained by the rules.
  using T = double;

  check_type<ptr<T>, wrap_arg_t<T*>>();
  check_type<ptr<int>, wrap_arg_t<int*>>();

  check_type<ptr<T>, wrap_arg_t<T&>>();
  check_type<ptr<int>, wrap_arg_t<int&>>();

  check_type<const_ptr<T>, wrap_arg_t<const T*>>();
  check_type<const int*, wrap_arg_t<const int*>>();

  check_type<const_ptr<T>, wrap_arg_t<const T&>>();
  check_type<const int*, wrap_arg_t<const int&>>();
}

// Function with a pointer return type, 
int* Change1(int& value) {
  value += 1;
  return &value;
}

WTEST(Change1) {
  int value = 0;
  // Wrapped signature: `ptr<int> (ptr<int>)`
  auto out = WrapChange(Change1)(ptr<int>{&value});
  EXPECT_EQ(*out.value, 1);
  EXPECT_EQ(value, 1);
}

// Unspecialized types.
const double& Func_3a(const double& value) { return value; }
// Specialized types.
const int& Func_3b(const int& value) { return value; }

WTEST(Func_3) {
  {
    double value = 0.;
    // Wrapped signature: `const_ptr<double> (const_ptr<double>)`
    auto out = WrapChange(Func_3a)(const_ptr<double>{&value});
    value = 1.;
    EXPECT_EQ(*out.value, 1.);
  }

  {
    int value = 0;
    // Wrapped signature: `const int* (const int*)`
    auto out = WrapChange(Func_3b)(&value);
    value = 1;
    EXPECT_EQ(*out, 1);
  }
}


// void Func_5(const int* value) {}

// void Func_6(int& value, std::function<void (int&)> callback) {
//   callback(value);
// }

// int& Func_7(int& value, const std::function<int& (int&)>& callback) {
//   return callback(value);
// }

// class MyClass {
//  public:
//   static void Func(MoveOnlyValue&& value) {}
//   void Method(MoveOnlyValue& value) { value.value += 2; }
//   void Method_2(MoveOnlyValue& value) const { value.value += 3; }
// };

// Provides a functor which can be default constructed and moved only.
struct MoveOnlyFunctor {
  MoveOnlyFunctor() {}
  MoveOnlyFunctor(const MoveOnlyFunctor&) = delete;
  MoveOnlyFunctor& operator=(const MoveOnlyFunctor&) = delete;
  MoveOnlyFunctor(MoveOnlyFunctor&&) = default;
  MoveOnlyFunctor& operator=(MoveOnlyFunctor&&) = default;
  // N.B. Per documentation, cannot overload operator(), as it's ambiguous when
  // attempting to infer arguments.
  void operator()(MoveOnlyValue& value) const {
    value.value += 4;
  }
};

struct ConstFunctor {
  void operator()(MoveOnlyValue& value) const { value.value += 5; }
};

GTEST_TEST(WrapFunction, ExampleFunctors) {
  // MoveOnlyValue v{0};

  // {
  //   auto out = WrapChange(Func_3)(&v.value);
  //   // Mutate to ensure we have a pointer.
  //   v.value += 1;
  //   EXPECT_EQ(*out, 2);
  //   EXPECT_EQ(v.value, 2);
  // }
  
  // CHECK(WrapChange(Func_4)(MoveOnlyValue{}));
  // CHECK(WrapChange(Func_5)(&v.value));

  // // Lambda.
  // auto void_ref = [](int& value) {
  //   value += 10;
  // };
  // CHECK(WrapChange(void_ref)(&v.value));

  // CHECK(WrapChange(MyClass::Func)(MoveOnlyValue{}));
  // MyClass c;
  // const MyClass& c_const{c};
  // CHECK(WrapChange(&MyClass::Method)(&c, &v));
  // CHECK(WrapChange(&MyClass::Method_2)(&c_const, &v));

  // MoveOnlyFunctor f;
  // CHECK(WrapChange(std::move(f))(&v));
  // ConstFunctor g;
  // CHECK(WrapChange(g)(&v));
  // const ConstFunctor& g_const{g};
  // CHECK(WrapChange(g_const)(&v));

  // // Callback.
  // CHECK(WrapChange(Func_6)(&v.value, WrapChange(void_ref)));

  // // Callback with return.
  // auto get_ref = [](int& value) -> int& {
  //   value += 100;
  //   return value;
  // };
  // CHECK(cout << *WrapChange(Func_7)(&v.value, WrapChange(get_ref)));

  // // Nested callback.
  // auto get_ref_nested = [get_ref](int& value,
  //     std::function<int& (int&, const std::function<int& (int&)>&)> func) -> auto& {
  //   value += 1000;
  //   return func(value, get_ref);
  // };
  // CHECK(cout << *WrapChange(get_ref_nested)(&v.value, WrapChange(Func_7)));
}

}  // namespace drake
