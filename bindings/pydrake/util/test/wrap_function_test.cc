#include <functional>
#include <iostream>
#include <type_traits>

#include "cpp/name_trait.h"
#include "cpp/wrap_function.h"

using namespace std;

namespace drake {
namespace {

namespace example_wrapper {

// Constructible only via brackets (no implicit conversions).
template <typename T>
struct wrapped_const_ptr {
  const T* ptr{};
};

template <typename T>
struct wrapped_ptr {
  // Ensure that this is not being used in lieu of `wrapped_ptr` (ensure that
  // our specializiation delegate correctly).
  static_assert(
      !std::is_const<T>::value, "Should be using `wrapped_const_ptr`");
  T* ptr{};
};

// Base case: Pass though.
template <typename T, typename = void>
struct wrap_example : public wrap_arg_default<T> {};

// Wraps any `const T*` with `wrapped_const_ptr`, except for `int`.
// SFINAE. Could be achieved with specialization, but using to uphold SFINAE
// contract provided by `WrapFunction`.
template <typename T>
struct wrap_example<const T*, std::enable_if_t<!std::is_same<T, int>::value>> {
  using Wrapped = wrapped_const_ptr<T>;
  static Wrapped wrap(const T* arg) {
    return {arg};
  }
  static const T* unwrap(Wrapped arg_wrapped) {
    return arg_wrapped.ptr;
  }
};

// Wraps any `const T&` with `wrapped_const_ptr`, except for `int`.
// Leverage `const T&` logic, such that we'd get the default template when
// SFINAE prevents matching.
template <typename T>
struct wrap_example<const T&> : public wrap_example<const T*> {
  using base = wrap_example<const T*>;
  using base::Wrapped;
  static Wrapped wrap(const T& arg) { return base::wrap(&arg); }
  static const T& unwrap(Wrapped arg_wrapped) {
    return *base::unwrap(arg_wrapped);
  }
};

// Wraps any mutable `T&` with `wrapped_ptr`.
template <typename T>
struct wrap_example<T&> {
  static T* wrap(T& arg) { return &arg; }
  static T& unwrap(T* arg_wrapped) { return *arg; }
};

// N.B. We are NOT wrapping `T*`!

// Test case to exercise `WrapFunction`.
// Mappings:
//   `T*`       -> `T*` (no mapping)
//   `T&`       -> `wrapped_ptr<T>` (always).
//   `const T*` -> `wrapped_const_ptr<T>`, if `T` is not `int`.
//   `const T&` -> `wrapped_const_ptr<T>`, if `T` is not `int`.
template <typename Func>
auto WrapExample(Func&& func) {
  return WrapFunction<wrap_example>(std::forward<Func>(func));
}

}  // namespace example_wrapper

namespace example_functors {

// Test arguments that are move-only.
struct MoveOnlyValue {
  MoveOnlyValue() = default;
  MoveOnlyValue(const MoveOnlyValue&) = delete;
  MoveOnlyValue& operator=(const MoveOnlyValue&) = delete;
  MoveOnlyValue(MoveOnlyValue&&) = default;
  MoveOnlyValue& operator=(MoveOnlyValue&&) = default;
  int value{};
};

// Function with `void` return type, `int` by value.
// Expectation: Signature should remain unchanged.
void Func_1(int value) {}
// Function with a pointer return type, 
// Expectation: Signature should remain unchanged.
int* Func_2(int& value) { value += 1; return &value; }

const int& Func_3(const int& value) { return value; }
void Func_4(MoveOnlyValue value) {}
void Func_5(const int* value) {}

void Func_6(int& value, std::function<void (int&)> callback) {
  callback(value);
}

int& Func_7(int& value, const std::function<int& (int&)>& callback) {
  return callback(value);
}

class MyClass {
 public:
  static void Func(MoveOnlyValue&& value) {}
  void Method(MoveOnlyValue& value) { value.value += 2; }
  void Method_2(MoveOnlyValue& value) const { value.value += 3; }
};

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
  using example_wrapper::WrapExample;

  MoveOnlyValue v{10};

  WrapExample(Func_1)(v.value);
  EXPECT_EQ(v.value, 10);

  // EXPECT_EQ(*WrapExample(Func_2)({&v.value}), 0);
  // EXPECT_EQ(v.value, 11);

  // CHECK(cout << *WrapExample(Func_3)(&v.value));
  // CHECK(WrapExample(Func_4)(MoveOnlyValue{}));
  // CHECK(WrapExample(Func_5)(&v.value));

  // // Lambda.
  // auto void_ref = [](int& value) {
  //   value += 10;
  // };
  // CHECK(WrapExample(void_ref)(&v.value));

  // CHECK(WrapExample(MyClass::Func)(MoveOnlyValue{}));
  // MyClass c;
  // const MyClass& c_const{c};
  // CHECK(WrapExample(&MyClass::Method)(&c, &v));
  // CHECK(WrapExample(&MyClass::Method_2)(&c_const, &v));

  // MoveOnlyFunctor f;
  // CHECK(WrapExample(std::move(f))(&v));
  // ConstFunctor g;
  // CHECK(WrapExample(g)(&v));
  // const ConstFunctor& g_const{g};
  // CHECK(WrapExample(g_const)(&v));

  // // Callback.
  // CHECK(WrapExample(Func_6)(&v.value, WrapExample(void_ref)));

  // // Callback with return.
  // auto get_ref = [](int& value) -> int& {
  //   value += 100;
  //   return value;
  // };
  // CHECK(cout << *WrapExample(Func_7)(&v.value, WrapExample(get_ref)));

  // // Nested callback.
  // auto get_ref_nested = [get_ref](int& value,
  //     std::function<int& (int&, const std::function<int& (int&)>&)> func) -> auto& {
  //   value += 1000;
  //   return func(value, get_ref);
  // };
  // CHECK(cout << *WrapExample(get_ref_nested)(&v.value, WrapExample(Func_7)));
}

}  // namespace example_functors

}  // namespace
}  // namespace drake
