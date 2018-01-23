#include <functional>
#include <type_traits>

#include <gtest/gtest.h>

#include "drake/bindings/pydrake/util/wrap_function.h"

using namespace std;

namespace drake {

// N.B. Anonymous namespace not used as it makes failure messages
// (static_assert) harder to interpret.

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
struct wrap_example : public wrap_arg_default<T> {
  // For `wrap_example<const T&>`.
  using Wrapped = T;
};

// Wraps any `const T*` with `const_ptr`, except for `int`.
// SFINAE. Could be achieved with specialization, but using to uphold SFINAE
// contract provided by `WrapFunction`.
template <typename T>
struct wrap_example<const T*, std::enable_if_t<!std::is_same<T, int>::value>> {
  // For `wrap_example<const T&>`.
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
struct wrap_example<const T&> : public wrap_example<const T*> {
  using base = wrap_example<const T*>;
  using Wrapped = typename base::Wrapped;

  static Wrapped wrap(const T& arg) { return base::wrap(&arg); }

  static const T& unwrap(Wrapped arg_wrapped) {
    return *base::unwrap(arg_wrapped);
  }
};

template <typename T>
struct wrap_example_mutable_ptr {
  static ptr<T> wrap(T* arg) { return {arg}; }
  static T* unwrap(ptr<T> arg_wrapped) { return arg_wrapped.value; }
};

// Wraps any mutable `T*` with `ptr`.
// N.B. Prevent `const T*` from binding here, since it may be rejected from
// SFINAE.
template <typename T>
struct wrap_example<T*, std::enable_if_t<!std::is_const<T>::value>>
    : public wrap_example_mutable_ptr<T> {};

// Wraps any mutable `T&` with `ptr`.
template <typename T>
struct wrap_example<T&> {
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
auto WrapExample(Func&& func) {
  return WrapFunction<wrap_example>(std::forward<Func>(func));
}

template <typename T>
using wrap_arg_t =
    detail::wrap_function_impl<wrap_example>::wrap_arg_t<T>;

template <typename Expected, typename Actual>
void check_type() {
  // Use this function to inspect types when failure is encountered.
  static_assert(std::is_same<Actual, Expected>::value, "Mismatch");
}

GTEST_TEST(WrapFunction, WrapCheck) {
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
// Wrapped signature: Unchanged.
void Func_1(int value) {}

// Function with a pointer return type, 
// Wrapped signature: `ptr<int> (ptr<int>)`
// int* Func_2(int& value) {
//   value += 1;
//   return &value;
// }

// Specialized types.
// Wrapped signature: `const int* (const int*)`
// const int& Func_3(const int& value) { return value; }

// void Func_4(MoveOnlyValue value) {}
// void Func_5(const int* value) {}

// void Func_6(int& value, std::function<void (int&)> callback) {
//   callback(value);
// }

// int& Func_7(int& value, const std::function<int& (int&)>& callback) {
//   return callback(value);
// }

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
  MoveOnlyValue v{0};

  {
    WrapExample(Func_1)(v.value);
    EXPECT_EQ(v.value, 0);
  }

  // {
  //   auto out = WrapExample(Func_2)(ptr<int>{&v.value});
  //   EXPECT_EQ(*out.value, 1);
  //   EXPECT_EQ(v.value, 1);
  // }

  // {
  //   auto out = WrapExample(Func_3)(&v.value);
  //   // Mutate to ensure we have a pointer.
  //   v.value += 1;
  //   EXPECT_EQ(*out, 2);
  //   EXPECT_EQ(v.value, 2);
  // }
  
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

}  // namespace drake
