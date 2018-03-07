#pragma once

#include <memory>
#include <type_traits>

namespace drake {

/** @cond */

/**
 This should generally be unused in public code; only use this class when you
 need to friend this class to permit private access to `Clone`. If you do this,
 please ensure that you use the `Token` parameter to use a private class token
 such that no other classes can gain backdoor access.
 */
template <typename T, typename Token = void>
struct cloneable_helper {
  template <typename U = T>
  static std::true_type Check(
      decltype(std::declval<U>().Clone())* ptr);
  static std::false_type Check(...);

  template <typename U = T>
  static auto Do(const U& other) { return other.Clone(); }
};

/**
 @anchor is_cloneable_doc
 Provides method for determining at run time if a class is "cloneable".

 __Usage__

 This gets used like `type_traits` functions (e.g., `is_copy_constructible`,
 `is_same`, etc.) To determine if a class is cloneable simply invoke:

 @code
 bool value = drake::is_cloneable<Foo>::value;
 @endcode

 If `Foo` is cloneable, it will evaluate to true. It can also be used in
 compile-time tests (e.g., SFINAE and `static_assert`s):

 @code
 static_assert(is_cloneable<Foo>::value, "This method requires its classes to "
                                         "be cloneable.");
 @endcode

 __Definition of "cloneability"__

 To be cloneable, the class `Foo` must have a public method of the form:

 @code
 unique_ptr<Foo> Foo::Clone() const;
 @endcode

 The pointer contained in the returned `unique_ptr` must point to a
 heap-allocated deep copy of the _concrete_ object. This test can confirm the
 proper signature, but cannot confirm the heap-allocated deep copy. A Clone()
 method that doesn't return such a copy of the _concrete_ object should be
 considered a malformed function.

 @warning It is important to note, that a `Clone()` method that returns a
 `unique_ptr` to a _super_ class is _not_ sufficient to be cloneable. In other
 words the presence of:
 @code
 unique_ptr<Base> Derived::Clone() const;
 @endcode
 will not make the `Derived` class cloneable.

 @tparam  T  The class to test for cloneability.
 */
template <typename T, typename Token = void>
using is_cloneable = decltype(cloneable_helper<T, Token>::Check(nullptr));

}  // namespace drake
