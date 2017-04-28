#pragma once

#include <type_traits>

namespace drake {

// Borrowing from the future, when compatible with C++1y compilation
namespace std_future {

// Invert the value of B::value and return anologous structure
// @ref http://en.cppreference.com/w/cpp/types/negation
template<class B>
struct negation : std::integral_constant<bool, !B::value> { };

// @ref http://en.cppreference.com/w/cpp/types/conjunction
// Simplified
template<typename T, typename... Args>
struct conjunction
    : std::integral_constant<bool,
          T::value ? conjunction<Args...>::value : false> {};

template<typename T>
struct conjunction<T> : T { };

}  // namespace std_future

namespace detail {
// Check that all types are the same
// Consider using with std::decay<>, std::remove_cv<>, and/or
// std::is_convertible<> for robust simplification
template <typename T, typename... Args>
struct is_all_same : public std_future::conjunction<std::is_same<T, Args>...>
{};

// Check that all types are different
// @see is_all_same
template <typename T, typename... Args>
struct is_all_different
    : public std_future::conjunction<
          std_future::negation<std::is_same<T, Args>>...> {};
}  // namespace detail

}  // namespace drake
