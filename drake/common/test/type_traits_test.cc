#include "drake/common/type_traits.h"

#include <type_traits>

#include <gtest/gtest.h>

namespace drake {
namespace {

using std::is_same;
using std::false_type;
using std::true_type;

using std_future::conjunction;
using std_future::negation;

using detail::is_all_same;
using detail::is_all_different;

// Verifies negation
GTEST_TEST(TypeTraits, Negation) {
  EXPECT_TRUE(negation<false_type>::value);
  EXPECT_FALSE(negation<true_type>::value);
}

// Verifies conjunction
GTEST_TEST(TypeTraits, Conjunction) {
  // True
  EXPECT_TRUE(conjunction<true_type>::value);
  // meta: Add extra parenthesis to permit proper CPP parsing of the macro call
  EXPECT_TRUE(( conjunction<true_type, true_type, true_type>::value ));
  // False
  EXPECT_FALSE(conjunction<false_type>::value);
  EXPECT_FALSE(( conjunction<false_type, true_type, true_type>::value ));
}

GTEST_TEST(TypeTraits, IsAll) {
  // Parameter pack expansion
  EXPECT_TRUE(( is_all_same<double,  double, double, double>::value ));
  EXPECT_FALSE(( is_all_same<double,  double, double, int>::value ));

  EXPECT_TRUE(( is_all_different<double,  char, double*, int>::value ));
  EXPECT_FALSE(( is_all_different<double,  char, double, int>::value ));
}

}  // anonymous namespace
}  // namespace drake
