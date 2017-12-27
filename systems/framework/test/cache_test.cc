#include "drake/systems/framework/cache.h"

// Tests the Context (runtime) side of caching, which consists of a
// Cache object containing CacheEntryValue objects, each corresponding to one
// of a System's CacheEntry objects. User interaction with the cache is
// through the CacheEntry objects; we're not attempting to test CacheEntry
// here. However, we do need to instantiate some CacheEntry objects to satisfy
// the API that creates CacheEntryValues.

#include <memory>
#include <stdexcept>

#include <gtest/gtest.h>

#include "drake/common/text_logging.h"
#include "drake/systems/framework/system_base.h"
#include "drake/systems/framework/test_utilities/my_vector.h"
#include "drake/systems/framework/test_utilities/pack_value.h"
#include "drake/systems/framework/value.h"

using std::string;
using Eigen::Vector3d;

namespace drake {
namespace systems {
namespace {

// These are dummies required by the API.
auto alloc = [](const ContextBase&){return AbstractValue::Make<int>(3);};
auto calc = [](const ContextBase&, AbstractValue* result) {
  result->SetValue(99);
};


// This is so we can use the contained cache & dependency graph objects.
class MyContextBase : public ContextBase {
 public:
  MyContextBase() = default;
  MyContextBase(const MyContextBase&) = default;
  // Promote these to public.
  using ContextBase::get_mutable_dependency_graph;
  using ContextBase::get_mutable_cache;
  using ContextBase::start_new_change_event;
 private:
  MyContextBase* DoCloneWithoutPointers() const final {
    return new MyContextBase(*this);
  }
};

// We need just a little infrastructure from System to work with cache entries.
class MySystemBase : public SystemBase {
 public:
  // For use as a cache entry calculator.
  void CalcString(const MyContextBase& my_context, string* out) const {
    *out ="calculated_result";
  }
  void CalcMyVector3(const MyContextBase& my_context, MyVector3d* out) const {
    out->set_value(Vector3d(3., 2., 1.));
  }

 private:
  std::unique_ptr<ContextBase> DoMakeContext() const override {
    return std::make_unique<MyContextBase>();
  }
  // Dummies. We're not going to call these.
  void DoAcquireContextResources(ContextBase*) const override {}
  void DoCheckValidContext(const ContextBase&) const override {}
};

// Dependency chains:
//   all_sources -> cache0
//   cache0 -> cache1
//   cache0, cache1 -> cache2
//
//   time -> string_entry
//   xc, string_entry -> vector_entry
//
// Value types:
//   int: cache0,1,2
//   string: string_entry
//   MyVector3: vector_entry
class CacheTest : public ::testing::Test {
 protected:
  void SetUp() override {
    auto& cache_entry0 = system_.DeclareCacheEntry(
        "entry0", alloc, calc, {system_.all_sources_ticket()});
    index0_ = cache()
                  .CreateNewCacheEntryValue(cache_entry0, &trackers())
                  .cache_index();
    cache_value(index0_).SetInitialValue(PackValue(0));

    auto& cache_entry1 = system_.DeclareCacheEntry("entry1", alloc, calc,
                                                  {cache_entry0.ticket()});
    index1_ = cache()
                  .CreateNewCacheEntryValue(cache_entry1, &trackers())
                  .cache_index();
    cache_value(index1_).SetInitialValue(PackValue(1));

    auto& cache_entry2 = system_.DeclareCacheEntry("entry2", alloc, calc,
                                                  {cache_entry0.ticket(),
                                                   cache_entry1.ticket()});
    index2_ = cache()
                  .CreateNewCacheEntryValue(cache_entry2, &trackers())
                  .cache_index();
    cache_value(index2_).SetInitialValue(PackValue(2));

    // Set the initial values and mark them up to date.
    // Make the initial values up to date.
    cache_value(index0_).set_is_up_to_date(true);
    cache_value(index1_).set_is_up_to_date(true);
    cache_value(index2_).set_is_up_to_date(true);

    auto& string_entry = system_.DeclareCacheEntry("string thing",
                                               string("initial"),
                                               &MySystemBase::CalcString,
                                               {system_.time_ticket()});
    string_index_ = cache()
        .CreateNewCacheEntryValue(string_entry, &trackers())
        .cache_index();
    cache_value(string_index_).SetInitialValue(string_entry.Allocate(context_));
    EXPECT_FALSE(cache_value(string_index_).is_up_to_date());
    cache_value(string_index_).set_is_up_to_date(true);

    auto& vector_entry =
        system_.DeclareCacheEntry("vector thing", &MySystemBase::CalcMyVector3,
                                  {system_.xc_ticket(), string_entry.ticket()});
    vector_index_ = cache()
                        .CreateNewCacheEntryValue(vector_entry, &trackers())
                        .cache_index();
    cache_value(vector_index_).SetInitialValue(vector_entry.Allocate(context_));
    EXPECT_FALSE(cache_value(vector_index_).is_up_to_date());
    cache_value(vector_index_).set_value(MyVector3d(Vector3d(99., 98., 97.)));
    EXPECT_TRUE(cache_value(vector_index_).is_up_to_date());
  }

  static CacheEntryValue& cache_value(CacheIndex index, Cache* cache) {
    return cache->get_mutable_cache_entry_value(index);
  }

  CacheEntryValue& cache_value(CacheIndex index) {
    return cache_value(index, &cache());
  }
  DependencyTracker& tracker(CacheIndex index) {
    return tracker(cache_value(index).ticket());
  }
  void invalidate(CacheIndex index) {
    const int64_t event = context_.start_new_change_event();
    cache_value(index).set_is_up_to_date(false);
    tracker(index).NoteValueChange(event);
  }

  Cache& cache() { return context_.get_mutable_cache(); }
  DependencyGraph& trackers() {
    return context_.get_mutable_dependency_graph();
  }
  DependencyTracker& tracker(DependencyTicket dticket) {
    return trackers().get_mutable_tracker(dticket);
  }

  // Turn on logging before anything else happens.
  bool logging_enabled_ = []() {
    // log()->set_level(spdlog::level::trace);
    // log()->flush_on(spdlog::level::trace);
    return true;
  }();

  // Create a System and a Context to match.
  MySystemBase system_;
  std::unique_ptr<ContextBase> context_base_ = system_.AllocateContext();
  MyContextBase& context_ = dynamic_cast<MyContextBase&>(*context_base_);
  CacheIndex index0_, index1_, index2_;
  CacheIndex string_index_, vector_index_;
};

// Test that the Get/Calc/Eval() methods work.
TEST_F(CacheTest, ValueMethodsWork) {
  auto& cache_entry = system_.DeclareCacheEntry("get test", alloc, calc,
                                               {system_.nothing_ticket()});
  CacheIndex index = cache()
      .CreateNewCacheEntryValue(cache_entry, &trackers())
      .cache_index();
  cache_value(index).SetInitialValue(PackValue(42));

  CacheEntryValue& ce_value = cache_value(index);
  EXPECT_EQ(ce_value.cache_index(), index);
  EXPECT_TRUE(ce_value.ticket().is_valid());
  EXPECT_EQ(ce_value.description(), "get test");

  EXPECT_FALSE(ce_value.is_up_to_date());  // Initial value is not up to date.
  EXPECT_THROW(ce_value.GetValueOrThrow<int>(), std::logic_error);
  ce_value.set_is_up_to_date(true);
  EXPECT_NO_THROW(ce_value.GetValueOrThrow<int>());
  EXPECT_EQ(42, ce_value.get_value<int>());
  const AbstractValue& value =
      cache().get_cache_entry_value(index).GetAbstractValueOrThrow();
  EXPECT_EQ(42, UnpackIntValue(value));

  const auto& string_entry = system_.get_cache_entry(string_index_);
  EXPECT_EQ(string_entry.Get<string>(context_), "initial");

  // Check that the Calc() method produces output but doesn't change the
  // stored value.
  auto out = AbstractValue::Make<string>("something");
  string_entry.Calc(context_, out.get());
  EXPECT_EQ(out->GetValue<string>(), "calculated_result");
  EXPECT_EQ(string_entry.Get<string>(context_), "initial");

  // The value is currently marked up to date, so Eval does nothing.
  const auto& result = string_entry.Eval<string>(context_);
  EXPECT_EQ(result, "initial");
  // Force out-of-date.
  cache_value(string_index_).set_is_up_to_date(false);
  EXPECT_THROW(string_entry.GetAbstract(context_), std::logic_error);
  EXPECT_THROW(string_entry.Get<string>(context_), std::logic_error);
  (void)string_entry.Eval<string>(context_);
  EXPECT_TRUE(string_entry.is_up_to_date(context_));
  EXPECT_NO_THROW(string_entry.Get<string>(context_));

  // The result reference was updated by the Eval().
  EXPECT_EQ(result, "calculated_result");
  // Get() returns the same reference.
  EXPECT_EQ(&string_entry.Get<string>(context_), &result);

  // This is the wrong value type.
  EXPECT_THROW(string_entry.Get<int>(context_), std::logic_error);
}

// Check that a chain of dependent cache entries gets invalidated properly.
// More extensive testing of dependency tracking is in the unit test for
// dependency trackers.
TEST_F(CacheTest, InvalidationWorks) {
  EXPECT_TRUE(cache_value(index0_).is_up_to_date());
  EXPECT_TRUE(cache_value(index1_).is_up_to_date());
  EXPECT_TRUE(cache_value(index2_).is_up_to_date());
  EXPECT_TRUE(cache_value(string_index_).is_up_to_date());
  EXPECT_TRUE(cache_value(vector_index_).is_up_to_date());
  // This should invalidate everything.
  context_.get_tracker(system_.time_ticket()).NoteValueChange(99);
  EXPECT_FALSE(cache_value(index0_).is_up_to_date());
  EXPECT_FALSE(cache_value(index1_).is_up_to_date());
  EXPECT_FALSE(cache_value(index2_).is_up_to_date());
  EXPECT_FALSE(cache_value(string_index_).is_up_to_date());
  EXPECT_FALSE(cache_value(vector_index_).is_up_to_date());
}

// Test that the vector-valued cache entry works and preserved the underlying
// concrete type.
TEST_F(CacheTest, VectorCacheEntryWorks) {
  auto& entry = system_.get_cache_entry(vector_index_);
  auto& entry_value = entry.get_mutable_cache_entry_value(context_);
  EXPECT_TRUE(entry_value.is_up_to_date());  // We set it during construction.
  const MyVector3d& contents = entry_value.get_value<MyVector3d>();
  Vector3d eigen_contents = contents.get_value();
  EXPECT_EQ(eigen_contents, Vector3d(99., 98., 97.));

  // Force Eval to recalculate by pretending we modified a z, which should
  // invalidate this xc-dependent cache entry.
  context_.get_mutable_tracker(system_.z_ticket()).NoteValueChange(1001);
  EXPECT_FALSE(entry_value.is_up_to_date());
  const MyVector3d& contents2 = entry.Eval<MyVector3d>(context_);
  EXPECT_TRUE(entry_value.is_up_to_date());
  Vector3d eigen_contents2 = contents2.get_value();
  EXPECT_EQ(eigen_contents2, Vector3d(3., 2., 1.));
  // The contents object should still be the same one.
  EXPECT_EQ(&contents2, &contents);
}

// Test that we can swap in a new value if it has the right type, and that
// the swapped-in value is invalid immediately after swapping. In Debug,
// test that we throw if the swapped-in value is null or has the
// wrong type.
TEST_F(CacheTest, CanSwapValue) {
  auto& entry = system_.get_cache_entry(string_index_);
  auto& entry_value = entry.get_mutable_cache_entry_value(context_);
  EXPECT_TRUE(entry_value.is_up_to_date());  // Set to "initial".
  EXPECT_EQ(entry_value.get_value<string>(), "initial");
  auto new_value = AbstractValue::Make<string>("new value");
  entry_value.swap_value(&new_value);
  EXPECT_EQ(new_value->GetValue<string>(), "initial");
  EXPECT_FALSE(entry_value.is_up_to_date());
  entry_value.set_is_up_to_date(true);
  EXPECT_EQ(entry_value.get_value<string>(), "new value");

// In Debug builds, try a bad swap and expect it to be caught.
#ifdef DRAKE_ASSERT_IS_ARMED
  std::unique_ptr<AbstractValue> empty_ptr;
  EXPECT_THROW(entry_value.swap_value(&empty_ptr), std::logic_error);
  auto bad_value = AbstractValue::Make<int>(29);
  EXPECT_THROW(entry_value.swap_value(&bad_value), std::logic_error);
#endif
}

TEST_F(CacheTest, InvalidationIsRecursive) {
  invalidate(index1_);
  EXPECT_EQ(0, cache_value(index0_).get_value<int>());
  EXPECT_EQ(0, cache_value(index0_).GetValueOrThrow<int>());
  EXPECT_FALSE(cache_value(index1_).is_up_to_date());
  EXPECT_FALSE(cache_value(index2_).is_up_to_date());
}

TEST_F(CacheTest, Copy) {
  Cache copy(cache());
  // The copy should have the same number of cache entry values.
  EXPECT_EQ(copy.num_entries(), cache().num_entries());

  // The copy should have the same values.
  EXPECT_EQ(cache_value(index0_, &copy).GetValueOrThrow<int>(),
            cache_value(index0_).GetValueOrThrow<int>());
  EXPECT_EQ(cache_value(index1_, &copy).GetValueOrThrow<int>(),
            cache_value(index1_).GetValueOrThrow<int>());
  EXPECT_EQ(cache_value(index2_, &copy).GetValueOrThrow<int>(),
            cache_value(index2_).GetValueOrThrow<int>());
  EXPECT_EQ(cache_value(string_index_, &copy).GetValueOrThrow<string>(),
            cache_value(string_index_).GetValueOrThrow<string>());
  EXPECT_EQ(cache_value(vector_index_, &copy).GetValueOrThrow<MyVector3d>()
                                             .get_value(),
            cache_value(vector_index_).GetValueOrThrow<MyVector3d>()
                                      .get_value());

  // Although the copy has the same dependency ticket numbers, we did not
  // copy the trackers so there is currently nothing tracking values.
  // Check one of the tickets.
  EXPECT_EQ(cache_value(string_index_, &copy).ticket(),
            cache_value(string_index_).ticket());

  // Changes to the copy should not affect the original.
  cache_value(index2_, &copy).set_is_up_to_date(false);  // Invalidate.
  cache_value(index2_, &copy).set_value<int>(99);  // Set new value & validate.
  EXPECT_EQ(cache_value(index2_, &copy).get_value<int>(), 99);
  EXPECT_EQ(cache_value(index2_).get_value<int>(), 2);

  // This should invalidate everything in the original cache, but nothing
  // in the copy. Just check one entry as representative.
  context_.get_tracker(system_.time_ticket()).NoteValueChange(10);
  EXPECT_FALSE(cache_value(string_index_).is_up_to_date());
  EXPECT_TRUE(cache_value(string_index_, &copy).is_up_to_date());
}

}  // namespace
}  // namespace systems
}  // namespace drake
