#pragma once

// GENERATED FILE DO NOT EDIT
// This file contains docstrings for the Python bindings that were
// automatically extracted by mkdoc.py.

#if defined(__GNUG__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif

// Symbol: test_header_doc
constexpr struct /* test_header_doc */ {
  // Symbol: RootLevelSymbol
  struct /* RootLevelSymbol */ {
    // Source: drake/tools/workspace/pybind11/test/test_header.h:9
    const char* doc = R"""(Root-level symbol.)""";
  } RootLevelSymbol;
  // Symbol: drake
  struct /* drake */ {
    // Symbol: drake::MidLevelSymbol
    struct /* MidLevelSymbol */ {
      // Source: drake/tools/workspace/pybind11/test/test_header.h:14
      const char* doc = R"""(Mid-level symbol.)""";
    } MidLevelSymbol;
    // Symbol: drake::mkdoc_test
    struct /* mkdoc_test */ {
      // Symbol: drake::mkdoc_test::Class
      struct /* Class */ {
        // Source: drake/tools/workspace/pybind11/test/test_header.h:27
        const char* doc = R"""(Class.)""";
        // Symbol: drake::mkdoc_test::Class::Class
        struct /* ctor */ {
          // Source: drake/tools/workspace/pybind11/test/test_header.h:29
          const char* doc = R"""()""";
          // Source: drake/tools/workspace/pybind11/test/test_header.h:29
          const char* doc_2 = R"""()""";
          // Source: drake/tools/workspace/pybind11/test/test_header.h:41
          const char* doc_3 = R"""(Custom constructor 1.)""";
          // Source: drake/tools/workspace/pybind11/test/test_header.h:44
          const char* doc_4 = R"""(Custom constructor 2.)""";
          // Source: drake/tools/workspace/pybind11/test/test_header.h:47
          const char* doc_5 = R"""(Custom constructor 3.)""";
        } ctor;
        // Symbol: drake::mkdoc_test::Class::DRAKE_COPYABLE_DEMAND_COPY_CAN_COMPILE
        struct /* DRAKE_COPYABLE_DEMAND_COPY_CAN_COMPILE */ {
          // Source: drake/tools/workspace/pybind11/test/test_header.h:29
          const char* doc = R"""()""";
        } DRAKE_COPYABLE_DEMAND_COPY_CAN_COMPILE;
        // Symbol: drake::mkdoc_test::Class::Nested
        struct /* Nested */ {
          // Source: drake/tools/workspace/pybind11/test/test_header.h:64
          const char* doc = R"""(Protected nested class.)""";
        } Nested;
        // Symbol: drake::mkdoc_test::Class::ProtectedMethod
        struct /* ProtectedMethod */ {
          // Source: drake/tools/workspace/pybind11/test/test_header.h:61
          const char* doc = R"""(Protected method.)""";
        } ProtectedMethod;
        // Symbol: drake::mkdoc_test::Class::PublicMethod
        struct /* PublicMethod */ {
          // Source: drake/tools/workspace/pybind11/test/test_header.h:50
          const char* doc = R"""(Public method.)""";
        } PublicMethod;
        // Symbol: drake::mkdoc_test::Class::PublicStatic
        struct /* PublicStatic */ {
          // Source: drake/tools/workspace/pybind11/test/test_header.h:57
          const char* doc = R"""(Static method.)""";
        } PublicStatic;
        // Symbol: drake::mkdoc_test::Class::PublicTemplateMethod
        struct /* PublicTemplateMethod */ {
          // Source: drake/tools/workspace/pybind11/test/test_header.h:54
          const char* doc = R"""(Public template method.)""";
        } PublicTemplateMethod;
        // Symbol: drake::mkdoc_test::Class::protected_member_
        struct /* protected_member_ */ {
          // Source: drake/tools/workspace/pybind11/test/test_header.h:67
          const char* doc = R"""(Protected member.)""";
        } protected_member_;
      } Class;
      // Symbol: drake::mkdoc_test::Struct
      struct /* Struct */ {
        // Source: drake/tools/workspace/pybind11/test/test_header.h:78
        const char* doc = R"""(Struct.)""";
        // Symbol: drake::mkdoc_test::Struct::field_1
        struct /* field_1 */ {
          // Source: drake/tools/workspace/pybind11/test/test_header.h:80
          const char* doc = R"""(Field 1.)""";
        } field_1;
        // Symbol: drake::mkdoc_test::Struct::field_2
        struct /* field_2 */ {
          // Source: drake/tools/workspace/pybind11/test/test_header.h:82
          const char* doc = R"""(Field 2.)""";
        } field_2;
      } Struct;
      // Symbol: drake::mkdoc_test::Template
      struct /* Template */ {
        // Source: drake/tools/workspace/pybind11/test/test_header.h:87
        const char* doc = R"""(Template class.)""";
        // Source: drake/tools/workspace/pybind11/test/test_header.h:95
        const char* doc_2 = R"""(Specialize.)""";
        // Symbol: drake::mkdoc_test::Template::Template<T>
        struct /* ctor */ {
          // Source: drake/tools/workspace/pybind11/test/test_header.h:90
          const char* doc = R"""(Default constructor.)""";
        } ctor;
      } Template;
      // Symbol: drake::mkdoc_test::func
      struct /* func */ {
        // Source: drake/tools/workspace/pybind11/test/test_header.h:19
        const char* doc = R"""(Function.)""";
        // Source: drake/tools/workspace/pybind11/test/test_header.h:21
        const char* doc_2 = R"""(Function, overload 1.)""";
        // Source: drake/tools/workspace/pybind11/test/test_header.h:24
        const char* doc_3 = R"""(Function, template overload.)""";
      } func;
    } mkdoc_test;
  } drake;
} test_header_doc;

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
