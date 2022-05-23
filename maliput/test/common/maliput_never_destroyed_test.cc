// Code in this file is inspired by:
// https://github.com/RobotLocomotion/drake/blob/master/common/test/never_destroyed_test.cc
//
// Drake's license follows:
//
// All components of Drake are licensed under the BSD 3-Clause License
// shown below. Where noted in the source code, some portions may
// be subject to other permissive, non-viral licenses.
//
// Copyright 2012-2016 Robot Locomotion Group @ CSAIL
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.  Redistributions
// in binary form must reproduce the above copyright notice, this list of
// conditions and the following disclaimer in the documentation and/or
// other materials provided with the distribution.  Neither the name of
// the Massachusetts Institute of Technology nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2019-2022, Toyota Research Institute. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "maliput/common/maliput_never_destroyed.h"

#include <exception>
#include <unordered_map>

#include <gtest/gtest.h>

#include "maliput/common/maliput_copyable.h"

namespace maliput {
namespace common {
namespace {

class Boom : public std::exception {};
struct DtorGoesBoom {
  ~DtorGoesBoom() noexcept(false) { throw Boom(); }
};

// Confirm that we see booms by default.
GTEST_TEST(NeverDestroyedTest, BoomTest) {
  try {
    { DtorGoesBoom foo; }
    GTEST_FAIL();
  } catch (const Boom&) {
    ASSERT_TRUE(true);
  }
}

// Confirm that our wrapper stops the booms.
GTEST_TEST(NeverDestroyedTest, NoBoomTest) {
  try {
    { never_destroyed<DtorGoesBoom> foo; }
    ASSERT_TRUE(true);
  } catch (const Boom& e) {
    GTEST_FAIL();
  }
}

// This is an example from the class overview API docs; we repeat it here to
// ensure it remains valid.
class Singleton {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(Singleton)
  static Singleton& getInstance() {
    static never_destroyed<Singleton> instance;
    return instance.access();
  }

 private:
  friend never_destroyed<Singleton>;
  Singleton() = default;
};
GTEST_TEST(NeverDestroyedExampleTest, Singleton) {
  const Singleton* get1 = &Singleton::getInstance();
  const Singleton* get2 = &Singleton::getInstance();
  EXPECT_EQ(get1, get2);
}

// This is an example from the class overview API docs; we repeat it here to
// ensure it remains valid.
enum class Foo { kBar, kBaz };
Foo ParseFoo(const std::string& foo_string) {
  using Dict = std::unordered_map<std::string, Foo>;
  static const maliput::common::never_destroyed<Dict> string_to_enum{std::initializer_list<Dict::value_type>{
      {"bar", Foo::kBar},
      {"baz", Foo::kBaz},
  }};
  return string_to_enum.access().at(foo_string);
}
GTEST_TEST(NeverDestroyedExampleTest, ParseFoo) {
  EXPECT_EQ(ParseFoo("bar"), Foo::kBar);
  EXPECT_EQ(ParseFoo("baz"), Foo::kBaz);
}

}  // namespace
}  // namespace common
}  // namespace maliput
