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
#include "maliput/common/passkey.h"

#include <gtest/gtest.h>

namespace maliput {
namespace common {
namespace test {
namespace {

// Foo and Bar classes below and the test show how the method level friendship
// works.

class Foo;

class Bar {
 public:
  Bar() = default;
  int get_value() const { return value_; }
  void set_value(Passkey<Foo>, int value) { value_ = value; }

 private:
  int value_{};
};

class Foo {
 public:
  Foo() = default;
  void SetValueToBar(int value, Bar* bar) const { bar->set_value({}, value); }
};

GTEST_TEST(PasskeyTest, PasskeyIdiom) {
  Bar bar;
  EXPECT_EQ(bar.get_value(), 0);

  const Foo foo;
  foo.SetValueToBar(123, &bar);
  EXPECT_EQ(bar.get_value(), 123);
}

}  // namespace
}  // namespace test
}  // namespace common
}  // namespace maliput
