// Code in this file is inspired by:
// https://github.com/RobotLocomotion/drake/blob/master/common/test/hash_test.cc
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

#include "maliput/common/maliput_hash.h"

#include <vector>

#include <gtest/gtest.h>

namespace maliput {
namespace common {
namespace {

// A hasher which simply records all of its invocations for later inspection
class MockHasher {
 public:
  void operator()(const void* data, size_t length) noexcept {
    const uint8_t* const begin = static_cast<const uint8_t*>(data);
    const uint8_t* const end = begin + length;
    record_.emplace_back(begin, end);
  }

  std::vector<std::vector<uint8_t>> record() const { return record_; }

 private:
  std::vector<std::vector<uint8_t>> record_;
};

GTEST_TEST(HashTest, HashAppendOptional) {
  // Test basic functionality:  ensure two equal values get hashed the same way,
  // and that an empty value and non-empty value are hashed differently
  // (regardless of whether or not the hashes turn out the same).
  std::optional<int> nonempty1(99);
  std::optional<int> nonempty2(99);
  MockHasher hash_nonempty1;
  MockHasher hash_nonempty2;
  hash_append(hash_nonempty1, nonempty1);
  hash_append(hash_nonempty2, nonempty2);
  EXPECT_EQ(hash_nonempty1.record(), hash_nonempty2.record());
  EXPECT_EQ(static_cast<int>(hash_nonempty1.record().size()), 2);

  std::optional<int> empty1;
  std::optional<int> empty2;
  MockHasher hash_empty1;
  MockHasher hash_empty2;
  hash_append(hash_empty1, empty1);
  hash_append(hash_empty2, empty2);
  EXPECT_EQ(hash_empty1.record(), hash_empty2.record());
  EXPECT_EQ(static_cast<int>(hash_empty1.record().size()), 1);

  // We specifically want to ensure that the hasher is called in a different
  // way for the empty and non-empty cases.  Given that `hash_append` for
  // `int` and `bool` each invoke the hasher once, the following expectation
  // on total invocation counts reliably tests this:
  EXPECT_NE(hash_empty1.record().size(), hash_nonempty1.record().size());
}

}  // namespace
}  // namespace common
}  // namespace maliput
