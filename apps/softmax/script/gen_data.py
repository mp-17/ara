#!/usr/bin/env python3
# Copyright 2021 ETH Zurich and University of Bologna.
#
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# arg1: vector size, arg2: filter size

import random as rand
import numpy as np
import sys

def emit(name, array, alignment='8'):
  print(".global %s" % name)
  print(".balign " + alignment)
  print("%s:" % name)
  bs = array.tobytes()
  for i in range(0, len(bs), 4):
    s = ""
    for n in range(4):
      s += "%02x" % bs[i+3-n]
    print("    .word 0x%s" % s)

def rand_matrix(N, dtype):
  return np.random.rand(N).astype(dtype)

############
## SCRIPT ##
############

if len(sys.argv) == 3:
  channels = int(sys.argv[1])
  innerSize = int(sys.argv[2])
else:
  print("Error. Give me two arguments: the number of channels and the inner size.")
  sys.exit()

dtype=np.float64

# Vector of samples
i = rand_matrix(channels * innerSize, dtype).astype(dtype)

# Results buffer
buf = np.zeros(channels * innerSize, dtype=dtype)
o_s = np.zeros(channels * innerSize, dtype=dtype)
o_g = np.zeros(channels * innerSize, dtype=dtype)
for c in range(channels):
  inp = i[c*innerSize : (c+1)*innerSize]
  i_ = inp - np.max(inp)
  o_m = np.exp(i_, dtype=dtype)
  s = np.sum(o_m)
  o_g[c*innerSize : (c+1)*innerSize] = o_m / s

# Create the file
print(".section .data,\"aw\",@progbits")
emit("channels", np.array(channels, dtype=np.uint64))
emit("innerSize", np.array(innerSize, dtype=np.uint64))
emit("i", i, 'NR_LANES*4*NR_CLUSTERS')
emit("buf", i, 'NR_LANES*4*NR_CLUSTERS')
emit("o_s", i, 'NR_LANES*4*NR_CLUSTERS')
emit("o_v", i, 'NR_LANES*4*NR_CLUSTERS')
emit("o_g", o_g, 'NR_LANES*4*NR_CLUSTERS')