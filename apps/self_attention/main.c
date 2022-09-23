// The APACHE License (APACHE)
// 
// Copyright (c) 2022 Xiaorui Yin. All rights reserved.
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
//   http://www.apache.org/licenses/LICENSE-2.0
// 
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <riscv_vector.h>
#include <stdio.h>
#include "kernel/self_attention.h"
#include "common/common.h"

#ifndef SPIKE
#include "printf.h"
#endif

#include "runtime.h"

extern const int n, d_model, dk;
extern float x[] __attribute__((aligned(32 * NR_LANES))); // matrix data (N x M) to normalize
extern float wq[] __attribute__((aligned(32 * NR_LANES))); // shift parameter
extern float q_bias[] __attribute__((aligned(32 * NR_LANES))); // bias parameter
extern float wk[] __attribute__((aligned(32 * NR_LANES))); // shift parameter
extern float k_bias[] __attribute__((aligned(32 * NR_LANES))); // bias parameter
extern float wv[] __attribute__((aligned(32 * NR_LANES))); // shift parameter
extern float v_bias[] __attribute__((aligned(32 * NR_LANES))); // bias parameter
extern float o_gold[] __attribute__((aligned(32 * NR_LANES)));
extern float o[] __attribute__((aligned(32 * NR_LANES)));

int main() {
  printf("\n");
  printf("========================\n");
  printf("=     Self-Attention   =\n");
  printf("========================\n");
  printf("\n");
  printf("\n");

#ifdef SPIKE
  // Enable V extension
  ENABLE_VEC;
#endif

#ifndef SPIKE
  start_timer();
  self_attention(x, o, wq, q_bias, wk, k_bias, wv, v_bias, n, d_model, dk, 0);
  stop_timer();
  
  // Performance metrics
  int64_t runtime = get_timer();
  float performance = (3*2*n*d_model*dk + 2*2*n*n*dk + 10*n*n) / (float)runtime;
  float utilization = 100 * performance / (2.0 * NR_LANES);

  printf("The execution took %d cycles.\n", runtime);
  printf("The performance is %f SPFLOP/cycle (%f%% utilization).\n",
        performance, utilization);
#else
  self_attention(x, o, wq, q_bias, wk, k_bias, wv, v_bias, n, d_model, dk, 0);
#endif

  printf("Verifying result\n");
  compare_matrix(o, o_gold, n, dk);
}
