// Copyright 2020 ETH Zurich and University of Bologna.
//
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


// Author : Navaneeth Kunhi Purayil (nkunhi@student.ethz.ch)
// Sample program to test load & stores in ARA cluster.

#ifndef SPIKE
#include "printf.h"
#else
#include <stdio.h>
#endif

extern double v64a[] __attribute__((aligned(32 * NR_LANES * NR_GROUPS), section(".l2")));
extern double v64b[] __attribute__((aligned(32 * NR_LANES * NR_GROUPS), section(".l2")));
extern float v32a[] __attribute__((aligned(32 * NR_LANES * NR_GROUPS), section(".l2")));
extern float v32b[] __attribute__((aligned(32 * NR_LANES * NR_GROUPS), section(".l2")));
extern _Float16 v16a[] __attribute__((aligned(32 * NR_LANES * NR_GROUPS), section(".l2")));
extern _Float16 v16b[] __attribute__((aligned(32 * NR_LANES * NR_GROUPS), section(".l2")));

int main() {
	int vl, avl=64;
	asm volatile("vsetvli %0, %1, e32, m1, ta, ma" : "=r"(vl) : "r"(avl));
	printf("vl:%d\n",vl);
	float *a_ = (float *) v32a;
	float *b_ = (float *) v32b;
	asm volatile("vle32.v v8,  (%0)" ::"r"(a_));
	asm volatile("vse32.v v8,  (%0)" ::"r"(b_));

	int err_cnt = 0;
	for (int i=0; i<avl; i++) {
		if (v32b[i] != v32a[i]) {
			printf("Error val:%f exp:%f\n", v32b[i], v32a[i]);
			return -1;
		}
	}
	if (err_cnt)
		printf("Failed!\n");
	else
		printf("Success!\n");
	return 0;
}