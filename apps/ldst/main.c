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
// Sample program to test basic LdSt/Slide/Reduction instructions for the ARA cluster.

#ifndef SPIKE
#include "printf.h"
#else
#include <stdio.h>
#endif

#define T double 
// #define T float
// #define T _Float16

extern T va[] __attribute__((aligned(32 * NR_LANES * NR_CLUSTERS), section(".l2")));
extern T vb[] __attribute__((aligned(32 * NR_LANES * NR_CLUSTERS), section(".l2")));
extern int vsize;

// #define LDST_TEST  1
// #define SLIDEDOWN_TEST 1
#define SLIDEUP_TEST 1
// #define REDUCTION_TEST 1

#define FP64 1
// #define FP32 1
// #define FP16 1

#ifdef LDST_TEST

int main() {
	printf("============Load/Store Test============\n");
	int vl, avl=vsize;
	int shift = 1;

	T *a_ = (T *) va+shift;
	T *b_ = (T *) vb;

#ifdef FP64
	asm volatile("vsetvli %0, %1, e64, m4, ta, ma" : "=r"(vl) : "r"(avl)); // FP64
	printf("vl:%d\n",vl);
	asm volatile("vle64.v v8,  (%0)" ::"r"(a_));  // FP64
	asm volatile("vse64.v v8,  (%0)" ::"r"(b_));  // FP64
#elif defined(FP32)
	asm volatile("vsetvli %0, %1, e32, m4, ta, ma" : "=r"(vl) : "r"(avl)); // FP32
	printf("vl:%d\n",vl);
	asm volatile("vle32.v v8,  (%0)" ::"r"(a_));  // FP32
	asm volatile("vse32.v v8,  (%0)" ::"r"(b_));  // FP32
#elif defined(FP16)
	asm volatile("vsetvli %0, %1, e16, m4, ta, ma" : "=r"(vl) : "r"(avl)); // FP16
	printf("vl:%d\n",vl);
	asm volatile("vle16.v v8,  (%0)" ::"r"(a_));  // FP16
	asm volatile("vse16.v v8,  (%0)" ::"r"(b_));  // FP16
#endif
	for (int i=0; i<avl; i++) {
		if (vb[i] != va[i+shift]) {
			printf("Error idx:%d val:%f exp:%f\n", i, vb[i], va[i+shift]);
			return -1;
		}
	}
	return 0;
}
#endif

#ifdef SLIDEDOWN_TEST

int main() {
	printf("============Slide1down Test============\n");
	int vl, avl=vsize;

	int shift = 1;
	T *a_ = (T *) va+shift;
	T *b_ = (T *) vb;

	T scal=1.25;

#ifdef FP64
	asm volatile("vsetvli %0, %1, e64, m4, ta, ma" : "=r"(vl) : "r"(avl)); // FP64
	printf("vl:%d\n",vl);
	asm volatile("vle64.v v8,  (%0)" ::"r"(a_));  // FP64
	asm volatile("vfslide1down.vf v12, v8, %0" ::"f"(scal));
	asm volatile("vse64.v v12,  (%0)" ::"r"(b_));  // FP64
#elif defined(FP32)
	asm volatile("vsetvli %0, %1, e32, m4, ta, ma" : "=r"(vl) : "r"(avl)); // FP32
	printf("vl:%d\n",vl);
	asm volatile("vle32.v v8,  (%0)" ::"r"(a_));  // FP32
	asm volatile("vfslide1down.vf v12, v8, %0" ::"f"(scal));
	asm volatile("vse32.v v12,  (%0)" ::"r"(b_));  // FP32
#elif defined(FP16)
	asm volatile("vsetvli %0, %1, e16, m4, ta, ma" : "=r"(vl) : "r"(avl)); // FP16
	printf("vl:%d\n",vl);
	asm volatile("vle16.v v8,  (%0)" ::"r"(a_));  // FP16
	asm volatile("vfslide1down.vf v12, v8, %0" ::"f"(scal));
	asm volatile("vse16.v v12,  (%0)" ::"r"(b_));  // FP16
#endif

	for (int i=0; i<avl-1; i++) {
	if (vb[i] != va[i+shift+1]) {
			printf("Error idx:%d val:%f exp:%f\n", i, vb[i], va[i+shift+1]);
			return -1;
		}
	}
	if (vb[avl-1]!=scal) {
		printf("Error idx:%d val:%f exp:%f\n", avl-1, vb[avl-1], scal);
		return -1;
	}
	return 0;
}

#endif

#ifdef SLIDEUP_TEST

int main() {
	printf("============Slide1up Test============\n");
	int vl, avl=vsize;
	int shift = 0;
	T *a_ = (T *) va+shift;
	T *b_ = (T *) vb;

  	T scal=1.25;
  
#ifdef FP64
	asm volatile("vsetvli %0, %1, e64, m4, ta, ma" : "=r"(vl) : "r"(avl)); // FP64
	printf("vl:%d\n",vl);
	asm volatile("vle64.v v8,  (%0)" ::"r"(a_));  // FP64
	asm volatile("vfslide1up.vf v12, v8, %0" ::"f"(scal));
	asm volatile("vse64.v v12,  (%0)" ::"r"(b_));  // FP64
#elif defined(FP32)
	asm volatile("vsetvli %0, %1, e32, m4, ta, ma" : "=r"(vl) : "r"(avl)); // FP32
	printf("vl:%d\n",vl);
	asm volatile("vle32.v v8,  (%0)" ::"r"(a_));  // FP32
	asm volatile("vfslide1up.vf v12, v8, %0" ::"f"(scal));
	asm volatile("vse32.v v12,  (%0)" ::"r"(b_));  // FP32
#elif defined(FP16)
	asm volatile("vsetvli %0, %1, e16, m4, ta, ma" : "=r"(vl) : "r"(avl)); // FP16
	printf("vl:%d\n",vl);
	asm volatile("vle16.v v8,  (%0)" ::"r"(a_));  // FP16
	asm volatile("vfslide1up.vf v12, v8, %0" ::"f"(scal));
	asm volatile("vse16.v v12,  (%0)" ::"r"(b_));  // FP16
#endif

	for (int i=1; i<avl; i++) {
    if (vb[i] != va[i+shift-1]) {
			printf("Error idx:%d val:%f exp:%f\n", i, vb[i], va[i+shift-1]);
		}
	}
	if (vb[0]!=scal) {
		printf("Error idx:%d val:%f exp:%f\n", 0, vb[0], scal);
	}
	return 0;
}

#endif

#ifdef REDUCTION_TEST
extern T red64, red32, red16;

int main() {
	printf("============Reduction Test============\n");
	int vl, avl=vsize;

	T *a_ = (T *) va;

	float red;

#ifdef FP64
	asm volatile("vsetvli %0, %1, e64, m4, ta, ma" : "=r"(vl) : "r"(avl)); // FP64
	printf("vl:%d\n",vl);
	asm volatile("vmv.s.x v0, zero");
	asm volatile("vle64.v v8,  (%0)" ::"r"(a_));  // FP64
#elif defined(FP32)
	asm volatile("vsetvli %0, %1, e32, m4, ta, ma" : "=r"(vl) : "r"(avl)); // FP32
	printf("vl:%d\n",vl);
	asm volatile("vmv.s.x v0, zero");
	asm volatile("vle32.v v8,  (%0)" ::"r"(a_));  // FP32
#elif defined(FP16)
	asm volatile("vsetvli %0, %1, e16, m4, ta, ma" : "=r"(vl) : "r"(avl)); // FP16
	printf("vl:%d\n",vl);
	asm volatile("vmv.s.x v0, zero");
	asm volatile("vle16.v v8,  (%0)" ::"r"(a_));  // FP16
#endif

	asm volatile("vfredusum.vs v0, v8, v0");
	asm volatile("vfmv.f.s %0, v0" : "=f"(red));

#ifdef FP64
	printf("Res:%f Exp:%f\n", red, red64);
#elif defined(FP32)
	printf("Res:%f Exp:%f\n", red, red32);
#elif defined(FP16)
	printf("Res:%f Exp:%f\n", red, red16);
#endif
  	
}

#endif


