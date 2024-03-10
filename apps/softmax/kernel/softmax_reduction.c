#include <stdint.h>
#include <string.h>

void softmax_vec_reduction(const double *i, const double *o, uint64_t channels,
                 uint64_t innerSize) {

  size_t avl = innerSize;
  size_t vl;

  double *i_ = (double *) i;
  double *o_ = (double *) o;
  
  float max, exp_sum;

  asm volatile("vsetvli %0, %1, e64, m1, ta, ma" : "=r"(vl) : "r"(avl)); // FP64
  
  for (uint64_t c=0; c < channels; c++) {
    
    asm("vle64.v v1, (%0)" ::"r"(i_)); // v1 - vec
    asm("vmv.s.x v2, zero");
    asm("vfredmax.vs v3, v1, v2");    // v2 - max value of vector
    asm volatile("vfmv.f.s %0, v3" : "=f"(max));
    asm volatile("vfsub.vf v30, v1, %0"::"f"(max));
    i_ += vl;

    ///// Start of exp() compute
    // Compute

    // v30 x
    // v14 tmp3
    // v15 tmp
    // v0  mask
    // v16 tmp2
    // v17 z
    // v5  y
    // v18 imm0
    // v19 tmpmv

    // Load vector
    // asm("vle64.v v30, (%0)" ::"r"(i));
    asm("vfmv.v.f v31, %0" ::"f"(88.3762626647949) : "v31"); // exp_hi
    asm("vfmin.vv     v30, v31, v30" ::: "v30");
    asm("vfmv.v.f v1,  %0" ::"f"(-88.3762626647949) : "v1"); // exp_lo
    asm("vfmax.vv     v30, v1, v30" ::: "v30");
    asm("vfmv.v.f v2,  %0" ::"f"(1.44269504088896341)
        : "v2");                                // cephes_LOG2EF
    asm("vfmv.v.f v13, %0" ::"f"(0.5) : "v13"); // fx
    asm("vfmacc.vv    v13, v30, v2" ::: "v13");
    asm("vfmv.v.f v3,  %0" ::"f"(0.693359375) : "v3"); // cephes_exp_C1
    asm("vfcvt.x.f.v  v14, v13" ::: "v14");
    asm("vfmv.v.f v4,  %0" ::"f"(-2.12194440e-4) : "v4"); // cephes_exp_C2
    asm("vfcvt.f.x.v  v15, v14" ::: "v15");
    asm("vfmv.v.f v5,  %0" ::"f"(1.9875691500E-4) : "v5"); // cephes_exp_p0
    asm("vmflt.vv v0, v13, v15" ::: "v0");
    asm("vfmv.v.f v6,  %0" ::"f"(1.3981999507E-3) : "v6"); // cephes_exp_p1
    asm("vmv.v.i  v12, 0" ::: "v12");                      // zero
    asm("vmerge.vvm v16, v12, v1, v0" ::: "v16");
    asm("vfmv.v.f v7,  %0" ::"f"(8.3334519073E-3) : "v7"); // cephes_exp_p2
    asm("vfsub.vv v13, v15, v6" ::: "v13");
    asm("vfmv.v.f v8,  %0" ::"f"(4.1665795894E-2) : "v8"); // cephes_exp_p3
    asm("vfmul.vv v15, v13, v3" ::: "v15");
    asm("vfmv.v.f v9,  %0" ::"f"(1.6666665459E-1) : "v9"); // cephes_exp_p4
    asm("vfmul.vv v17, v13, v4" ::: "v17");
    asm("vfmv.v.f v10, %0" ::"f"(5.0000001201E-1) : "v10"); // cephes_exp_p5
    asm("vfmul.vv v17, v13, v4" ::: "v17");
    asm("vfmv.v.f v11, %0" ::"f"(1.0) : "v11"); // one

    asm("vfmul.vv v17, v13, v4" ::: "v17");
    asm("vfsub.vv v30, v30, v15" ::: "v30");
    asm("vfsub.vv v30, v30, v17" ::: "v30");
    asm("vfmul.vv v17, v30, v30" ::: "v17");

    asm("vfmadd.vv  v5, v30, v6" ::: "v5");
    asm("vfmadd.vv  v5, v30, v7" ::: "v5");
    asm("vfmadd.vv  v5, v30, v8" ::: "v5");
    asm("vfmadd.vv  v5, v30, v9" ::: "v5");
    asm("vfmadd.vv  v5, v30, v10" ::: "v5");
    asm("vfmadd.vv  v5, v17, v30" ::: "v5");

    asm("vfadd.vv v5, v5, v11" ::: "v5");

    asm("vfcvt.x.f.v  v18, v13" ::: "v18");
    asm("vmv.v.x v19, %0" ::"r"(1023) : "v19");
    asm("vadd.vv v18, v18, v19" ::: "v8");
    asm("vmv.v.x v19, %0" ::"r"(52) : "v19");
    asm("vsll.vv v18, v18, v19" ::: "v18");
    asm("vfmul.vv v5, v5, v18" ::: "v5");

    // Store
    // asm("vse64.v v5, (%0)" ::"r"(o));

    ///// End of exp() compute
    asm("vmv.s.x v4, zero");
    asm("vfredusum.vs v4, v5, v4");
    asm volatile("vfmv.f.s %0, v4" : "=f"(exp_sum));
    asm volatile("vfdiv.vf v6, v5, %0" :: "f"(exp_sum));

    asm("vse64.v v6, (%0)" :: "r"(o_));
    o_ += vl;
  }

}