#include <stdint.h>
#include <string.h>
#include <riscv_vector.h>
#include "../softmax/lib/exp.h"

/*void softmax_vec_reduction(const double *i, const double *o, uint64_t channels,
                 uint64_t innerSize) {

  size_t avl = innerSize;
  size_t vl;

  double *i_ = (double *) i;
  double *o_ = (double *) o;
  
  double max, exp_sum;

  asm volatile("vsetvli %0, %1, e64, m1, ta, ma" : "=r"(vl) : "r"(avl)); // FP64
  asm volatile("vle64.v v21, (%0)"  ::"r"(i_));

  for (uint64_t c=0; c < channels; c+=2) {
    asm volatile("vmv.s.x v22, zero");
    asm volatile("vfredmax.vs v23, v21, v22");    // v2 - max value of vector
    asm volatile("vfmv.f.s %0, v23" : "=f"(max));
    asm volatile("vfsub.vf v30, v21, %0"::"f"(max));
    i_ += vl;

    
    ///// Start of exp() compute
    asm volatile("vfmv.v.f v31, %0" ::"f"(88.3762626647949) : "v31"); // exp_hi
    asm volatile("vfmin.vv     v30, v31, v30" ::: "v30");
    asm volatile("vfmv.v.f v1,  %0" ::"f"(-88.3762626647949) : "v1"); // exp_lo
    asm volatile("vfmax.vv     v30, v1, v30" ::: "v30");
    asm volatile("vfmv.v.f v2,  %0" ::"f"(1.44269504088896341) : "v2"); // cephes_LOG2EF
    asm volatile("vfmv.v.f v13, %0" ::"f"(0.5) : "v13"); // fx
    asm volatile("vfmacc.vv    v13, v30, v2" ::: "v13");
    asm volatile("vfmv.v.f v3,  %0" ::"f"(0.693359375) : "v3"); // cephes_exp_C1
    asm volatile("vfcvt.x.f.v  v14, v13" ::: "v14");
    asm volatile("vfmv.v.f v4,  %0" ::"f"(-2.12194440e-4) : "v4"); // cephes_exp_C2
    asm volatile("vfcvt.f.x.v  v15, v14" ::: "v15");
    asm volatile("vfmv.v.f v5,  %0" ::"f"(1.9875691500E-4) : "v5"); // cephes_exp_p0
    asm volatile("vmflt.vv v0, v13, v15" ::: "v0");
    asm volatile("vfmv.v.f v6,  %0" ::"f"(1.3981999507E-3) : "v6"); // cephes_exp_p1
    asm volatile("vmv.v.i  v12, 0" ::: "v12");                      // zero
    asm volatile("vmerge.vvm v16, v12, v1, v0" ::: "v16");
    asm volatile("vfmv.v.f v7,  %0" ::"f"(8.3334519073E-3) : "v7"); // cephes_exp_p2
    asm volatile("vfsub.vv v13, v15, v6" ::: "v13");
    asm volatile("vfmv.v.f v8,  %0" ::"f"(4.1665795894E-2) : "v8"); // cephes_exp_p3
    asm volatile("vfmul.vv v15, v13, v3" ::: "v15");
    asm volatile("vfmv.v.f v9,  %0" ::"f"(1.6666665459E-1) : "v9"); // cephes_exp_p4
    asm volatile("vfmul.vv v17, v13, v4" ::: "v17");
    asm volatile("vfmv.v.f v10, %0" ::"f"(5.0000001201E-1) : "v10"); // cephes_exp_p5
    asm volatile("vfmul.vv v17, v13, v4" ::: "v17");
    asm volatile("vfmv.v.f v11, %0" ::"f"(1.0) : "v11"); // one

    asm volatile("vfmul.vv v17, v13, v4" ::: "v17");
    asm volatile("vfsub.vv v30, v30, v15" ::: "v30");
    asm volatile("vfsub.vv v30, v30, v17" ::: "v30");
    asm volatile("vfmul.vv v17, v30, v30" ::: "v17");
    
    if ((c+1) < channels) {
      asm volatile("vle64.v v21, (%0)"  ::"r"(i_));
      asm volatile("vmv.s.x v22, zero");
    }

    asm volatile("vfmadd.vv  v5, v30, v6" ::: "v5");
    asm volatile("vfmadd.vv  v5, v30, v7" ::: "v5");
    asm volatile("vfmadd.vv  v5, v30, v8" ::: "v5");
    asm volatile("vfmadd.vv  v5, v30, v9" ::: "v5");
    asm volatile("vfmadd.vv  v5, v30, v10" ::: "v5");
    asm volatile("vfmadd.vv  v5, v17, v30" ::: "v5");

    if ((c+1) < channels) {
      asm volatile("vfredmax.vs v23, v21, v22");    // v2 - max value of vector
      asm volatile("vfmv.f.s %0, v23" : "=f"(max));
      i_ += vl;
    }

    asm volatile("vfadd.vv v5, v5, v11" ::: "v5");

    asm volatile("vfcvt.x.f.v  v18, v13" ::: "v18");
    asm volatile("vmv.v.x v19, %0" ::"r"(1023) : "v19");
    asm volatile("vadd.vv v18, v18, v19" ::: "v8");
    asm volatile("vmv.v.x v19, %0" ::"r"(52) : "v19");
    asm volatile("vsll.vv v18, v18, v19" ::: "v18");
    asm volatile("vfmul.vv v5, v5, v18" ::: "v5");

    ///// End of exp() compute
    
    asm volatile("vmv.s.x v4, zero");
    asm volatile("vfredusum.vs v4, v5, v4");
    
    // Unroll-2
    if ((c+1) < channels) {
      asm volatile("vfsub.vf v30, v21, %0"::"f"(max));

      asm volatile("vfmv.f.s %0, v4" : "=f"(exp_sum));
      asm volatile("vfdiv.vf v6, v5, %0" :: "f"(exp_sum));
      asm volatile("vse64.v v6, (%0)" :: "r"(o_));
      o_ += vl;
      
      ///// Start of exp() compute

      asm volatile("vfmv.v.f v31, %0" ::"f"(88.3762626647949) : "v31"); // exp_hi
      asm volatile("vfmin.vv     v30, v31, v30" ::: "v30");
      asm volatile("vfmv.v.f v1,  %0" ::"f"(-88.3762626647949) : "v1"); // exp_lo
      asm volatile("vfmax.vv     v30, v1, v30" ::: "v30");
      asm volatile("vfmv.v.f v2,  %0" ::"f"(1.44269504088896341) : "v2"); // cephes_LOG2EF
      asm volatile("vfmv.v.f v13, %0" ::"f"(0.5) : "v13"); // fx
      asm volatile("vfmacc.vv    v13, v30, v2" ::: "v13");
      asm volatile("vfmv.v.f v3,  %0" ::"f"(0.693359375) : "v3"); // cephes_exp_C1
      asm volatile("vfcvt.x.f.v  v14, v13" ::: "v14");
      asm volatile("vfmv.v.f v4,  %0" ::"f"(-2.12194440e-4) : "v4"); // cephes_exp_C2
      asm volatile("vfcvt.f.x.v  v15, v14" ::: "v15");
      asm volatile("vfmv.v.f v5,  %0" ::"f"(1.9875691500E-4) : "v5"); // cephes_exp_p0
      asm volatile("vmflt.vv v0, v13, v15" ::: "v0");
      asm volatile("vfmv.v.f v6,  %0" ::"f"(1.3981999507E-3) : "v6"); // cephes_exp_p1
      asm volatile("vmv.v.i  v12, 0" ::: "v12");                      // zero
      asm volatile("vmerge.vvm v16, v12, v1, v0" ::: "v16");
      asm volatile("vfmv.v.f v7,  %0" ::"f"(8.3334519073E-3) : "v7"); // cephes_exp_p2
      asm volatile("vfsub.vv v13, v15, v6" ::: "v13");
      asm volatile("vfmv.v.f v8,  %0" ::"f"(4.1665795894E-2) : "v8"); // cephes_exp_p3
      asm volatile("vfmul.vv v15, v13, v3" ::: "v15");
      asm volatile("vfmv.v.f v9,  %0" ::"f"(1.6666665459E-1) : "v9"); // cephes_exp_p4
      asm volatile("vfmul.vv v17, v13, v4" ::: "v17");
      asm volatile("vfmv.v.f v10, %0" ::"f"(5.0000001201E-1) : "v10"); // cephes_exp_p5
      asm volatile("vfmul.vv v17, v13, v4" ::: "v17");
      asm volatile("vfmv.v.f v11, %0" ::"f"(1.0) : "v11"); // one

      asm volatile("vfmul.vv v17, v13, v4" ::: "v17");
      asm volatile("vfsub.vv v30, v30, v15" ::: "v30");
      asm volatile("vfsub.vv v30, v30, v17" ::: "v30");
      asm volatile("vfmul.vv v17, v30, v30" ::: "v17");

      asm volatile("vfmadd.vv  v5, v30, v6" ::: "v5");
      asm volatile("vfmadd.vv  v5, v30, v7" ::: "v5");
      asm volatile("vfmadd.vv  v5, v30, v8" ::: "v5");
      asm volatile("vfmadd.vv  v5, v30, v9" ::: "v5");
      asm volatile("vfmadd.vv  v5, v30, v10" ::: "v5");
      asm volatile("vfmadd.vv  v5, v17, v30" ::: "v5");

      asm volatile("vfadd.vv v5, v5, v11" ::: "v5");

      asm volatile("vfcvt.x.f.v  v18, v13" ::: "v18");
      asm volatile("vmv.v.x v19, %0" ::"r"(1023) : "v19");
      asm volatile("vadd.vv v18, v18, v19" ::: "v8");
      asm volatile("vmv.v.x v19, %0" ::"r"(52) : "v19");
      asm volatile("vsll.vv v18, v18, v19" ::: "v18");
      asm volatile("vfmul.vv v5, v5, v18" ::: "v5");

      ///// End of exp() compute
      asm volatile("vmv.s.x v4, zero");
      asm volatile("vfredusum.vs v4, v5, v4");

      if ((c+2) < channels) {
        asm volatile("vle64.v v21, (%0)"  ::"r"(i_));
      }
    } 
    asm volatile("vfmv.f.s %0, v4" : "=f"(exp_sum));
    asm volatile("vfdiv.vf v6, v5, %0" :: "f"(exp_sum));
    asm volatile("vse64.v v6, (%0)" :: "r"(o_));
    o_ += vl;
  }

}*/


void softmax_vec_reduction(const double *i, const double *o, uint64_t channels,
                 uint64_t innerSize) {

  size_t avl = innerSize;
  size_t vl;

  double *i_ = (double *) i;
  double *o_ = (double *) o;

  vl = vsetvl_e64m1(avl); // For now assuming avl fits VRF, so vl = avl

  vfloat64m1_t vec_zero = vfmv_v_f_f64m1(0, vl);
  vfloat64m1_t vec_a = vle64_v_f64m1(i_, vl);

  for (uint64_t c=0; c<channels; c+=1) {
    // Find max
    vfloat64m1_t vec_red_max;
    vec_red_max = vfredmax_vs_f64m1_f64m1(vec_red_max, vec_a, vec_zero, vl);
    double max = vfmv_f_s_f64m1_f64(vec_red_max);
    vfloat64m1_t vec_max = vfmv_v_f_f64m1(max, vl);
    vfloat64m1_t vec_b = vfsub_vv_f64m1(vec_a, vec_max, vl);
    
    // Find exp
    vfloat64m1_t vec_c = __exp_1xf64(vec_b, vl);
    
    // Sum and divide
    vfloat64m1_t vec_red_sum;
    vec_red_sum = vfredusum_vs_f64m1_f64m1(vec_red_sum, vec_c, vec_zero, vl);
    double sum = vfmv_f_s_f64m1_f64(vec_red_sum);
    vfloat64m1_t vec_sum = vfmv_v_f_f64m1(sum, vl);
    vfloat64m1_t vec_res = vfdiv_vv_f64m1(vec_c, vec_sum, vl);
    
    // Load next row
    i_ += vl;
    if (c+1 < channels) {
      vec_a = vle64_v_f64m1(i_, vl);
    }

    vse64_v_f64m1(o_, vec_res, vl);
    o_ += vl;

  }

}