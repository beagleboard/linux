// vfwcvt.f.f.v *vd, vs2, vm
VI_VFP_LOOP_BASE
  VI_CHECK_DSS(false);
  {
  float32_t vs2 = *vector_elt_float32_t(VECTOR, rs2_num, i);
  *vector_elt_float64_t(VECTOR, rd_num, i) = f32_to_f64(vs2);
  }
  set_fp_exceptions;
VI_VFP_LOOP_WIDE_END
