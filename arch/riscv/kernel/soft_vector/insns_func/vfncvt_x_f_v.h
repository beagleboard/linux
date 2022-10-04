// vfncvt.x.f.v *vd, vs2, vm
VI_VFP_LOOP_BASE
  VI_CHECK_SD;
{
  float64_t vs2 = *vector_elt_float64_t(VECTOR, rs2_num, i);
  *vector_elt_int32_t(VECTOR, rd_num, i) = f64_to_i32(vs2, STATE.frm, true);
}
VI_VFP_LOOP_END
