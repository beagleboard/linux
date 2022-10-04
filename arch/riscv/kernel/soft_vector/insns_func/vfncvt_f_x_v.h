// vfncvt.f.x.v *vd, vs2, vm
VI_VFP_LOOP_BASE
  VI_CHECK_SD;
{
  int64_t vs2 = *vector_elt_int64_t(VECTOR, rs2_num, i);
  *vector_elt_float32_t(VECTOR, rd_num, i) = i64_to_f32(vs2);
}
VI_VFP_LOOP_END
