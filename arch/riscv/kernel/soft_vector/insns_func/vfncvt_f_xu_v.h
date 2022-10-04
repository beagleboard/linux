// vfncvt.f.xu.v *vd, vs2, vm
VI_VFP_LOOP_BASE
  VI_CHECK_SD;
{
  uint64_t vs2 = *vector_elt_uint64_t(VECTOR, rs2_num, i);
  *vector_elt_float32_t(VECTOR, rd_num, i) = ui64_to_f32(vs2);
}
VI_VFP_LOOP_END
