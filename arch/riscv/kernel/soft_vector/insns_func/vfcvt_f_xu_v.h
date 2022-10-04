// vfcvt.f.xu.v *vd, *vd2, vm
VI_VFP_VV_LOOP
({
  uint32_t vs2_u = *vector_elt_uint32_t(VECTOR, rs2_num, i);
  *vd =ui32_to_f32(vs2_u);
},
{
  uint64_t vs2_u = *vector_elt_uint64_t(VECTOR, rs2_num, i);
  *vd =ui64_to_f64(vs2_u);
})
