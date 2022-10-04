// vfcvt.f.x.v *vd, *vd2, vm
VI_VFP_VV_LOOP
({
  int32_t vs2_i = *vector_elt_int32_t(VECTOR, rs2_num, i);
  *vd =i32_to_f32(vs2_i);
},
{
  int64_t vs2_i = *vector_elt_int64_t(VECTOR, rs2_num, i);
  *vd =i64_to_f64(vs2_i);
})
