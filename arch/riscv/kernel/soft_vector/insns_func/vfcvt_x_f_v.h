// vfcvt.x.f.v *vd, *vd2, vm
VI_VFP_VV_LOOP
({
  *vector_elt_int32_t(VECTOR, rd_num, i) = f32_to_i32(vs2, STATE.frm, true);
},{
  *vector_elt_int64_t(VECTOR, rd_num, i) = f64_to_i64(vs2, STATE.frm, true);
})
