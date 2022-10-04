// vfcvt.xu.f.v *vd, *vd2, vm
VI_VFP_VV_LOOP
({
  *vector_elt_uint32_t(VECTOR, rd_num, i) = f32_to_ui32(vs2, STATE.frm, true);
},
{
  *vector_elt_uint64_t(VECTOR, rd_num, i) = f64_to_ui64(vs2, STATE.frm, true);
})
