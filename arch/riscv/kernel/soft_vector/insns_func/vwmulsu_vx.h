// vwmulsu.vx *vd, vs2, rs1
VI_CHECK_DSS(false);
VI_VX_LOOP_WIDEN
({
  switch(VECTOR->vsew) {
  case e8:
    *vector_elt_uint16_t(VECTOR, rd_num, i) = (int16_t)(int8_t)vs2 * (int16_t)(uint8_t)rs1;
    break;
  case e16:
    *vector_elt_uint32_t(VECTOR, rd_num, i) = (int32_t)(int16_t)vs2 * (int32_t)(uint16_t)rs1;
    break;
  default:
    *vector_elt_uint64_t(VECTOR, rd_num, i) = (int64_t)(int32_t)vs2 * (int64_t)(uint32_t)rs1;
    break;
  }
})
