// vmv_s_x: *vd[0] = rs1
require(v_vm(insn) == 1);
require(VECTOR->vsew == e8 || VECTOR->vsew == e16 ||
        VECTOR->vsew == e32 || VECTOR->vsew == e64);
{
reg_t vl = VECTOR->vl;

if (vl > 0) {
  reg_t rd_num = rd(insn);
  reg_t sew = VECTOR->vsew;
  const reg_t max_len = VECTOR->VLEN / sew;
  reg_t i;
  switch(sew) {
  case e8:
    *vector_elt_uint8_t(VECTOR, rd_num, 0) = RS1;
    break;
  case e16:
    *vector_elt_uint16_t(VECTOR, rd_num, 0) = RS1;
    break;
  case e32:
    *vector_elt_uint32_t(VECTOR, rd_num, 0) = RS1;
    break;
  default:
    *vector_elt_uint64_t(VECTOR, rd_num, 0) = RS1;
    break;
  }

  for (i = 1; i < max_len; ++i) {
    switch(sew) {
    case e8:
      *vector_elt_uint8_t(VECTOR, rd_num, i) = 0;
      break;
    case e16:
      *vector_elt_uint16_t(VECTOR, rd_num, i) = 0;
      break;
    case e32:
      *vector_elt_uint32_t(VECTOR, rd_num, i) = 0;
      break;
    default:
      *vector_elt_uint64_t(VECTOR, rd_num, i) = 0;
      break;
    }
  }

  vl = 0;
}

}
