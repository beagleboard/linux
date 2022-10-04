// vext_x_v: rd = vs2[rs1]
require(v_vm(insn) == 1);
{
uint64_t xmask = UINT64_MAX >> (64 - get_max_xlen());
reg_t rs1 = RS1;
VI_LOOP_BASE
VI_LOOP_END_NO_TAIL_ZERO
if (!(rs1 >= 0 && rs1 < (VECTOR->VLEN/sew))) {
  WRITE_RD(0);
} else {
  switch(sew) {
  case e8:
    WRITE_RD(*vector_elt_uint8_t(VECTOR, rs2_num, rs1));
    break;
  case e16:
    WRITE_RD(*vector_elt_uint16_t(VECTOR, rs2_num, rs1));
    break;
  case e32:
    if (get_max_xlen() == 32)
      WRITE_RD(*vector_elt_int32_t(VECTOR, rs2_num, rs1));
    else
      WRITE_RD(*vector_elt_uint32_t(VECTOR, rs2_num, rs1));
    break;
  case e64:
    if (get_max_xlen() <= sew)
      WRITE_RD(*vector_elt_uint64_t(VECTOR, rs2_num, rs1) & xmask);
    else
      WRITE_RD(*vector_elt_uint64_t(VECTOR, rs2_num, rs1));
    break;
  }
}
}
