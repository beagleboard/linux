// vrgather.vx *vd, vs2, rs1, vm # *vd[i] = (rs1 >= VLMAX) ? 0 : vs2[rs1];
require(VECTOR->vsew >= e8 && VECTOR->vsew <= e64);
require(!VECTOR->vill);
{
reg_t vl = VECTOR->vl;
reg_t sew = VECTOR->vsew;
reg_t rd_num = rd(insn);
reg_t rs2_num = rs2(insn);
reg_t rs1 = RS1;
reg_t i;
for (i = VECTOR->vstart; i < vl; ++i) {
  VI_LOOP_ELEMENT_SKIP();

  switch (sew) {
  case e8:
    *vector_elt_uint8_t(VECTOR, rd_num, i) = rs1 >= VECTOR->vlmax ? 0 : *vector_elt_uint8_t(VECTOR, rs2_num, rs1);
    break;
  case e16:
    *vector_elt_uint16_t(VECTOR, rd_num, i) = rs1 >= VECTOR->vlmax ? 0 : *vector_elt_uint16_t(VECTOR, rs2_num, rs1);
    break;
  case e32:
    *vector_elt_uint32_t(VECTOR, rd_num, i) = rs1 >= VECTOR->vlmax ? 0 : *vector_elt_uint32_t(VECTOR, rs2_num, rs1);
    break;
  default:
    *vector_elt_uint64_t(VECTOR, rd_num, i) = rs1 >= VECTOR->vlmax ? 0 : *vector_elt_uint64_t(VECTOR, rs2_num, rs1);
    break;
  }
}

VI_TAIL_ZERO(1);
}
VECTOR->vstart = 0;
