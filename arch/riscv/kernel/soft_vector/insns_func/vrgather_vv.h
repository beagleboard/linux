// vrgather.vv *vd, vs2, vs1, vm # *vd[i] = (vs1[i] >= VLMAX) ? 0 : vs2[vs1[i]];
require(VECTOR->vsew >= e8 && VECTOR->vsew <= e64);
require(!VECTOR->vill);
{
reg_t vl = VECTOR->vl;
reg_t sew = VECTOR->vsew;
reg_t rd_num = rd(insn);
reg_t rs1_num = insn_rs1(insn);
reg_t rs2_num = rs2(insn);
reg_t i;
for (i = VECTOR->vstart; i < vl; ++i) {
  VI_LOOP_ELEMENT_SKIP();
  VI_CHECK_VREG_OVERLAP(rd_num, rs1_num);
  VI_CHECK_VREG_OVERLAP(rd_num, rs2_num);
  switch (sew) {
  case e8: {
    uint8_t vs1 = *vector_elt_uint8_t(VECTOR, rs1_num, i);
    //if (i > 255) continue;
    *vector_elt_uint8_t(VECTOR, rd_num, i) = vs1 >= VECTOR->vlmax ? 0 : *vector_elt_uint8_t(VECTOR, rs2_num, vs1);
    break;
  }
  case e16: {
    uint16_t vs1 = *vector_elt_uint16_t(VECTOR, rs1_num, i);
    *vector_elt_uint16_t(VECTOR, rd_num, i) = vs1 >= VECTOR->vlmax ? 0 : *vector_elt_uint16_t(VECTOR, rs2_num, vs1);
    break;
  }
  case e32: {
    uint32_t vs1 = *vector_elt_uint32_t(VECTOR, rs1_num, i);
    *vector_elt_uint32_t(VECTOR, rd_num, i) = vs1 >= VECTOR->vlmax ? 0 : *vector_elt_uint32_t(VECTOR, rs2_num, vs1);
    break;
  }
  default: {
    uint64_t vs1 = *vector_elt_uint64_t(VECTOR, rs1_num, i);
    *vector_elt_uint64_t(VECTOR, rd_num, i) = vs1 >= VECTOR->vlmax ? 0 : *vector_elt_uint64_t(VECTOR, rs2_num, vs1);
    break;
  }
  }
}

VI_TAIL_ZERO(1);
}
VECTOR->vstart = 0;
