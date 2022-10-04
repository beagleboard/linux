// vmpopc rd, vs2, vm
require(VECTOR->vsew >= e8 && VECTOR->vsew <= e64);
require(!VECTOR->vill);
{
reg_t vl = VECTOR->vl;
reg_t sew = VECTOR->vsew;
reg_t rd_num = rd(insn);

reg_t i;
for (i = VECTOR->vstart ; i < VECTOR->vl; ++i) {
  VI_LOOP_ELEMENT_SKIP();

  switch (sew) {
  case e8:
    *vector_elt_uint8_t(VECTOR, rd_num, i) = i;
    break;
  case e16:
    *vector_elt_uint16_t(VECTOR, rd_num, i) = i;
    break;
  case e32:
    *vector_elt_uint32_t(VECTOR, rd_num, i) = i;
    break;
  default:
    *vector_elt_uint64_t(VECTOR, rd_num, i) = i;
    break;
  }
}

VI_TAIL_ZERO(1);
VECTOR->vstart = 0;
}
