// vmfirst rd, vs2
require(VECTOR->vsew >= e8 && VECTOR->vsew <= e64);
require(!VECTOR->vill);
require(VECTOR->vstart == 0);
{
reg_t vl = VECTOR->vl;
reg_t rs2_num = rs2(insn);
reg_t pos = -1;
reg_t i;
for (i=VECTOR->vstart; i < vl; ++i) {
  VI_LOOP_ELEMENT_SKIP()
  {
  bool vs2_lsb = ((*vector_elt_uint64_t(VECTOR,rs2_num, midx) >> mpos) & 0x1) == 1;
  if (vs2_lsb) {
    pos = i;
    break;
  }
  }
}
VECTOR->vstart = 0;
WRITE_RD(pos);
}
