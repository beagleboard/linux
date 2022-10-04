// vsxe.v and vsxseg[2-8]e.v
reg_t sew = VECTOR->vsew;
VI_DUPLICATE_VREG(rs2(insn), VECTOR->vlmax);
require(sew >= e8 && sew <= e64);
if (sew == e8) {
  VI_ST_WITH_I(index[i], fn, uint8, 1);
} else if (sew == e16) {
  VI_ST_WITH_I(index[i], fn, uint16, 2);
} else if (sew == e32) {
  VI_ST_WITH_I(index[i], fn, uint32, 4);
} else if (sew == e64) {
  VI_ST_WITH_I(index[i], fn, uint64, 8);
}

