// vlxe.v and vlxseg[2-8]e.v
reg_t sew = VECTOR->vsew;
VI_DUPLICATE_VREG(rs2(insn), VECTOR->vlmax);
if (sew == e8) {
  VI_LD_WITH_I(index[i], fn, int8, 1);
} else if (sew == e16) {
  VI_LD_WITH_I(index[i], fn, int16, 2);
} else if (sew == e32) {
  VI_LD_WITH_I(index[i], fn, int32, 4);
} else if (sew == e64) {
  VI_LD_WITH_I(index[i], fn, int64, 8);
}

