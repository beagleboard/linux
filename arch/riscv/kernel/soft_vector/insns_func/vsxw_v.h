// vsxw.v and vsxseg[2-8]w.v
require(VECTOR->vsew >= e32);
{
VI_DUPLICATE_VREG(rs2(insn), VECTOR->vlmax);
{
VI_ST_WITH_I(index[i], fn, uint32, 4);
}
}
