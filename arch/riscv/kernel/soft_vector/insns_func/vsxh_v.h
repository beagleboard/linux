// vsxh.v and vsxseg[2-8]h.v
require(VECTOR->vsew >= e16);
{
VI_DUPLICATE_VREG(rs2(insn), VECTOR->vlmax);
{
VI_ST_WITH_I(index[i], fn, uint16, 2);
}
}
