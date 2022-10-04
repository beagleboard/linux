// vsxb.v and vsxseg[2-8]b.v
require(VECTOR->vsew >= e8);
{
VI_DUPLICATE_VREG(rs2(insn), VECTOR->vlmax);
{
VI_ST_WITH_I(index[i], fn, uint8, 1);
}
}
