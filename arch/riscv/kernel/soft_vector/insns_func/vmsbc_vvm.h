// vmsbc.vvm *vd, vs2, rs1
require(!(rd(insn) == 0 && VECTOR->vlmul > 1));
{
VI_VV_LOOP_CARRY
({
  uint64_t v0 = *vector_elt_uint64_t(VECTOR, 0, midx);
  const uint64_t mmask = (UINT64_MAX << (64 - mlen)) >> (64 - mlen - mpos);
  const uint128_t op_mask = (UINT64_MAX >> (64 - sew));
  uint64_t carry = (v0 >> mpos) & 0x1;

  uint128_t res = (op_mask & vs2) - (op_mask & vs1) - carry;

  carry = (res >> sew) & 0x1u;
  *vd =(*vd & ~mmask) | ((carry << mpos) & mmask);
})
}
