// vadc.vxm *vd, vs2, rs1
require(!(rd(insn) == 0 && VECTOR->vlmul > 1));
VI_VX_LOOP
({
  uint64_t v0 = *vector_elt_uint64_t(VECTOR, 0, midx);
  const uint128_t op_mask = (UINT64_MAX >> (64 - sew));
  uint64_t carry = (v0 >> mpos) & 0x1;

  uint128_t res = (op_mask & rs1) + (op_mask & vs2) + carry;
  *vd =res;
})
