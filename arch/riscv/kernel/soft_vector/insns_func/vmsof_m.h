// vmsof.m rd, vs2, vm
require(VECTOR->vsew >= e8 && VECTOR->vsew <= e64);
require(!VECTOR->vill);
{
reg_t vl = VECTOR->vl;
reg_t rd_num = rd(insn);
reg_t rs2_num = rs2(insn);

bool has_one = false;
reg_t i;
for (i = VECTOR->vstart ; i < vl; ++i) {
  const int mlen = VECTOR->vmlen;
  const int midx = (mlen * i) / 64;
  const int mpos = (mlen * i) % 64;
  const uint64_t mmask = (UINT64_MAX << (64 - mlen)) >> (64 - mlen - mpos);

  bool vs2_lsb = ((*vector_elt_uint64_t(VECTOR,rs2_num, midx) >> mpos) & 0x1) == 1;
  bool do_mask = (*vector_elt_uint64_t(VECTOR, 0, midx) >> mpos) & 0x1;
  uint64_t *vd = vector_elt_uint64_t(VECTOR, rd_num, midx);

  if (v_vm(insn) == 1 || (v_vm(insn) == 0 && do_mask)) {
    uint64_t res = 0;
    if(!has_one && vs2_lsb) {
      has_one = true;
      res = 1;
    }
    *vd =(*vd & ~mmask) | ((res << mpos) & mmask);
  }
}

VI_TAIL_ZERO_MASK(rd_num);
}
VECTOR->vstart = 0;
