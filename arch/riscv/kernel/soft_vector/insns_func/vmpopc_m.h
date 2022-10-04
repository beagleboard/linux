// vmpopc rd, vs2, vm
require(VECTOR->vsew >= e8 && VECTOR->vsew <= e64);
require(!VECTOR->vill);
require(VECTOR->vstart == 0);
{
reg_t vl = VECTOR->vl;
reg_t rs2_num = rs2(insn);
reg_t popcount = 0;
reg_t i;
for (i=VECTOR->vstart; i<vl; ++i) {
  const int mlen = VECTOR->vmlen;
  const int midx = (mlen * i) / 64;
  const int mpos = (mlen * i) % 64;

  bool vs2_lsb = ((*vector_elt_uint64_t(VECTOR,rs2_num, midx) >> mpos) & 0x1) == 1;
  if (v_vm(insn) == 1) {
    popcount += vs2_lsb;
  } else {
    bool do_mask = (*vector_elt_uint64_t(VECTOR, 0, midx) >> mpos) & 0x1;
    popcount += (vs2_lsb && do_mask);
  }
}
VECTOR->vstart = 0;
WRITE_RD(popcount);
}
