// vmpopc rd, vs2, vm
require(VECTOR->vsew >= e8 && VECTOR->vsew <= e64);
require(!VECTOR->vill);
require(VECTOR->vstart == 0);
{
reg_t vl = VECTOR->vl;
reg_t sew = VECTOR->vsew;
reg_t rd_num = rd(insn);
reg_t rs2_num = rs2(insn);

int cnt = 0;
reg_t i;
for (i = 0; i < vl; ++i) {
  const int mlen = VECTOR->vmlen;
  const int midx = (mlen * i) / 64;
  const int mpos = (mlen * i) % 64;

  bool vs2_lsb = ((*vector_elt_uint64_t(VECTOR, rs2_num, midx) >> mpos) & 0x1) == 1;
  bool do_mask = (*vector_elt_uint64_t(VECTOR, 0, midx) >> mpos) & 0x1;

  bool has_one = false;
  bool use_ori = (v_vm(insn) == 0) && !do_mask;
  if (v_vm(insn) == 1 || (v_vm(insn) == 0 && do_mask)) {
    if (vs2_lsb) {
      has_one = true;
    }
  }

  switch (sew) {
  case e8:
    *vector_elt_uint8_t(VECTOR, rd_num, i) = use_ori ?
                                   *vector_elt_uint8_t(VECTOR, rd_num, i) : cnt;
    break;
  case e16:
    *vector_elt_uint16_t(VECTOR, rd_num, i) = use_ori ?
                                    *vector_elt_uint16_t(VECTOR, rd_num, i) : cnt;
    break;
  case e32:
    *vector_elt_uint32_t(VECTOR, rd_num, i) = use_ori ?
                                    *vector_elt_uint32_t(VECTOR, rd_num, i) : cnt;
    break;
  default:
    *vector_elt_uint64_t(VECTOR, rd_num, i) = use_ori ?
                                    *vector_elt_uint64_t(VECTOR, rd_num, i) : cnt;
    break;
  }

  if (has_one) {
    cnt++;
  }
}

VI_TAIL_ZERO(1);
}
