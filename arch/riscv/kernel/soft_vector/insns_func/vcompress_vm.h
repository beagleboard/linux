// vcompress *vd, vs2, vs1
require(VECTOR->vsew >= e8 && VECTOR->vsew <= e64);
require(!VECTOR->vill);
require(VECTOR->vstart == 0);
{
reg_t sew = VECTOR->vsew;
reg_t vl = VECTOR->vl;
reg_t rd_num = rd(insn);
reg_t rs1_num = insn_rs1(insn);
reg_t rs2_num = rs2(insn);
reg_t pos = 0;
reg_t i;
for (i = VECTOR->vstart ; i < vl; ++i) {
  const int mlen = VECTOR->vmlen;
  const int midx = (mlen * i) / 64;
  const int mpos = (mlen * i) % 64;

  bool do_mask = (*vector_elt_uint64_t(VECTOR, rs1_num, midx) >> mpos) & 0x1;
  if (do_mask) {
    switch (sew) {
    case e8:
      *vector_elt_uint8_t(VECTOR, rd_num, pos) = *vector_elt_uint8_t(VECTOR, rs2_num, i);
      break;
    case e16:
      *vector_elt_uint16_t(VECTOR, rd_num, pos) = *vector_elt_uint16_t(VECTOR, rs2_num, i);
      break;
    case e32:
      *vector_elt_uint32_t(VECTOR, rd_num, pos) = *vector_elt_uint32_t(VECTOR, rs2_num, i);
      break;
    default:
      *vector_elt_uint64_t(VECTOR, rd_num, pos) = *vector_elt_uint64_t(VECTOR, rs2_num, i);
      break;
    }

    ++pos;
  }
}

if (vl > 0 && TAIL_ZEROING) {
  uint8_t *tail = vector_elt_uint8_t(VECTOR, rd_num, pos * ((sew >> 3) * 1));
  memset(tail, 0, (VECTOR->vlmax - pos) * ((sew >> 3) * 1));
}
}

