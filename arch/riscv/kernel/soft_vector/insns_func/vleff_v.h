
const reg_t nf = v_nf(insn) + 1;
require(VECTOR->vsew >= e8 && VECTOR->vsew <= e64);
require((nf * VECTOR->vlmul) <= (NVPR / 4));
{
const reg_t sew = VECTOR->vsew;
const reg_t vl = VECTOR->vl;
const reg_t baseAddr = RS1;
const reg_t rd_num = rd(insn);
bool early_stop = false;
const reg_t vlmul = VECTOR->vlmul;
reg_t i;
for (i = 0; i < VECTOR->vlmax && vl != 0; ++i) {
  bool is_valid = true;
  bool is_zero = false;
  reg_t fn;
  VI_STRIP(i);
  VI_ELEMENT_SKIP(i);

for (fn = 0; fn < nf; ++fn) {
    load_uint8(baseAddr + (i * nf + fn) * 1);

    switch (sew) {
    case e8:
      *vector_elt_uint8_t(VECTOR, rd_num + fn * vlmul, vreg_inx) =
        is_valid ? load_uint8(baseAddr + (i * nf + fn) * 1) : 0;
      is_zero = is_valid && *vector_elt_uint8_t(VECTOR, rd_num + fn * vlmul, vreg_inx) == 0;
      break;
    case e16:
      *vector_elt_uint16_t(VECTOR, rd_num + fn * vlmul, vreg_inx) =
        is_valid ? load_uint16(baseAddr + (i * nf + fn) * 2) : 0;
      is_zero = is_valid && *vector_elt_uint16_t(VECTOR, rd_num + fn * vlmul, vreg_inx) == 0;
      break;
    case e32:
      *vector_elt_uint32_t(VECTOR, rd_num + fn * vlmul, vreg_inx) =
        is_valid ? load_uint32(baseAddr + (i * nf + fn) * 4) : 0;
      is_zero = is_valid && *vector_elt_uint32_t(VECTOR, rd_num + fn * vlmul, vreg_inx) == 0;
      break;
    case e64:
      *vector_elt_uint64_t(VECTOR, rd_num + fn * vlmul, vreg_inx) =
        is_valid ? load_uint64(baseAddr + (i * nf + fn) * 8) : 0;
      is_zero = is_valid && *vector_elt_uint64_t(VECTOR, rd_num + fn * vlmul, vreg_inx) == 0;
      break;
    }

    if (is_zero) {
      VECTOR->vl = i;
      early_stop = true;
      break;
    }
  }

  if (early_stop) {
    break;
  }
}
}
VECTOR->vstart = 0;
