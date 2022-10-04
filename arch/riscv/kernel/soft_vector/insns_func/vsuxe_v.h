// vsxe.v and vsxseg[2-8]e.v
const reg_t sew = VECTOR->vsew;
const reg_t vl = VECTOR->vl;
reg_t baseAddr = RS1;
reg_t stride = rs2(insn);
reg_t vs3 = rd(insn);
reg_t vlmax = VECTOR->vlmax;
VI_DUPLICATE_VREG(stride, vlmax);
require(sew >= e8 && sew <= e64);
for (i = 0; i < vlmax && vl != 0; ++i) {
  bool is_valid = true;
  VI_STRIP(i)
  VI_ELEMENT_SKIP(i);
  switch (sew) {
  case e8:
    if (is_valid)
      store_uint8(baseAddr + index[i],
                      *vector_elt_uint8_t(VECTOR, vs3, vreg_inx));
    break;
  case e16:
    if (is_valid)
      store_uint16(baseAddr + index[i],
                       *vector_elt_uint16_t(VECTOR, vs3, vreg_inx));
    break;
  case e32:
    if (is_valid)
      store_uint32(baseAddr + index[i],
                       *vector_elt_uint32_t(VECTOR, vs3, vreg_inx));
    break;
  case e64:
    if (is_valid)
      store_uint64(baseAddr + index[i],
                       *vector_elt_uint64_t(VECTOR, vs3, vreg_inx));
    break;
  }
}
VECTOR->vstart = 0;
