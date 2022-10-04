// vsxw.v and vsxseg[2-8]w.v
reg_t vl = VECTOR->vl;
reg_t baseAddr = RS1;
reg_t stride = rs2(insn);
reg_t vs3 = rd(insn);
reg_t vlmax = VECTOR->vlmax;
VI_DUPLICATE_VREG(stride, vlmax);
require(VECTOR->vsew >= e32);
for (i = 0; i < vlmax && vl != 0; ++i) {
  bool is_valid = true;
  VI_STRIP(i)
  VI_ELEMENT_SKIP(i);
  switch (VECTOR->vsew) {
  case e32:
    if (is_valid)
      store_uint32(baseAddr + index[i],
                       *vector_elt_uint32_t(VECTOR, vs3, vreg_inx));
    break;
  case e64:
    if (is_valid)
      store_uint32(baseAddr + index[i],
                       *vector_elt_uint64_t(VECTOR, vs3, vreg_inx));
    break;
  }
}
VECTOR->vstart = 0;
