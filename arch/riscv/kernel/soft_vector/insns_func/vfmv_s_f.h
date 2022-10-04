// vfmv_s_f: *vd[0] = rs1 (vs2=0)
reg_t vl = VECTOR->vl;
require(v_vm(insn) == 1);
require_fp;

if (vl > 0) {
  reg_t i;
  reg_t rd_num = rd(insn);
  reg_t sew = VECTOR->vsew;

  if (FLEN == 64)
    *vector_elt_uint64_t(VECTOR, rd_num, 0) = f64_f(FRS1).v;
  else
    *vector_elt_uint64_t(VECTOR, rd_num, 0) = f32_f(FRS1).v;
  {
  const reg_t max_len = VECTOR->VLEN / sew;
  for (i = 1; i < max_len; ++i) {
    switch(sew) {
    case e64:
      *vector_elt_uint64_t(VECTOR, rd_num, i) = 0;
      break;
    default:
      require(false);
      break;
    }
  }
  }
  vl = 0;
}
