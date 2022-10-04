// vfmerge_vf *vd, vs2, vs1, vm
VI_VFP_COMMON;
{
reg_t sew = VECTOR->vsew;
reg_t i;
for (i=VECTOR->vstart; i<vl; ++i) {
  float64_t *vd =vector_elt_float64_t(VECTOR, rd_num, i);
  float64_t rs1 = f64_f(READ_FREG(rs1_num));
  float64_t vs2 = *vector_elt_float64_t(VECTOR, rs2_num, i);

  int midx = (VECTOR->vmlen * i) / 64;
  int mpos = (VECTOR->vmlen * i) % 64;
  bool use_first = (*vector_elt_uint64_t(VECTOR, 0, midx) >> mpos) & 0x1;

  *vd =use_first ? rs1 : vs2;
}

VI_TAIL_ZERO(1);
}
VECTOR->vstart = 0;
set_fp_exceptions;
