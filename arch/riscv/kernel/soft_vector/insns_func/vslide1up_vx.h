//vslide1up.vx *vd, vs2, rs1
if (v_vm(insn) == 0)
  require(rd(insn) != 0);

VI_CHECK_SS
{
VI_LOOP_BASE
if (i != 0) {
  if (sew == e8) {
    VI_XI_SLIDEUP_PARAMS(8, 1);
    *vd =vs2;
  } else if(sew == e16) {
    VI_XI_SLIDEUP_PARAMS(16, 1);
    *vd =vs2;
  } else if(sew == e32) {
    VI_XI_SLIDEUP_PARAMS(32, 1);
    *vd =vs2;
  } else if(sew == e64) {
    VI_XI_SLIDEUP_PARAMS(64, 1);
    *vd =vs2;
  }
} else {
  if (sew == e8) {
    *vector_elt_uint8_t(VECTOR, rd_num, 0) = RS1;
  } else if(sew == e16) {
    *vector_elt_uint16_t(VECTOR, rd_num, 0) = RS1;
  } else if(sew == e32) {
    *vector_elt_uint32_t(VECTOR, rd_num, 0) = RS1;
  } else if(sew == e64) {
    *vector_elt_uint64_t(VECTOR, rd_num, 0) = RS1;
  }
}
VI_LOOP_END
}
