// vfmv_f_s: rd = vs2[0] (rs1=0)
require(v_vm(insn) == 1);
require_fp;
require(VECTOR->vsew == e8 || VECTOR->vsew == e16 || VECTOR->vsew == e32 || VECTOR->vsew == e64);

{
reg_t rs2_num = rs2(insn);
uint64_t vs2_0 = 0;
const reg_t sew = VECTOR->vsew;
switch(sew) {
case e8:
  vs2_0 = *vector_elt_uint8_t(VECTOR, rs2_num, 0);
  break;
case e16:
  vs2_0 = *vector_elt_uint16_t(VECTOR, rs2_num, 0);
  break;
case e32:
  vs2_0 = *vector_elt_uint32_t(VECTOR, rs2_num, 0);
  break;
default:
  vs2_0 = *vector_elt_uint64_t(VECTOR, rs2_num, 0);
  break;
}

// nan_extened
if (FLEN > sew) {
  vs2_0 = vs2_0 | ~((1ul << sew) - 1);
}

if (FLEN == 64) {
  WRITE_FRD(f64(vs2_0), 64);
} else {
  WRITE_FRD(f32(vs2_0), 32);
}
}
