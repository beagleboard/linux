// vsmul
VRM xrm = VECTOR->vxrm;
int64_t int_max = (1ul << (VECTOR->vsew - 1)) - 1;
int64_t int_min = - (1 << (VECTOR->vsew - 1));
int64_t sign_mask = ((1ul << (VECTOR->vsew - 1)));

VI_VX_ULOOP
({
  int64_t rs1_sign;
  int64_t vs2_sign;
  int64_t result_sign;

  bool overflow = rs1 == vs2 && rs1 == int_min;

  uint128_t result = (int128_t)(int64_t)rs1 * (int128_t)(int64_t)vs2;
  //result &= ((uint128_t)1llu << ((sew * 2) - 2)) - 1;
  rs1_sign = rs1 & sign_mask;
  vs2_sign = vs2 & sign_mask;
  result_sign = (rs1_sign ^ vs2_sign) & sign_mask;
  // rounding
  INT_ROUNDING(result, xrm, sew - 1);
  // unsigned shifting
  result = result >> (sew - 1);

  // saturation
  if (overflow) {
    result = int_max;
    VECTOR->vxsat = 1;
  } else {
    //result |= result_sign;
  }
  *vd =result;
})
