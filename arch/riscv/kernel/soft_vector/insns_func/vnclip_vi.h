// vnclip: *vd[i] = clip(round(vs2[i] + rnd) >> simm)
VRM xrm = VECTOR->vxrm;
int64_t int_max = (1 << (VECTOR->vsew - 1)) - 1;
int64_t int_min = -(1 << (VECTOR->vsew - 1));
VI_VVXI_LOOP_NARROW
({

  int64_t result = vs2;
// rounding
  INT_ROUNDING(result, xrm, sew);

  result = vsext(result, sew * 2) >> (zimm5 & ((sew * 2) < 32? (sew * 2) - 1: 31));

// saturation
  if (result < int_min) {
    result = int_min;
    VECTOR->vxsat = 1;
  } else if (result > int_max) {
    result = int_max;
    VECTOR->vxsat = 1;
  }

  *vd =result;
})
