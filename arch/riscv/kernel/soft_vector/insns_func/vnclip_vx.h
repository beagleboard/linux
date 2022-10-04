// vnclip: *vd[i] = clip(round(vs2[i] + rnd) >> rs1[i])
VRM xrm = VECTOR->vxrm;
int64_t int_max = (1 << (VECTOR->vsew - 1)) - 1;
int64_t int_min = -(1 << (VECTOR->vsew - 1));
VI_VVXI_LOOP_NARROW
({

  int64_t result = vs2;
  uint64_t unsigned_shift_amount;
// rounding
  INT_ROUNDING(result, xrm, sew);

// unsigned shifting to rs1
  unsigned_shift_amount = (uint64_t)(rs1 & ((sew * 2) - 1));
  if (unsigned_shift_amount >= (2 * sew)) {
    unsigned_shift_amount = 2 * sew - 1;
  }
  result = vsext(result, sew * 2) >> unsigned_shift_amount;

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
