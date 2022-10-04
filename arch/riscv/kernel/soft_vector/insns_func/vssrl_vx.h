// vssrl.vx *vd, vs2, rs1
VRM xrm = VECTOR->vxrm;
VI_VX_ULOOP
({
  int sh = rs1 & (sew - 1);
  uint128_t val = vs2;

  INT_ROUNDING(val, xrm, sh);
  *vd = val >> sh;
})
