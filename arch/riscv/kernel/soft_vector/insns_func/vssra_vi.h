// vssra.vi *vd, vs2, simm5
VRM xrm = VECTOR->vxrm;
VI_VI_LOOP
({
  int sh = simm5 & (sew - 1) & 0x1f;
  INT_ROUNDING(vs2, xrm, sh);
  *vd =vs2 >> sh;
})
