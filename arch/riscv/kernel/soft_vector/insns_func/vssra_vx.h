// vssra.vx *vd, vs2, rs1
VRM xrm = VECTOR->vxrm;
VI_VX_LOOP
({
  int sh = rs1 & (sew - 1);

  INT_ROUNDING(vs2, xrm, sh);
  *vd =vs2 >> sh;
})
