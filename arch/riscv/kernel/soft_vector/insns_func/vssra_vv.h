// vssra.vv *vd, vs2, vs1
VRM xrm = VECTOR->vxrm;
VI_VV_LOOP
({
  int sh = vs1 & (sew - 1);

  INT_ROUNDING(vs2, xrm, sh);
  *vd =vs2 >> sh;
})
