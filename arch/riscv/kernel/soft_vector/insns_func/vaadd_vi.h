// vaadd: Averaging adds of integers
VRM xrm = VECTOR->vxrm;
VI_VI_LOOP
({
  int64_t result = simm5 + vs2;
  INT_ROUNDING(result, xrm, 1);
  result = vzext(result >> 1, sew);
  *vd =result;
})
