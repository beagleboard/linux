// vmadd: *vd[i] = (*vd[i] * x[rs1]) + vs2[i]
VI_VX_LOOP
({
  *vd =*vd * rs1 + vs2;
})
