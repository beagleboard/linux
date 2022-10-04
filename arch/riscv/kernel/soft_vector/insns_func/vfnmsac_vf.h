// vfnmsac: *vd[i] = -(f[rs1] * vs2[i]) + *vd[i]
VI_VFP_VF_LOOP
({
  *vd =f32_mulAdd(rs1, f32(vs2.v ^ F32_SIGN), *vd);
},
{
  *vd =f64_mulAdd(rs1, f64(vs2.v ^ F64_SIGN), *vd);
})
