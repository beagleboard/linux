// vfwsub.vf *vd, vs2, rs1
VI_VFP_VF_LOOP_WIDE
({
  *vd =f64_sub(vs2, rs1);
})
