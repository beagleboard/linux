// vfwadd.vf *vd, vs2, rs1
VI_VFP_VF_LOOP_WIDE
({
  *vd =f64_add(vs2, rs1);
})
