// vford.vv *vd, vs2, vs1, vm
VI_VFP_LOOP_CMP
({
  res = !(f64_isSignalingNaN(vs2) || f64_isSignalingNaN(vs1));
})
