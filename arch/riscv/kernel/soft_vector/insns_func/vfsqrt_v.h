// vsqrt.v *vd, *vd2, vm
VI_VFP_VV_LOOP
({
  *vd =f32_sqrt(vs2);
},
{
  *vd =f64_sqrt(vs2);
})
