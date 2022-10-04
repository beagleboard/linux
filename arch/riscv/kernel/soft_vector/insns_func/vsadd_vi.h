// vsadd.vi *vd, vs2 simm5
VI_LOOP_BASE
{
bool sat = false;
switch(sew) {
case e8: {
  VI_PARAMS(8);
  *vd =sat_add_int8_t_uint8_t(vs2, vsext(simm5, sew), &sat);
  break;
}
case e16: {
  VI_PARAMS(16);
  *vd =sat_add_int16_t_uint16_t(vs2, vsext(simm5, sew), &sat);
  break;
}
case e32: {
  VI_PARAMS(32);
  *vd =sat_add_int32_t_uint32_t(vs2, vsext(simm5, sew), &sat);
  break;
}
default: {
  VI_PARAMS(64);
  *vd =sat_add_int64_t_uint64_t(vs2, vsext(simm5, sew), &sat);
  break;
}
}
VECTOR->vxsat |= sat;
}
VI_LOOP_END
