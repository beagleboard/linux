// vssub.vv *vd, vs2, vs1
VI_LOOP_BASE
{
bool sat = false;

switch (sew) {
case e8: {
  VV_PARAMS(8);
  *vd =sat_sub_int8_t_uint8_t(vs2, vs1, &sat);
  break;
}
case e16: {
  VV_PARAMS(16);
  *vd =sat_sub_int16_t_uint16_t(vs2, vs1, &sat);
  break;
}
case e32: {
  VV_PARAMS(32);
  *vd =sat_sub_int32_t_uint32_t(vs2, vs1, &sat);
  break;
}
default: {
  VV_PARAMS(64);
  *vd =sat_sub_int64_t_uint64_t(vs2, vs1, &sat);
  break;
}
}
VECTOR->vxsat |= sat;
}
VI_LOOP_END
