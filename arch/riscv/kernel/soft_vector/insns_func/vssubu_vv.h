// vssubu.vv *vd, vs2, vs1
VI_LOOP_BASE
{
bool sat = false;

switch (sew) {
case e8: {
  VV_U_PARAMS(8);
  *vd =sat_subu_uint8_t(vs2, vs1, &sat);
  break;
}
case e16: {
  VV_U_PARAMS(16);
  *vd =sat_subu_uint16_t(vs2, vs1, &sat);
  break;
}
case e32: {
  VV_U_PARAMS(32);
  *vd =sat_subu_uint32_t(vs2, vs1, &sat);
  break;
}
default: {
  VV_U_PARAMS(64);
  *vd =sat_subu_uint64_t(vs2, vs1, &sat);
  break;
}
}
VECTOR->vxsat |= sat;
}
VI_LOOP_END
