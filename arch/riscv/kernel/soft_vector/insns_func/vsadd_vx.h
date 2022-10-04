// vsadd.vx *vd, vs2, rs1
VI_LOOP_BASE
{
bool sat = false;
switch(sew) {
case e8: {
  VX_PARAMS(8);
  *vd =sat_add_int8_t_uint8_t(vs2, rs1, &sat);
  break;
}
case e16: {
  VX_PARAMS(16);
  *vd =sat_add_int16_t_uint16_t(vs2, rs1, &sat);
  break;
}
case e32: {
  VX_PARAMS(32);
  *vd =sat_add_int32_t_uint32_t(vs2, rs1, &sat);
  break;
}
default: {
  VX_PARAMS(64);
  *vd =sat_add_int64_t_uint64_t(vs2, rs1, &sat);
  break;
}
}
VECTOR->vxsat |= sat;
}
VI_LOOP_END
