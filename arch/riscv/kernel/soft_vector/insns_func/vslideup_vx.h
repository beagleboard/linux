//vslideup.vx *vd, vs2, rs1

VI_LOOP_BASE
{
const reg_t offset = RS1;
if (VECTOR->vstart < offset && i < offset)
  continue;

switch (sew) {
case e8: {
  VI_XI_SLIDEUP_PARAMS(8, offset);
  *vd =vs2;
}
break;
case e16: {
  VI_XI_SLIDEUP_PARAMS(16, offset);
  *vd =vs2;
}
break;
case e32: {
  VI_XI_SLIDEUP_PARAMS(32, offset);
  *vd =vs2;
}
break;
default: {
  VI_XI_SLIDEUP_PARAMS(64, offset);
  *vd =vs2;
}
break;
}
}
VI_LOOP_END
