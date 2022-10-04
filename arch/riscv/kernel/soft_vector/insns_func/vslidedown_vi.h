// vslidedown.vi *vd, vs2, rs1
VI_LOOP_BASE
{
const reg_t sh = v_zimm5(insn);
bool is_valid = (i + sh) < VECTOR->vlmax;
reg_t offset = 0;

if (is_valid) {
  offset = sh;
}

switch (sew) {
case e8: {
  VI_XI_SLIDEDOWN_PARAMS(8, offset);
  *vd =is_valid ? vs2 : 0;
}
break;
case e16: {
  VI_XI_SLIDEDOWN_PARAMS(16, offset);
  *vd =is_valid ? vs2 : 0;
}
break;
case e32: {
  VI_XI_SLIDEDOWN_PARAMS(32, offset);
  *vd =is_valid ? vs2 : 0;
}
break;
default: {
  VI_XI_SLIDEDOWN_PARAMS(64, offset);
  *vd =is_valid ? vs2 : 0;
}
break;
}
}
VI_LOOP_END
