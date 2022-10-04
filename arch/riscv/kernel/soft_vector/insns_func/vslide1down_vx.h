//vslide1down.vx *vd, vs2, rs1
VI_LOOP_BASE
if (i != vl - 1) {
  switch (sew) {
  case e8: {
    VI_XI_SLIDEDOWN_PARAMS(8, 1);
    *vd =vs2;
  }
  break;
  case e16: {
    VI_XI_SLIDEDOWN_PARAMS(16, 1);
    *vd =vs2;
  }
  break;
  case e32: {
    VI_XI_SLIDEDOWN_PARAMS(32, 1);
    *vd =vs2;
  }
  break;
  default: {
    VI_XI_SLIDEDOWN_PARAMS(64, 1);
    *vd =vs2;
  }
  break;
  }
} else {
  switch (sew) {
  case e8:
    *vector_elt_uint8_t(VECTOR, rd_num, vl - 1) = RS1;
    break;
  case e16:
    *vector_elt_uint16_t(VECTOR, rd_num, vl - 1) = RS1;
    break;
  case e32:
    *vector_elt_uint32_t(VECTOR, rd_num, vl - 1) = RS1;
    break;
  default:
    *vector_elt_uint64_t(VECTOR, rd_num, vl - 1) = RS1;
    break;
  }
}
VI_LOOP_END
