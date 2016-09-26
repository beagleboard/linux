#
# Calc mult/shift coefficients for cycles2ns conversation
#
# Copyright (C) 2016 Texas Instruments Incorporated - http://www.ti.com
# Author: Grygorii Strashko <grygorii.strashko@ti.com>
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License as
# published by the Free Software Foundation version 2.
#
# This program is distributed "as is" WITHOUT ANY WARRANTY of any
# kind, whether express or implied; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.


CC        = $(CROSS_COMPILE)gcc
INC       = -I$(KBUILD_OUTPUT)/usr/include
CFLAGS    = -Wall $(INC)
LDLIBS    = -lrt
PROGS     = calc_cc_mult_shift

all: $(PROGS)

testptp: calc_cc_mult_shift.o

clean:
	rm -f calc_cc_mult_shift.o

distclean: clean
	rm -f $(PROGS)
