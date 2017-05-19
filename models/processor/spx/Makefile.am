CXX = mpic++ -g -Wall -O3 --std=c++11

MAINTAINERCLEANFILES=  aclocal.m4 config.log config.status configure COPYING depcomp \
		      INSTALL install-sh Makefile.in missing

spxlib_LIBRARIES = libspx.a
spxlibdir = $(libdir)/manifold

libspx_a_SOURCES = \
	instruction.cc \
	component.cc \
	pipeline.cc \
	cache/cache.cc \
	pipeline/outorder.cc \
	pipeline/inorder.cc \
	core.cc \
    qsim-proxy.cc \
	cs_disas.cpp

pkginclude_spxdir = $(includedir)/manifold/spx

pkginclude_spx_HEADERS = \
	instruction.h \
	component.h \
	pipeline.h \
	cache/cache.h \
	pipeline/outorder.h \
	pipeline/inorder.h \
	core.h \
    qsim-proxy.h \
	cs_disas.h

CAPSTONE_DIR="/home/Documents/TrialInstall/qsim_testing/capstone/"
libspx_a_CPPFLAGS = -I$(KERNEL_INC) -I$(QSIM_INC) -I$(KITFOX_INC) -I$(CAPSTONE_DIR) -Ipipeline -lqsim -lqsim-client -lcapstone
