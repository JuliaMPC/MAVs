######################################################################
#
#  Copyright (c) 2015 by Artelys
#
#  Makefile for Unix platforms.
#
#    To build on Linux:    gmake
#    To build on MacOSX:   gnumake
#
#  This makefile builds both statically linked and dynamically linked
#  versions of each example.  The dynamic versions cannot execute unless
#  ../../lib is added to $LD_LIBRARY_PATH.
#    bash shells:  export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:../../lib
#    tcsh shells:  setenv LD_LIBRARY_PATH :../../lib
#
######################################################################

# Set the location of Knitro.
KNDIR = ../..
KNRELEASE = 1030

# Set up platform-specific make parameters.
UNAME = $(shell uname -s)

# These are parameters for Linux platforms.
ifeq ($(findstring Linux,$(UNAME)), Linux)
  CC = gcc -fopenmp
  FC = gfortran -fopenmp
  LD = gfortran -fopenmp
  LIBS = -ldl -lstdc++
  KNLIB_STATIC  = $(KNDIR)/lib/libknitro$(KNRELEASE).a
  KNLIB_DYNAMIC = $(KNDIR)/lib/libknitro.so

  CFLAGS = -c -O
  FFLAGS = -c -O

  # Try to detect if this is a 64-bit platform.
  MNAME = $(shell uname -m)
  ifeq ($(findstring x86_64,$(MNAME)), x86_64)
    CFLAGS = -c -O -m64
    FFLAGS = -c -O -m64
  endif
endif

# These are parameters for MacOSX platforms.
ifeq ($(findstring Darwin,$(UNAME)), Darwin)
  CC = gcc -arch x86_64
  FC = gfortran -arch x86_64
  LD = gfortran -arch x86_64
  LIBS = -ldl -lstdc++ -lgfortran
  KNLIB_STATIC  = $(KNDIR)/lib/libknitro$(KNRELEASE).a
  KNLIB_DYNAMIC = $(KNDIR)/lib/libknitro.dylib
  CFLAGS = -c -O
  FFLAGS = -c -O
endif

######################################################################

TGTS = \
example_static \
example_dynamic

default: $(TGTS)

knitro_fortran.o : knitro_fortran.c $(KNDIR)/include/knitro.h
	$(CC) $(CFLAGS) -I$(KNDIR)/include $<

problemQCQP.o : problemQCQP.f
	$(FC) $(FFLAGS) $<

exampleProgram.o : exampleProgram.f
	$(FC) $(FFLAGS) $<

OBJS = problemQCQP.o exampleProgram.o knitro_fortran.o

example_static: $(OBJS) $(KNLIB_STATIC)
	$(LD) -o $@ $^ $(LIBS)
example_dynamic: $(OBJS) $(KNLIB_DYNAMIC)
	$(LD) -o $@ $^ $(LIBS)

clean:
	rm -f *.o
	rm -f $(TGTS)
