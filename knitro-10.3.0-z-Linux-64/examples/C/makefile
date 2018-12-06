######################################################################
#
#  Copyright (c) 2016 by Artelys
#
#  Makefile for Unix platforms.
#
#  Edit the macro for OPTPROB to solve one of the example problems.
#    To build on Linux:    gmake
#    To build on MacOSX:   gnumake
#
#  This makefile builds both statically linked and dynamically linked
#  versions of each example.  The dynamic versions (and also the static 
#  versions for MacOSX) cannot execute unless
#  ../../lib is added to $LD_LIBRARY_PATH (on MacOSX use the environment
#  variable $DYLD_LIBRARY_PATH).
#    bash shells:  export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:../../lib
#    tcsh shells:  setenv LD_LIBRARY_PATH :../../lib
#
######################################################################

# Choose a problem to solve by uncommenting one of the lines below.
OPTPROB = problemHS15
#OPTPROB = problemQCQP
OPTPROB_MIP = problemMINLP
OPTPROB_LSQ = problemLSQ

# Set the location of Knitro.
KNDIR = ../..
KNRELEASE = 1030

# Set up platform-specific make parameters.
UNAME = $(shell uname -s)

# These are parameters for Linux platforms.
ifeq ($(findstring Linux,$(UNAME)), Linux)
  CC = gcc
  LD = g++
  LIBS = -ldl 
  KNLIB_STATIC  = $(KNDIR)/lib/libknitro$(KNRELEASE).a
  KNLIB_DYNAMIC = $(KNDIR)/lib/libknitro.so

  CFLAGS = -c -O

  # Try to detect if this is a 64-bit platform.
  MNAME = $(shell uname -m)
  ifeq ($(findstring x86_64,$(MNAME)), x86_64)
    CC = gcc -fopenmp
    LD = g++ -fopenmp
    CFLAGS = -c -O -m64
  endif
endif

# These are parameters for MacOSX platforms.
ifeq ($(findstring Darwin,$(UNAME)), Darwin)
# gcc compiler options
#  CC = gcc -arch x86_64 -fopenmp 
#  LD = g++ -arch x86_64 
# clang compiler options
  CC = clang -arch x86_64 -stdlib=libstdc++
  LD = clang++ -arch x86_64 -stdlib=libstdc++
  LIBS = $(KNDIR)/lib/libiomp5.dylib -ldl 
  KNLIB_STATIC  = $(KNDIR)/lib/libknitro$(KNRELEASE).a
  KNLIB_DYNAMIC = $(KNDIR)/lib/libknitro.dylib
  CFLAGS = -c -O
endif

######################################################################

TGTS = \
callback1_static \
callback1_dynamic \
callback2_static \
callback2_dynamic \
callbackMINLP_static \
callbackMINLP_dynamic \
callbackLSQ_static \
callbackLSQ_dynamic \
multiStart_static \
multiStart_dynamic \
restart_static \
restart_dynamic \
tuner_static \
tuner_dynamic

default: $(TGTS)

problemHS15.o : problemHS15.c problemDef.h
	$(CC) $(CFLAGS) -I$(KNDIR)/include $<

problemQCQP.o : problemQCQP.c problemDef.h
	$(CC) $(CFLAGS) -I$(KNDIR)/include $<

problemMINLP.o : problemMINLP.c problemDef.h
	$(CC) $(CFLAGS) -I$(KNDIR)/include $<

problemLSQ.o : problemLSQ.c problemDef.h
	$(CC) $(CFLAGS) -I$(KNDIR)/include $<	

callbackExample1.o : callbackExample1.c problemDef.h
	$(CC) $(CFLAGS) -I$(KNDIR)/include $<

callbackExample2.o : callbackExample2.c problemDef.h
	$(CC) $(CFLAGS) -I$(KNDIR)/include $<

callbackExampleMINLP.o : callbackExampleMINLP.c problemDef.h
	$(CC) $(CFLAGS) -I$(KNDIR)/include $<

callbackExampleLSQ.o : callbackExampleLSQ.c problemDef.h
	$(CC) $(CFLAGS) -I$(KNDIR)/include $<

multiStartExample.o : multiStartExample.c problemDef.h
	$(CC) $(CFLAGS) -I$(KNDIR)/include $<

restartExample.o : restartExample.c problemDef.h
	$(CC) $(CFLAGS) -I$(KNDIR)/include $<

tunerExample.o : tunerExample.c problemDef.h
	$(CC) $(CFLAGS) -I$(KNDIR)/include $<

callback1_static: $(OPTPROB).o callbackExample1.o $(KNLIB_STATIC)
	$(LD) -o $@ $^ $(LIBS)
callback1_dynamic: $(OPTPROB).o callbackExample1.o $(KNLIB_DYNAMIC)
	$(LD) -o $@ $^ $(LIBS)

callback2_static: $(OPTPROB).o callbackExample2.o $(KNLIB_STATIC)
	$(LD) -o $@ $^ $(LIBS)
callback2_dynamic: $(OPTPROB).o callbackExample2.o $(KNLIB_DYNAMIC)
	$(LD) -o $@ $^ $(LIBS)

callbackMINLP_static: $(OPTPROB_MIP).o callbackExampleMINLP.o $(KNLIB_STATIC)
	$(LD) -o $@ $^ $(LIBS)
callbackMINLP_dynamic: $(OPTPROB_MIP).o callbackExampleMINLP.o $(KNLIB_DYNAMIC)
	$(LD) -o $@ $^ $(LIBS)

callbackLSQ_static: $(OPTPROB_LSQ).o callbackExampleLSQ.o $(KNLIB_STATIC)
	$(LD) -o $@ $^ $(LIBS)
callbackLSQ_dynamic: $(OPTPROB_LSQ).o callbackExampleLSQ.o $(KNLIB_DYNAMIC)
	$(LD) -o $@ $^ $(LIBS)

multiStart_static: $(OPTPROB).o multiStartExample.o $(KNLIB_STATIC)
	$(LD) -o $@ $^ $(LIBS)
multiStart_dynamic: $(OPTPROB).o multiStartExample.o $(KNLIB_DYNAMIC)
	$(LD) -o $@ $^ $(LIBS)

restart_static: $(OPTPROB).o restartExample.o $(KNLIB_STATIC)
	$(LD) -o $@ $^ $(LIBS)
restart_dynamic: $(OPTPROB).o restartExample.o $(KNLIB_DYNAMIC)
	$(LD) -o $@ $^ $(LIBS)

tuner_static: $(OPTPROB).o tunerExample.o $(KNLIB_STATIC)
	$(LD) -o $@ $^ $(LIBS)
tuner_dynamic: $(OPTPROB).o tunerExample.o $(KNLIB_DYNAMIC)
	$(LD) -o $@ $^ $(LIBS)

clean:
	rm -f *.o
	rm -f $(TGTS)
