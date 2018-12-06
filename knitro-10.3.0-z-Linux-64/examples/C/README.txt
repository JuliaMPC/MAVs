Using Artelys Knitro with C
---------------------------

This directory contains example code illustrating different ways to
call Artelys Knitro from C language applications.

README.txt:           This file.

makefile              Makefile that builds all examples on Unix machines.
                      To execute on Linux type "gmake", on Mac OS X type
                      "gnumake".

makefile.win          Makefile that builds all examples on Windows machines
                      using the Microsoft Visual C++ compiler.
                      To execute, type "nmake -f makefile.win".

callbackExample1.c    An example driver that solves any problem implemented
                      with the "problemDef.h" interface using the callback API.
                      Callback means Knitro is given function pointers that
                      it can invoke whenever it needs problem information.
                      The example defines 3 separate callback functions:
                      one for evaluating functions, one for first derivatives,
                      and one for second derivatives.  The example also shows
                      how to set user options by reading from the "knitro.opt" 
                      file and how to perform a derivative check.

callbackExample2.c    An example driver very similar to callbackExample1.
                      The difference is that only 2 callback functions are
                      defined.  The evaluation of functions and their
                      first derivatives is combined into a single callback.
                      The example also shows how to employ the "userParams"
                      argument to store ancillary data between callbacks.

callbackExampleMINLP.c  An example driver that solves a mixed integer
                      nonlinear programming (MINLP) model using the
                      callback API.

callbackExampleLSQ.c  An example driver that solves a nonlinear least-
                      squares (LSQ) model using the callback API.

multiStartExample.c   An example driver that shows how to use the Knitro
		      parallel, multi-start feature to search for multiple
		      local solutions.

restartExample.c      An example driver similar to callbackExample1, but
                      using KTR_restart to solve the problem repeatedly,
                      varying a user option each time.  The example shows
                      how to restart without reloading the problem definition.

blasAcmlExample.c     An example wrapper that makes the AMD Core Math Library
                      (ACML) available for use by Knitro.  Comments in the
                      file describe the steps necessary to compile and link
                      the dynamic library ("makefile" does not create it).

tunerExample.c        An example driver that shows how to use the Knitro-Tuner.

tuner-fixed.opt       A sample Knitro input file of user options to fix for
                      tunerExample.c.

tuner-explore.opt     A sample Tuner options file that specifies which options
                      to explore in tunerExample.c

knitro.opt            A sample Knitro input file of user options.  Contents
                      can be modified using any text-based editor.  The file
                      is read by callbackExample1 and callbackExample2.
                      A similar file can be generated from any application
                      by calling KTR_save_param_file.


problemDef.h          An example interface for defining nonlinear optimization
                      problems in C.  The interface is designed to work
                      with the example drivers in this directory.

problemHS15.c         An implementation of "problemDef.h" for standard
                      Hock and Schittkowski problem number 15.

problemQCQP.c         An implementation of "problemDef.h" for a simple
                      quadratically constrained quadratic programming problem.

problemMINLP.c        An implementation of "problemDef.h" for a simple
                      mixed integer nonlinear programming example.

problemLSQ.c          An implementation of "problemDef.h" for a simple
                      nonlinear least-squares example.
