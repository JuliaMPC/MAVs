Using Artelys Knitro with Fortran
---------------------------------

This directory contains example code illustrating one way to
call Artelys Knitro from a Fortran language application.

README.txt:           This file.

makefile              Makefile that builds all examples on Linux or Max OS X
                      machines.  To execute, type "make" or "gmake".

makefile.win          Makefile that builds all examples on Windows machines
                      using the Intel Visual Fortran 9.0 and Microsoft Visual
                      C++ compilers.
                      To execute, type "nmake -f makefile.win".

exampleProgram.f      An example driver that solves any problem implemented
                      with routines like those of "problemQCQP.f".  

problemQCQP.f         Fortran routines that define a simple quadratically
                      constrained quadratic programming problem.  The same
                      problem is implemented for the C language API in
                      "examples/C/problemQCQP.c".

knitro_fortran.c      A set of C wrappers that export Knitro function calls
                      to Fortran.  The most essential Knitro calls are
                      provided; others may be added in a similar manner.

knitro.opt            A sample Knitro input file of user options.  Contents
                      can be modified using any text-based editor.  The file
                      is read by "exampleProgram.f".
