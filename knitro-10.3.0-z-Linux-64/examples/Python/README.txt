Using Artelys Knitro with Python
--------------------------------

This directory contains example code illustrating different ways to
call Artelys Knitro from Python.

README.txt:           This file.

exampleHS15.py        An example that solves a simple nonlinear problem 
                      coded in Python using the callback API. To run:
                         python exampleHS15.py

exampleQCQP.py        An example that solves a simple nonlinear problem 
                      coded in Python using the reverse communication
                      API. To run:
                         python exampleQCQP.py

exampleMINLP.py       An example that solves a simple, convex,
                      mixed-integer nonlinear problem (MINLP) coded 
                      in Python using the callback API. To run:
                         python exampleMINLP.py

exampleHS15NumPy.py   An example that is similar to exampleHS15.py 
                      but makes use of NumPy arrays instead of Python
                      lists. To run:
                         python exampleHS15NumPy.py

exampleHS15Tuner.py   An example that shows how to use the Knitro-Tuner
                      on the HS15 nonlinear problem example.
                         python exampleHS15Tuner.py

tuner-fixed.opt       A sample Knitro input file of user options to fix for
                      the Knitro-Tuner in exampleHS15Tuner.py.

tuner-explore.opt     A sample Tuner options file that specifies which options
                      to explore for the Knitro-Tuner in exampleHS15Tuner.py.

knitro.py             Knitro-Python interface

knitroNumPy.py        NumPy support for the Knitro-Python interface
                      (see exampleHS15NumPy.py)


To run Knitro for Python:
------------------------

1) Make sure the Knitro-Python interface can be loaded from Python.

2) Make sure the Knitro dll/library can be dynamically loaded.  That 
   is, make sure the Knitro "lib" directory is specified in the %PATH%
   environment variable on Windows ($LD_LIBRARY_PATH on Unix and
   $DYLD_LIBRARY_PATH on Mac OS X).

3) Open a Terminal and type:
      python example.py
   where "example.py" is the name of the Python model you want to run.

For more information on using Knitro through Python, please see the
Section "Knitro in a Python application" in the Knitro User's Manual.
