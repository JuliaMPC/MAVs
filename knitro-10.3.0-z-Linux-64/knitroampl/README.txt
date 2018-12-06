
Using Artelys Knitro with AMPL
------------------------------

NOTE: This directory contains two versions of the Knitro-AMPL binary 
file for 64-bit Linux.  The default binary "knitroampl" implements 
parallel features using OpenMP.  This binary requires glibc 2.5
and OpenMP 1.0 (provided with gcc 4.4 or higher).
In order to use this binary you must have a newer installation of
Linux that provides the required OpenMP dependent libraries.
If you have an older installation of Linux that is not compatible
with the default "knitoampl" binary,  then you can use the older
binary "knitroampl_s" that is provided.  This binary DOES NOT use
OpenMP and thus do not implement any of the parallel features in
Knitro. The sequential binary "knitroampl_s" uses glibc 2.5
(provided with gcc 4.1 or higher).

In order to use Knitro with AMPL, you will need to purchase a version
of AMPL (available from Artelys). See
  http://www.artelys.com/optimization-tools/ampl

To choose the knitro solver, at the AMPL command prompt type

    option solver knitroampl;

or if the knitro executable was not in your path

    option solver "/path/to/your/knitroampl";
    
for example:
    option solver "../bin/knitroampl";                (Unix)
    option solver "c:\Program Files\...\knitroampl";  (Windows)

User definable Knitro parameters can be set as follows

    option knitro_options "maxit=100 alg=1";

which makes the maximum number of allowable iterations 100,
and chooses the interior-point direct algorithm.

To load and solve the test problem, type

    model "testproblem.mod";
    option solver knitroampl;
    solve;

For additional help please see the Knitro User's Manual and
AMPL book provided in the doc directory of this distribution.  
The Knitro User's Manual is also available at
  http://www.artelys.com/tools/knitro_doc/
