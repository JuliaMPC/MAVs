====================================================
=== Important note on object-oriented interfaces ===
====================================================

Release: v1.2 for Knitro 10.3

----------------
IMPORTANT NOTICE
----------------
This is the Java object-oriented interface for Artelys Knitro.

Please note that significant changes may happen in future versions of this
interface. These changes may not be backwards compatible.

However, since the interfaces is relying on Knitro C interface,
this version of the interfaces will still work with future releases of Knitro.

Comments and feedbacks on object-oriented interfaces are welcome and should
be addressed to support-knitro@artelys.com.


Knitro Java interface
=====================

Knitro Java interface is delivered as a Jar file containing built classes.
Java interfaces are using Java Native Access (JNA) to connect with Knitro
library.

JNA is distributed under GNU Lesser General Public License 2.1 (LGPL,
version 2.1) available online at http://www.gnu.org/licenses/licenses.html
and Apache Software License 2.0 (ASL, version 2.1) available online at
http://www.apache.org/licenses/.

Any use of Knitro Java interfaces shall comply with either of these two licenses.
Knitro and Knitro interfaces are properties of Artelys. Artelys Knitro customers
are granted the rights to use and modify these interfaces. They can redistribute
software using compiled versions of the interfaces to the same extent as their
general Knitro redistribution permissions.

Packaging
---------
Interfaces are redistributed within 3 Jar files, contained in the lib/ subdirectory.
In order to use interfaces within a Java project, one need to include both :

  - Knitro-Interfaces-1.2-KTR_10.3.jar containing Knitro Java interface
      classes (without examples).
  - Jna jar files (jna-4.2.0.jar and jna-platform-4.2.0.jar). One should
      also be able to use Knitro Java interface with other versions of JNA
      though it has not been tested.

A fourth Jar file: Knitro-Interfaces-1.2-KTR_10.3-sources.jar contains Java
sources of Knitro Java interface.

Building examples
-----------------

First, make sure that the KNITRODIR environment variable is correctly set and
provides an absolute path to the Knitro installation directory.

Java interfaces contain 2 main examples, 7 example problems and 5 example callbacks.
  - ExampleSolver, solving one HS15 problem and one example QCQP problem, presenting
      callbacks and bound reinitialization.
  - AllProblemsSolver, running all example problems
 
Examples can be run using make (Linux/Mac OS) or gmake (Windows) from the "Java"
folder and then run using the following command lines:
  - Windows:
      java -cp "out;lib/*" com.artelys.knitro.examples.ExampleSolver
      java -cp "out;lib/*" com.artelys.knitro.examples.AllProblemsSolver
  - Linux / Mac OS:
      java -cp "out:lib/*" com.artelys.knitro.examples.ExampleSolver
      java -cp "out:lib/*" com.artelys.knitro.examples.AllProblemsSolver

Or directly from you prefered IDE by creating a project and importing jars from
the lib subdirectory.
