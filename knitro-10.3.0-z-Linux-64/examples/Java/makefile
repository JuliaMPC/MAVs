######################################################################
#
#  Copyright (c) 2015 by Artelys
#
#  Makefile for Unix platforms.
#    To build on Linux:    gmake
#    To build on MacOSX:   gnumake
#    To build on Solaris:  gmake
#    To build on windows:  use gmake for windows or make from cygwin
#  Assume the javac compiler is present in $PATH.
#
#  The examples built in this makefile cannot execute unless KNITRODIR
#  environment variable is defined and set to a valid Knitro directory.
#
#  Then to run the examples:
#
#  Under Unix systems, use:
#    java -cp "out:lib/*" com.artelys.knitro.examples.ExampleSolver
#    java -cp "out:lib/*" com.artelys.knitro.examples.AllProblemsSolver
#
#  Under Windows, use:
#    java -cp "out;lib/*" com.artelys.knitro.examples.ExampleSolver
#    java -cp "out;lib/*" com.artelys.knitro.examples.AllProblemsSolver
#
#  For 64-bit systems, add -d64 option
######################################################################

# Set the location of the Knitro jar file.
KNJAR=lib/Knitro-Interfaces-1.2-KTR_10.3.0.jar

SRCDIR=examples
OBJDIR=out
TARGET_PACKAGE=com/artelys/knitro/examples

TGT_CLASSES=\
	ExampleSolver.class \
	AllProblemsSolver.class

SEP=:
ifeq ($(OS),Windows_NT)
	SEP=;
endif

all: outdir $(TGT_CLASSES)

outdir: | $(OBJDIR)

$(OBJDIR):
	mkdir -p $(OBJDIR)

ExampleSolver.class : $(SRCDIR)/$(TARGET_PACKAGE)/ExampleSolver.java $(KNJAR)
	javac -d $(OBJDIR) -classpath "$(SRCDIR)$(SEP)$(KNJAR)" $<

AllProblemsSolver.class : $(SRCDIR)/$(TARGET_PACKAGE)/AllProblemsSolver.java $(KNJAR)
	javac -d $(OBJDIR) -classpath "$(SRCDIR)$(SEP)$(KNJAR)" $<

clean:
	rm -rf $(OBJDIR)
