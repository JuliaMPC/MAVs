#
# Copyright (c) 2015 by Artelys
# All Rights Reserved.                                 
#
# Example problem formulated as an AMPL model used
# to demonstate using Knitro with AMPL.
# The problem has two local solutions:
#   the point (0,0,8) with objective 936.0, and
#   the point (7,0,0) with objective 951.0
# The problem also has a locally infeasible point at:
#   (0,4,0) with objective 968.0
#

# Define variables and enforce that they be non-negative.

var x{j in 1..3} >= 0;

# Objective function to be minimized.

minimize obj:

         1000 - x[1]^2 - 2*x[2]^2 - x[3]^2 - x[1]*x[2] - x[1]*x[3];

# Equality constraint.

s.t. c1: 8*x[1] + 14*x[2] + 7*x[3] - 56 = 0;

# Inequality constraint.

s.t. c2: x[1]^2 + x[2]^2 + x[3]^2 -25 >= 0;

data;

# Define initial point.

let x[1] := 2;
let x[2] := 2;
let x[3] := 2;
