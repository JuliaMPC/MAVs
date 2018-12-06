C *******************************************************
C * Copyright (c) 2015 by Artelys                       *
C * All Rights Reserved                                 *
C *******************************************************

C+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
C  This file contains routines to define a small QCQP (quadratically
C  constrained quadratic programming) test problem.
C
C  min   1000 - x0^2 - 2 x1^2 - x2^2 - x0 x1 - x0 x2
C  s.t.  8 x0 + 14 x1 + 7 x2 - 56 = 0
C        x0^2 + x1^2 + x2^2 - 25 >= 0
C        x0 >= 0, x1 >= 0, x2 >= 0
C
C  The start point (2, 2, 2) converges to the minimum at (0, 0, 8),
C  with final objective = 936.0.  From a different start point,
C  Knitro may converge to an alternate local solution at (7, 0, 0),
C  with objective = 951.0.
C+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


C---------------------------------------------------------------------
C     FUNCTION get_problem_sizes
C---------------------------------------------------------------------

      subroutine get_problem_sizes (n, m, nnzj, nnzh)

      implicit none

      integer  n, m, nnzj, nnzh


      n = 3
      m = 2
      nnzj = 6
      nnzh = 5

      return
      end


C---------------------------------------------------------------------
C     FUNCTION load_problem_def
C---------------------------------------------------------------------

      subroutine load_problem_def (n, m, nnzj, nnzh,
     $                             objtype, objgoal,
     $                             xlobnds, xupbnds, xinitial,
     $                             ctype, clobnds, cupbnds,
     $                             jacindvar, jacindcon,
     $                             hrow, hcol)

      implicit none

      integer           n, m, nnzj, nnzh
      integer           objtype, objgoal
      double precision  xlobnds(n), xupbnds(n), xinitial(n)
      integer           ctype(m)
      double precision  clobnds(m), cupbnds(m)
      integer           jacindvar(nnzj), jacindcon(nnzj)
      integer           hrow(nnzh), hcol(nnzh)


C---- LOCAL VARIABLES.

      integer           i

C---- THE PARAMETER infbound MUST BE GREATER THAN OR EQUAL TO THE KNITRO
C---- PARAMETER KTR_INFBOUND DEFINED IN THE HEADER FILE "knitro.h".
C---- KNITRO TREATS ALL VALUES GREATER THAN OR EQUAL IN MAGNITUDE TO
C---- KTR_INFBOUND AS INFINITE.
      double precision  infbound
      parameter        (infbound = 1.d+20)


C---- THESE CONSTANTS ARE DEFINED IN "knitro.h":
C----   KTR_OBJGOAL_MINIMIZE = 0
C----   KTR_OBJGOAL_MAXIMIZE = 1
C----   KTR_OBJTYPE_GENERAL   = 0
C----   KTR_OBJTYPE_LINEAR    = 1
C----   KTR_OBJTYPE_QUADRATIC = 2
C----   KTR_CONTYPE_GENERAL   = 0
C----   KTR_CONTYPE_LINEAR    = 1
C----   KTR_CONTYPE_QUADRATIC = 2
      objgoal = 0
      objtype = 2
      ctype(1) = 1
      ctype(2) = 2

C---- SET VARIABLE BOUNDS AND INITIAL VALUES.
      do i=1,n
          xinitial(i) = 2.0
          xlobnds(i) = 0.0
          xupbnds(i) = infbound
      end do

C---- SET CONSTRAINT BOUNDS ( clobnd <= c(x) <= cupbnd ).
      clobnds(1) = 0.0
      cupbnds(1) = 0.0
      clobnds(2) = 0.0
      cupbnds(2) = infbound

C---- PROVIDE FIRST DERIVATIVE STRUCTURAL INFORMATION.
C---- NUMBER INDICES FROM ZERO ACCORDING TO THE C LANGUAGE CONVENTION,
C---- BECAUSE THE KNITRO SOLVER IS WRITTEN IN C.
      jacindcon(1) = 0
      jacindvar(1) = 0
      jacindcon(2) = 0
      jacindvar(2) = 1
      jacindcon(3) = 0
      jacindvar(3) = 2
      jacindcon(4) = 1
      jacindvar(4) = 0
      jacindcon(5) = 1
      jacindvar(5) = 1
      jacindcon(6) = 1
      jacindvar(6) = 2

C---- PROVIDE SECOND DERIVATIVE STRUCTURAL INFORMATION;
C---- ONLY THE NONZEROES OF THE UPPER TRIANGLE OF THE HESSIAN.
C---- NUMBER INDICES FROM ZERO ACCORDING TO THE C LANGUAGE CONVENTION,
C---- BECAUSE THE KNITRO SOLVER IS WRITTEN IN C.
      hrow(1) = 0
      hcol(1) = 0
      hrow(2) = 0
      hcol(2) = 1
      hrow(3) = 0
      hcol(3) = 2
      hrow(4) = 1
      hcol(4) = 1
      hrow(5) = 2
      hcol(5) = 2

      return
      end


C---------------------------------------------------------------------
C     FUNCTION compute_fc
C---------------------------------------------------------------------
C  COMPUTE THE FUNCTION AND CONSTRAINT VALUES AT x.

      subroutine compute_fc (x, obj, c)

      implicit none

      double precision  x(*), obj, c(*)


      obj = 1.0d+03 - x(1)**2 - 2.0d+00*x(2)**2
     $      - x(3)**2 - x(1)*x(2) - x(1)*x(3)


C---- LINEAR EQUALITY CONSTRAINT.
      c(1) = 8.0d+00*x(1) + 14.0d+00*x(2) + 7.0d+00*x(3) - 56.0d+00

C---- QUADRATIC INEQUALITY CONSTRAINT.
      c(2) = x(1)**2 + x(2)**2 + x(3)**2 - 25.0d+00

      return 
      end


C---------------------------------------------------------------------
C     FUNCTION compute_ga
C---------------------------------------------------------------------
C  COMPUTE THE FUNCTION AND CONSTRAINT FIRST DERIVATIVES AT x.

      subroutine compute_ga (x, objgrad, jac)

      implicit none

      double precision  x(*), objgrad(*), jac(*)


C---- GRADIENT OF THE OBJECTIVE FUNCTION.
      objgrad(1) = -2.0d0*x(1) - x(2) - x(3)
      objgrad(2) = -4.0d0*x(2) - x(1)
      objgrad(3) = -2.0d0*x(3) - x(1)

C---- GRADIENT OF THE FIRST CONSTRAINT, c(1).
      jac(1) =  8.0d0
      jac(2) = 14.0d0
      jac(3) =  7.0d0

C---- GRADIENT OF THE SECOND CONSTRAINT, c(2).
      jac(4) = 2.0d0*x(1)
      jac(5) = 2.0d0*x(2)
      jac(6) = 2.0d0*x(3)

      return
      end


C---------------------------------------------------------------------
C     FUNCTION compute_h
C---------------------------------------------------------------------
C  COMPUTE THE HESSIAN OF THE LAGRANGIAN AT x AND lambda.

      subroutine compute_h (x, objscaler, lambda, hess)

      implicit none

      double precision  objscaler
      double precision x(*), lambda(*), hess(*)

      hess(1) = -2.0d0*objscaler + 2.0d0*lambda(2)
      hess(2) = -1.0d0*objscaler
      hess(3) = -1.0d0*objscaler
      hess(4) = -4.0d0*objscaler + 2.0d0*lambda(2)
      hess(5) = -2.0d0*objscaler + 2.0d0*lambda(2) 

      return
      end


C---------------------------------------------------------------------
C     FUNCTION compute_hv
C---------------------------------------------------------------------
C  COMPUTE THE HESSIAN OF THE LAGRANGIAN TIMES vector AT x AND lambda.
C  RETURN THE RESULT IN vector.

      subroutine compute_hv (x, objscaler, lambda, vector)

      implicit none

      double precision  objscaler
      double precision  x(*), lambda(*), vector(*)

      integer           i
      double precision  tmp(3)


      tmp(1) = (-2.0d0*objscaler + 2.0d0*lambda(2))*vector(1)
     $         - objscaler*vector(2) - objscaler*vector(3)
      tmp(2) = -objscaler*vector(1) + 
     $         (-4.0d0*objscaler + 2.0d0*lambda(2))*vector(2)
      tmp(3) = -objscaler*vector(1) + 
     $         (-2.0d0*objscaler + 2.0d0*lambda(2))*vector(3)

      do i=1,3
          vector(i) = tmp(i)
      end do      

      return
      end


C----- End of source code -------------------------------------------
