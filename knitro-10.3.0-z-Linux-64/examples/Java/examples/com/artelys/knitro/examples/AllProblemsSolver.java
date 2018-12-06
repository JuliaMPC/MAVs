/*******************************************************/
/* Copyright (c) 2015 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/

package com.artelys.knitro.examples;


import com.artelys.knitro.api.*;
import com.artelys.knitro.examples.Problems.*;

import java.util.Arrays;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;

/** Generates and run all examples with default options.
 * Examples are run using reflection but nothing that fancy is needed (see ExampleSolver). */
public class AllProblemsSolver
{
    /** A list of all example problems provided */
    @SuppressWarnings("unchecked")
	private static final Class<? extends KTRProblem> KTRProblems[] = new Class[]{
            ProblemHS15.class, ProblemHS15UserTermination.class, ProblemMINLP.class, ProblemMPEC.class,
            ProblemQCQP.class, ProblemRosenbrock.class, ProblemRosenbrockExtended.class,
            ProblemEnergy.class, ProblemLSQ.class
    };

    /** List of problems providing no exact gradient */
    @SuppressWarnings("unchecked")
	private static final List<Class<? extends KTRProblem>> NoExactGrad = Arrays.asList(
            (Class<? extends KTRProblem>[]) new Class[]{
                    ProblemMPEC.class, ProblemRosenbrockExtended.class, ProblemEnergy.class
            }
    );

    /** List of problems for LSQ */
    @SuppressWarnings("unchecked")
    private static final List<Class<? extends KTRProblem>> LSQProblems = Arrays.asList(
            (Class<? extends KTRProblem>[]) new Class[]{
                    ProblemLSQ.class
            }
    );

    public static void main(String[] args) throws Exception {
        // Solve each problem
        for (Class<? extends KTRIProblem> problem: KTRProblems) {
            System.out.println(new String(new char[50]).replace("\0", "="));
            System.out.println("Solving "+problem.getSimpleName());

            KTRIProblem instance = generateInstance(problem);

            // Create a new solver
            KTRISolver solver;
            if (NoExactGrad.contains(problem)) {
                solver = new KTRSolver(instance, KTRConstants.KTR_GRADOPT_CENTRAL, KTRConstants.KTR_HESSOPT_BFGS);
            } else if (LSQProblems.contains(problem)) {
                solver = new KTRSolverLSQ(instance);
            } else {
                solver = new KTRSolver(instance);
            }

            // Remove all outputs
            solver.setParam(KTRConstants.KTR_PARAM_OUTLEV, KTRConstants.KTR_OUTLEV_NONE);

            // Solve problem
            int result = solver.solve();

            // Display results
            printSolutionResults(solver, result);
        }
    }

    /**
     * Build a list of problems with all examples
     * @return a list of initialized problems with all examples
     */
    public static List<KTRIProblem> makeExampleProblemList() {
        List<KTRIProblem> problems = new LinkedList<KTRIProblem>();

        for (Class<? extends KTRIProblem> cpb: KTRProblems) {
            problems.add(generateInstance(cpb));
        }

        return problems;
    }

    /**
     * Generate a problem using reflection on its class.
     * @param problemClass
     * @return
     */
    private static KTRIProblem generateInstance(final Class<? extends KTRIProblem> problemClass)
    {
        try
        {
            return problemClass.newInstance();
        }
        catch (Exception e)
        {
            System.err.println("Error: failed to generate instance of problem " + problemClass.getName());
        }
        return null;
    }

    public static void printSolutionResults(KTRISolver solver, int solveStatus) throws KTRException {
        if (solveStatus != 0) {
            System.out.println("Failed to solve the problem, final status = " + solveStatus);
            return;
        }

        System.out.println("Solution found");
        System.out.format("%25s = %.2e%n", "Objective value", solver.getObjValue());
        System.out.format("%25s = (", "Solution point");
        Iterator<Double> it = solver.getXValues().iterator();
        while (it.hasNext()) {
            System.out.format("%.2f", it.next());
            if (it.hasNext())
                System.out.print(", ");
        }
        System.out.println(") ");

        if (!solver.getProblem().isMipProblem())
        {
            if (!solver.getProblem().isLSQProblem()) {
                System.out.format("%25s = %.2e%n", "Feasibility violation", ((KTRSolver) solver).getAbsFeasError());
            }
            System.out.format("%25s = %.2e%n%n", "KKT optimality violation", solver.getAbsOptError());
        } else {
            System.out.format("%25s = %.2e%n%n", "Absolute integrality gap", ((KTRSolver)solver).getMipAbsGap());
        }
    }
}
