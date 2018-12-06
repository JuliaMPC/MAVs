/*******************************************************/
/* Copyright (c) 2015 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/

package com.artelys.knitro.examples;

import com.artelys.knitro.api.KTRConstants;
import com.artelys.knitro.api.KTRException;
import com.artelys.knitro.api.KTRSolver;
import com.artelys.knitro.api.KTRSolverLSQ;
import com.artelys.knitro.examples.Problems.*;
import com.artelys.knitro.examples.callbacks.ExampleMSInitPtCallback;
import com.artelys.knitro.examples.callbacks.ExampleMSProcessCallback;
import com.artelys.knitro.examples.callbacks.ExampleOutputRedirection;

import java.util.ArrayList;
import java.util.List;


/**
 * A simple example of using this API to define and solve problems with or without user-defined parameters.<br/>
 * Solves HS15 and QCQP examples.
 */
public class ExampleSolver
{

    public static void main(String[] args) throws KTRException
    {
        runHS15();
        runHS15scale();
        runHS15userTermination();
        runRosenbrock();
        runLSQ();
        runQCQP();
        runMINLP();
        runMINLPmisqp();
        runMINLPreform();
    }

    /**
     * A very simple example running HS15 problem with no user option
     */
    private static void runHS15() throws KTRException
    {
        // Instantiate problem
        ProblemHS15 instance = new ProblemHS15();

        // Create new solver
        KTRSolver solver = new KTRSolver(instance);

        // Solve problem
        solver.solve();
    }

    /**
     * Running the HS15 problem with user scaling
     */
    private static void runHS15scale() throws KTRException
    {
        // Instantiate problem
        ProblemHS15 instance = new ProblemHS15();

        // Create new solver
        KTRSolver solver = new KTRSolver(instance);

        // Scale variables
        List<Double> xScaleFactors = new ArrayList<Double>();
        xScaleFactors.add(0.01);
        xScaleFactors.add(0.01);
        List<Double> xScaleCenters = new ArrayList<Double>();
        xScaleCenters.add(-0.3);
        xScaleCenters.add(-0.3);
        solver.setVarScaling(xScaleFactors, xScaleCenters);

        // Scale constraints
        List<Double> cScaleFactors = new ArrayList<Double>();
        cScaleFactors.add(2.0);
        cScaleFactors.add(2.0);
        List<Double> ccScaleFactors = new ArrayList<Double>();
        solver.setConScaling(cScaleFactors, ccScaleFactors);

        // Scale objective
        solver.setObjScaling(1.0);

        // Solve problem
        solver.solve();
    }


    private static void runHS15userTermination() throws KTRException
    {
        // Instantiate problem. At most 10 evaluations are authorized.
        ProblemHS15 instance = new ProblemHS15UserTermination(10);

        // Create new solver
        KTRSolver solver = new KTRSolver(instance);

        // Solve problem. Will return after 11 evaluations (11th evaluation is preempted and returns user termination)
        solver.solve();
    }

    private static void runRosenbrock() throws KTRException
    {
        // Instantiate problem. At most 10 evaluations are authorized.
        ProblemRosenbrock instance = new ProblemRosenbrock();

        // Create new solver
        KTRSolver solver = new KTRSolver(instance);

        // Create new solver
        solver.setParam(KTRConstants.KTR_PARAM_DERIVCHECK, KTRConstants.KTR_DERIVCHECK_ALL);

        // Solve problem. Will return after 11 evaluations (11th evaluation is preempted and returns user termination)
        solver.solve();
    }

    /**
     * Running the LSQ exmaple
     */
    private static void runLSQ() throws KTRException
    {
        // Instantiate LSQ problem */
        ProblemLSQ instance = new ProblemLSQ();
        List<Integer> resTypes = new ArrayList<Integer>();
        for (int i=0 ; i<instance.getNumRes() ; i++) {
            resTypes.add(i, KTRConstants.KTR_RESTYPE_GENERAL);
        }
        instance.setResTypes(resTypes);

        // Create new KTRSolverLSQ
        KTRSolverLSQ solver = new KTRSolverLSQ(instance);

        // Solve lsq problem
        solver.solve();
    }

    /**
     * A simple example solving an MINLP using the default options
     */
    private static void runMINLP() throws KTRException
    {
        // Instantiate problem
        ProblemMINLP instance = new ProblemMINLP();

        // Create new solver
        KTRSolver solver = new KTRSolver(instance);

        // Solve problem
        solver.solve();
    }

    /**
     * A simple example solving an MINLP using the MISQP algorithm
     */
    private static void runMINLPmisqp() throws KTRException
    {
        // Instantiate problem
        ProblemMINLP instance = new ProblemMINLP();

        // Create new solver
        KTRSolver solver = new KTRSolver(instance);

        // Set MISQP option
        solver.setParam(KTRConstants.KTR_PARAM_MIP_METHOD, KTRConstants.KTR_MIP_METHOD_MISQP);
        solver.setParam("algorithm", KTRConstants.KTR_ALG_ACT_CG);

        // Solve problem
        solver.solve();
    }

    /**
     * A simple example solving an MINLP after reformulating some integer variables
     */
    private static void runMINLPreform() throws KTRException
    {
        // Instantiate problem
        ProblemMINLP instance = new ProblemMINLP();

        // Create new solver
        KTRSolver solver = new KTRSolver(instance);

        // Relax variables
        solver.setIntVarStrategy(4, KTRConstants.KTR_MIP_INTVAR_STRATEGY_RELAX);
        solver.setIntVarStrategy(5, KTRConstants.KTR_MIP_INTVAR_STRATEGY_RELAX);

        solver.setParam("algorithm", KTRConstants.KTR_ALG_ACT_CG);

        // Solve problem
        solver.solve();
    }

    /**
     * A simple example solving a QCQP with a few user options and user-defined callback for init point.<br/>
     * Optimization is performed using forward gradient evaluation and finite differences.
     * The multi-start is enabled with user-defined initial point and callback on each solution.
     * Finally, Knitro output is redirected to another callback (writing it to standard error).
     */
    private static void runQCQP() throws KTRException
    {
        // Instantiate problem
        ProblemQCQP instance = new ProblemQCQP();

        // ===== Define additional callbacks ===== //

        // Set user defined callback for multi-start init point
        ExampleMSInitPtCallback initPtCallback = new ExampleMSInitPtCallback(0, 5000);
        instance.setMSInitPtCallback(initPtCallback);

        // Set a callback which is called after each multi-start solve
        ExampleMSProcessCallback processCallback = new ExampleMSProcessCallback();
        instance.setMSProcessCallback(processCallback);

        // Redirect Knitro output to standard error
        instance.setPutStringFunction(new ExampleOutputRedirection());

        // ===== Create and solve instance ===== //
        // Create a new solver and set it to use forward finite-differences to compute the gradient
        // and BFGS for the hessian
        KTRSolver solver = new KTRSolver(instance, KTRConstants.KTR_GRADOPT_FORWARD, KTRConstants.KTR_HESSOPT_BFGS);

        // Set additional parameters
        solver.setParam(KTRConstants.KTR_PARAM_OUTLEV, 4);
        solver.setParam(KTRConstants.KTR_PARAM_OUTMODE, 2);
        solver.setParam(KTRConstants.KTR_PARAM_DEBUG, 1);

        // Set multi-start parameter to true and tell knitro to use user-defined callback for initial points
        solver.useMSInitptCallback();

        // Call post solve callback
        solver.useMSProcessCallback();

        // ===== Solve instance ===== //
        solver.solve();

        // ===== Modify variable bounds and callback parameters and resolve ===== //
        // Disable callback verbosity
        initPtCallback.setVerbosity(false);
        processCallback.setEnabled(false);

        // Modify bounds: makes previous best solution infeasible
        instance.setVarUpBnds(7.0);

        // Solve
        solver.solve();
    }
}
