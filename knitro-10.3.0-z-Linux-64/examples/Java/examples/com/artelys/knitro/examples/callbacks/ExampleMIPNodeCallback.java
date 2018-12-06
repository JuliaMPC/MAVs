/*******************************************************/
/* Copyright (c) 2015 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/

package com.artelys.knitro.examples.callbacks;

import com.artelys.knitro.api.KTRConstants;
import com.artelys.knitro.api.KTRException;
import com.artelys.knitro.api.KTRMipNodeCallback;
import com.artelys.knitro.api.KTRISolver;
import com.artelys.knitro.api.KTRSolver;

import java.util.List;


/**
 * A MIP call back example.
 */
public class ExampleMIPNodeCallback extends KTRMipNodeCallback
{

    @Override
    public int callbackFunction(
            List<Double> x,
            List<Double> lambda,
            double obj,
            List<Double> c,
            KTRISolver isolver) throws KTRException
    {
        KTRSolver solver = null; 
        if (isolver instanceof KTRSolver) {
            solver = (KTRSolver) isolver;
        }

        /*---- PRINT INFO ABOUT THE STATUS OF THE MIP SOLUTION. */
        System.out.println("callbackProcessNode:");
        System.out.format("    Node number    = %1$s", solver.getMipNumNodes());
        System.out.format("    Node objective = %1$s", obj);
        System.out.format("    Current relaxation bound = %1$s", solver.getMipRelaxationBnd());

        double incumbentBound = solver.getMipIncumbentObj();

        if (Math.abs(incumbentBound) >= KTRConstants.KTR_INFBOUND) {
            System.out.println("    No integer feasible point found yet.");
        } else {
            System.out.format("    Current incumbent bound  = %1$s", incumbentBound);
            System.out.format("    Absolute integrality gap = %1$s", solver.getMipAbsGap());
            System.out.format("    Relative integrality gap = %1$s", solver.getMipRelGap());
        }

        return 0;
    }
}
