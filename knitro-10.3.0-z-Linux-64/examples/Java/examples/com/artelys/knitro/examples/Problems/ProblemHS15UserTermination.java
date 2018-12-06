/*******************************************************/
/* Copyright (c) 2015 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/

package com.artelys.knitro.examples.Problems;

import com.artelys.knitro.api.KTRConstants;
import com.artelys.knitro.api.KTREnums;
import com.artelys.knitro.api.KTRProblem;

import java.util.List;


public class ProblemHS15UserTermination extends ProblemHS15
{
    private int evaluationCounter;
    private int maxEvaluations;

    // This constructor is called by reflection from AllPoblemSolver
    public ProblemHS15UserTermination() {
        this(10);
    }

    public ProblemHS15UserTermination(int maxEvaluations) {
        this.evaluationCounter = 0;
        this.maxEvaluations = maxEvaluations;
    }

    @Override
    public int evaluateFC(
            List<Double> x,
            List<Double> obj,
            List<Double> c,
            List<Double> objGrad,
            List<Double> jac) {
        // Check evaluation count
        // Using a lock for accurate stop when using parallel features
        synchronized (this)
        {
            if (evaluationCounter >= maxEvaluations)
                return KTRConstants.KTR_RC_USER_TERMINATION;
            ++evaluationCounter;
        }

        obj.set(0, super.evaluateFC(x, c, objGrad, jac));

        return 0;
    }
}
