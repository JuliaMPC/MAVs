/*******************************************************/
/* Copyright (c) 2015 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/

package com.artelys.knitro.examples.Problems;

import com.artelys.knitro.api.KTRConstants;
import com.artelys.knitro.api.KTREnums;
import com.artelys.knitro.api.KTRProblem;

import java.util.List;

public class ProblemRosenbrockExtended extends KTRProblem
{

    /** Build a Rosenbrock problem with 30 dimensions */
    public ProblemRosenbrockExtended() {
        this(5);
    }

    /** Constructor: build extended Rosenbrock problem
     * @param nVars number of dimensions of the Rosenbrock function (>= 2)
     */
    public ProblemRosenbrockExtended(int nVars) {
        super(nVars, 0);
        SetObjectiveProperties();
        SetVariableProperties();
    }

    @Override
    public double evaluateFC
            (
                    List<Double> x,
                    List<Double> c,
                    List<Double> objGrad,
                    List<Double> jac) {
        double obj = 0;
        for (int i = 0; i < getNumVars() - 1; i++) {
            obj += (1 - x.get(i)) * (1 - x.get(i)) + 100 * (x.get(i + 1) - x.get(i) * x.get(i)) * (x.get(i + 1) - x.get(i) * x.get(i));
        }

        return obj;
    }

    private void SetVariableProperties() {
        this.setVarLoBnds(-KTRConstants.KTR_INFBOUND);
        this.setVarUpBnds(KTRConstants.KTR_INFBOUND);
    }

    private void SetObjectiveProperties() {
        setObjType(KTREnums.ObjectiveType.ObjGeneral.getValue());
        setObjGoal(KTREnums.ObjectiveGoal.Minimize.getValue());
    }

}
