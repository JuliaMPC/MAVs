/*******************************************************/
/* Copyright (c) 2015 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/

package com.artelys.knitro.examples.Problems;

import com.artelys.knitro.api.KTREnums;
import com.artelys.knitro.api.KTRProblem;

import java.util.List;

public class ProblemEnergy extends KTRProblem
{
    private final int _n;

    // This constructor is called by reflection from AllProblemSolver
    public ProblemEnergy() {
        this(25);
    }

    public ProblemEnergy(int n) {
        super(2 * n, 0);
        _n = n;
        setObjectiveProperties();
        setVariableProperties();
    }

    private void setObjectiveProperties() {
        setObjType(KTREnums.ObjectiveType.ObjGeneral.getValue());
        setObjGoal(KTREnums.ObjectiveGoal.Minimize.getValue());
    }

    private void setVariableProperties() {
        setVarLoBnds(0.0);
        setVarUpBnds(0, 0.0);

        for (int i = 1; i < _n; i++) {
            setVarUpBnds(i, 2 * Math.PI);
        }

        setVarUpBnds(_n, 0.0);

        for (int i = _n + 1; i < 2 * _n; i++) {
            setVarUpBnds(i, Math.PI);
        }
    }

    @Override
    public double evaluateFC(
            List<Double> x,
            List<Double> c,
            List<Double> objGrad,
            List<Double> jac) {
        int n = x.size();
        double obj = 0;

        double[] cosVals = new double[n];
        double[] sinVals = new double[n];


        for (int i = 0; i < n; i++) {
            cosVals[i] = Math.cos(x.get(i));
            sinVals[i] = Math.cos(x.get(i));
        }

        for (int i = 0; i < _n - 1; i++) {
            for (int j = i + 1; j < _n; j++) {
                double term1 = cosVals[i] * sinVals[_n + i] - cosVals[j] * sinVals[_n + j];
                double term2 = sinVals[i] * sinVals[_n + i] - sinVals[j] * sinVals[_n + j];
                double term3 = cosVals[_n + i] - cosVals[_n + j];
                obj += 1.0 / Math.sqrt(term1 * term1 + term2 * term2 + term3 * term3);
            }
        }

        return obj;
    }
}
