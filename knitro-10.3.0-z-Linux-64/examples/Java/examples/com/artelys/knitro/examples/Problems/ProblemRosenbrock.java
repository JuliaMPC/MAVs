/*******************************************************/
/* Copyright (c) 2015 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/

package com.artelys.knitro.examples.Problems;

import com.artelys.knitro.api.KTRConstants;
import com.artelys.knitro.api.KTREnums;
import com.artelys.knitro.api.KTRException;
import com.artelys.knitro.api.KTRProblem;

import java.util.ArrayList;
import java.util.List;

public class ProblemRosenbrock extends KTRProblem
{

    public ProblemRosenbrock() throws KTRException
    {
        super(2, 0, 0, 3);
        setObjectiveProperties();
        setVariableProperties();
        setDerivativeProperties();
    }

    @Override
    public int evaluateGA(
            List<Double> x,
            List<Double> objGrad,
            List<Double> jac) {

        return 0;
    }

    @Override
    public double evaluateFC(
            List<Double> x,
            List<Double> c,
            List<Double> objGrad,
            List<Double> jac) {

        double z = 1 - x.get(0);
        double w = x.get(1) - x.get(0) * x.get(0);

        // Evaluate objective function and constraint simultaneously.

        objGrad.set(0, -2 * z - 400 * x.get(0) * w);
        objGrad.set(1, 200 * w);

        return z * z + 100 * w * w;
    }

    @Override
    public int evaluateHess(
            List<Double> x,
            double objScaler,
            List<Double> lambda,
            List<Double> hess) {

        hess.set(0, (2 - 400 * x.get(1) + 1200 * x.get(0) * x.get(0)) * objScaler);
        hess.set(1, -400 * x.get(0) * objScaler);
        hess.set(2, 200 * objScaler);
        return 0;
    }

    private void setObjectiveProperties() {
        this.setObjType(KTREnums.ObjectiveType.ObjGeneral.getValue());
        this.setObjGoal(KTREnums.ObjectiveGoal.Minimize.getValue());
    }

    private void setVariableProperties() {
        this.setVarLoBnds(-KTRConstants.KTR_INFBOUND);
        this.setVarUpBnds(KTRConstants.KTR_INFBOUND);
    }

    private void setDerivativeProperties() throws KTRException {
        List<Integer> hessCols = new ArrayList<Integer>(3);
        List<Integer> hessRows = new ArrayList<Integer>(3);

        hessRows.add(0, 0);
        hessCols.add(0, 0);
        hessRows.add(1, 0);
        hessCols.add(1, 1);
        hessRows.add(2, 1);
        hessCols.add(2, 1);

        this.setHessIndexCols(hessCols);
        this.setHessIndexRows(hessRows);
    }
}
