/*******************************************************/
/* Copyright (c) 2015 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/

package com.artelys.knitro.examples.Problems;

import com.artelys.knitro.api.KTRConstants;
import com.artelys.knitro.api.KTREnums;
import com.artelys.knitro.api.KTRProblem;

import java.util.List;

public class ProblemQCQP extends KTRProblem
{
    public ProblemQCQP() {
        super(3, 2, 6, 5);
        setObjectiveProperties();
        setVariableProperties();
        setConstraintProperties();
        setDerivativeProperties();
    }

    private void setObjectiveProperties() {
        setObjType(KTREnums.ObjectiveType.ObjQuadratic.getValue());
        setObjGoal(KTREnums.ObjectiveGoal.Minimize.getValue());
    }

    private void setDerivativeProperties() {
        setJacIndexCons(0, 0);
        setJacIndexCons(1, 0);
        setJacIndexCons(2, 0);
        setJacIndexCons(3, 1);
        setJacIndexCons(4, 1);
        setJacIndexCons(5, 1);

        setJacIndexVars(0, 0);
        setJacIndexVars(1, 1);
        setJacIndexVars(2, 2);
        setJacIndexVars(3, 0);
        setJacIndexVars(4, 1);
        setJacIndexVars(5, 2);

        setHessIndexRows(0, 0);
        setHessIndexRows(1, 0);
        setHessIndexRows(2, 0);
        setHessIndexRows(3, 1);
        setHessIndexRows(4, 2);

        setHessIndexCols(0, 0);
        setHessIndexCols(1, 1);
        setHessIndexCols(2, 2);
        setHessIndexCols(3, 1);
        setHessIndexCols(4, 2);
    }

    private void setConstraintProperties() {
        setConTypes(0, KTREnums.ConstraintType.ConLinear.getValue());
        setConTypes(1, KTREnums.ConstraintType.ConQuadratic.getValue());

        setConLoBnds(0, 0.0);
        setConUpBnds(0, 0.0);

        setConLoBnds(1, 0.0);
        setConUpBnds(1, KTRConstants.KTR_INFBOUND);
    }

    private void setVariableProperties() {
        setVarLoBnds(0, 0.0);
        setVarUpBnds(0, KTRConstants.KTR_INFBOUND);

        setVarLoBnds(1, 0.0);
        setVarUpBnds(1, KTRConstants.KTR_INFBOUND);

        setVarLoBnds(2, 0.0);
        setVarUpBnds(2, KTRConstants.KTR_INFBOUND);

        setXInitial(0, 2.0);
        setXInitial(1, 2.0);
        setXInitial(2, 2.0);
    }

    @Override
    public double evaluateFC(
            List<Double> x,
            List<Double> c,
            List<Double> objGrad,
            List<Double> jac) {
        /*---- LINEAR EQUALITY CONSTRAINT. */
        c.set(0, 8.0 * x.get(0) + 14.0 * x.get(1) + 7.0 * x.get(2) - 56.0);

            /*---- QUADRATIC INEQUALITY CONSTRAINTS. */
        c.set(1, x.get(0) * x.get(0) + x.get(1) * x.get(1) + x.get(2) * x.get(2) - 25.0);

        return 1000 - x.get(0) * x.get(0) - 2.0 * x.get(1) * x.get(1) - x.get(2) * x.get(2)
                - x.get(0) * x.get(1) - x.get(0) * x.get(2);
    }

    @Override
    public int evaluateGA(
            List<Double> x,
            List<Double> objGrad,
            List<Double> jac) {

        objGrad.set(0, -2.0e0 * x.get(0) - x.get(1) - x.get(2));
        objGrad.set(1, -4.0e0 * x.get(1) - x.get(0));
        objGrad.set(2, -2.0e0 * x.get(2) - x.get(0));

        /*---- GRADIENT OF THE FIRST CONSTRAINT, c[0]. */
        jac.set(0, 8.0e0);
        jac.set(1, 14.0e0);
        jac.set(2, 7.0e0);

        /*---- GRADIENT OF THE SECOND CONSTRAINT, c[1]. */
        jac.set(3, 2.0e0 * x.get(0));
        jac.set(4, 2.0e0 * x.get(1));
        jac.set(5, 2.0e0 * x.get(2));

        return 0;
    }

    @Override
    public int evaluateHess(
            List<Double> x,
            double objScaler,
            List<Double> lambda,
            List<Double> hess) {
        hess.set(0, -2.0e0 * objScaler + 2.0e0 * lambda.get(1));
        hess.set(1, -1.0e0 * objScaler);
        hess.set(2, -1.0e0 * objScaler);
        hess.set(3, -4.0e0 * objScaler + 2.0e0 * lambda.get(1));
        hess.set(4, -2.0e0 * objScaler + 2.0e0 * lambda.get(1));
        return 0;
    }
}
