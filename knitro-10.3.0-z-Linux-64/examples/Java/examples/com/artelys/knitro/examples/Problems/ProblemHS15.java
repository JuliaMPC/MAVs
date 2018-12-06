/*******************************************************/
/* Copyright (c) 2015 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/

package com.artelys.knitro.examples.Problems;

import com.artelys.knitro.api.KTRConstants;
import com.artelys.knitro.api.KTREnums;
import com.artelys.knitro.api.KTRProblem;

import java.util.List;

public class ProblemHS15 extends KTRProblem
{
    public ProblemHS15() {
        super(2, 2, 4, 3);
        setObjectiveProperties();
        setVariableProperties();
        setConstraintProperties();
        setDerivativeProperties();
    }

    private void setObjectiveProperties() {
        setObjType(KTREnums.ObjectiveType.ObjGeneral.getValue());
        setObjGoal(KTREnums.ObjectiveGoal.Minimize.getValue());
    }

    private void setDerivativeProperties() {
        setJacIndexCons(0, 0);
        setJacIndexCons(1, 0);
        setJacIndexCons(2, 1);
        setJacIndexCons(3, 1);

        setJacIndexVars(0, 0);
        setJacIndexVars(1, 1);
        setJacIndexVars(2, 0);
        setJacIndexVars(3, 1);

        // Provide second derivative structural information;
        // Only the nonzeros of the upper triangle of the Hessian.

        setHessIndexRows(0, 0);
        setHessIndexRows(1, 0);
        setHessIndexRows(2, 1);

        setHessIndexCols(0, 0);
        setHessIndexCols(1, 1);
        setHessIndexCols(2, 1);
    }

    private void setConstraintProperties() {
        setConTypes(0, KTREnums.ConstraintType.ConQuadratic.getValue());
        setConTypes(1, KTREnums.ConstraintType.ConQuadratic.getValue());

        setConLoBnds(0, 1.0);
        setConLoBnds(1, 0.0);

        setConUpBnds(0, KTRConstants.KTR_INFBOUND);
        setConUpBnds(1, KTRConstants.KTR_INFBOUND);
    }

    private void setVariableProperties() {
        setVarLoBnds(0, -KTRConstants.KTR_INFBOUND);
        setVarLoBnds(1, -KTRConstants.KTR_INFBOUND);

        setVarUpBnds(0, 0.5);
        setVarUpBnds(1, KTRConstants.KTR_INFBOUND);

        setXInitial(0, -2.0);
        setXInitial(1, 1.0);
    }

    @Override
    public double evaluateFC(
            List<Double> x,
            List<Double> c,
            List<Double> objGrad,
            List<Double> jac) {
        double tmp = x.get(1) - x.get(0) * x.get(0);
        double obj = 100.0 * (tmp * tmp) + ((1.0 - x.get(0)) * (1.0 - x.get(0)));
        c.set(0, x.get(0) * x.get(1));
        c.set(1, x.get(0) + (x.get(1) * x.get(1)));

        return obj;
    }

    @Override
    public int evaluateGA(
            List<Double> x,
            List<Double> objGrad,
            List<Double> jac) {

        double tmp = x.get(1) - x.get(0) * x.get(0);
        objGrad.set(0, (-400.0 * tmp * x.get(0)) - (2.0 * (1.0 - x.get(0))));
        objGrad.set(1, 200.0 * tmp);

        jac.set(0, x.get(1));
        jac.set(1, x.get(0));
        jac.set(2, 1.0);
        jac.set(3, 2.0 * x.get(1));

        return 0;
    }

    @Override
    public int evaluateHess(
            List<Double> x,
            double objScaler,
            List<Double> lambda,
            List<Double> hess) {
        hess.set(0, objScaler * ((-400.0 * x.get(1)) + (1200.0 * x.get(0) * x.get(0)) + 2.0));
        hess.set(1, (objScaler * (-400.0 * x.get(0))) + lambda.get(0));
        hess.set(2, (objScaler * 200.0) + (lambda.get(1) * 2.0));
        return 0;
    }

    @Override
    public int evaluateHessianVector(
            List<Double> x,
            double objScaler,
            List<Double> lambda,
            List<Double> vector) {

        double[] tmp = new double[2];

    /*---- H[0,0]*v[0] + H[0,1]*v[1]. */
        tmp[0] = (objScaler * (((-400.0 * x.get(1)) + (1200.0 * x.get(0) * x.get(0)) + 2.0))) * vector.get(0)
                + (objScaler * (-400.0 * x.get(0)) + lambda.get(0)) * vector.get(1);

    /*---- H[1,0]*v[0] + H[1,1]*v[1]. */
        tmp[1] = (objScaler * (-400.0 * x.get(0)) + lambda.get(0)) * vector.get(0)
                + (objScaler * 200.0 + (lambda.get(1) * 2.0)) * vector.get(1);

        vector.set(0, tmp[0]);
        vector.set(1, tmp[1]);
        return 0;
    }
}
