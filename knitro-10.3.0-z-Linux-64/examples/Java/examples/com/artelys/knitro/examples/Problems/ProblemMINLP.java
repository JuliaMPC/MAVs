/*******************************************************/
/* Copyright (c) 2015 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/

package com.artelys.knitro.examples.Problems;

import com.artelys.knitro.api.KTRConstants;
import com.artelys.knitro.api.KTREnums;
import com.artelys.knitro.api.KTRProblem;

import java.util.List;

public class ProblemMINLP extends KTRProblem
{
    public ProblemMINLP() {
        super(6, 6, 16, 3);
        setObjectiveProperties();
        setVariableProperties();
        setConstraintProperties();
        setDerivativeProperties();
    }

    private void setObjectiveProperties() {
        setObjType(KTREnums.ObjectiveType.ObjGeneral.getValue());
        setObjFnType(KTREnums.FunctionType.Convex.getValue());
        setObjGoal(KTREnums.ObjectiveGoal.Minimize.getValue());
    }

    private void setConstraintProperties() {
        setConTypes(0, KTREnums.ConstraintType.ConGeneral.getValue());
        setConTypes(1, KTREnums.ConstraintType.ConGeneral.getValue());
        setConTypes(2, KTREnums.ConstraintType.ConLinear.getValue());
        setConTypes(3, KTREnums.ConstraintType.ConLinear.getValue());
        setConTypes(4, KTREnums.ConstraintType.ConLinear.getValue());
        setConTypes(5, KTREnums.ConstraintType.ConLinear.getValue());

        setConLoBnds(0, 0.0);
        setConLoBnds(1, -2.0);
        setConLoBnds(2, -KTRConstants.KTR_INFBOUND);
        setConLoBnds(3, -KTRConstants.KTR_INFBOUND);
        setConLoBnds(4, -KTRConstants.KTR_INFBOUND);
        setConLoBnds(5, -KTRConstants.KTR_INFBOUND);

        setConUpBnds(0, KTRConstants.KTR_INFBOUND);
        setConUpBnds(1, KTRConstants.KTR_INFBOUND);
        setConUpBnds(2, 0.0);
        setConUpBnds(3, 0.0);
        setConUpBnds(4, 0.0);
        setConUpBnds(5, 1.0);

        setConFnTypes(0, KTREnums.FunctionType.Convex.getValue());
        setConFnTypes(1, KTREnums.FunctionType.Convex.getValue());
        setConFnTypes(2, KTREnums.FunctionType.Convex.getValue());
        setConFnTypes(3, KTREnums.FunctionType.Convex.getValue());
        setConFnTypes(4, KTREnums.FunctionType.Convex.getValue());
        setConFnTypes(5, KTREnums.FunctionType.Convex.getValue());
    }

    private void setVariableProperties() {
        setVarLoBnds(0.0);
        setVarUpBnds(0, 2.0);
        setVarUpBnds(1, 2.0);
        setVarUpBnds(2, 1.0);
        setVarUpBnds(3, 1.0);
        setVarUpBnds(4, 1.0);
        setVarUpBnds(5, 1.0);

        setVarTypes(0, KTREnums.VariableType.Continuous.getValue());
        setVarTypes(1, KTREnums.VariableType.Continuous.getValue());
        setVarTypes(2, KTREnums.VariableType.Continuous.getValue());
        setVarTypes(3, KTREnums.VariableType.Binary.getValue());
        setVarTypes(4, KTREnums.VariableType.Binary.getValue());
        setVarTypes(5, KTREnums.VariableType.Binary.getValue());
    }

    private void setDerivativeProperties() {
        setJacIndexCons(0, 0);
        setJacIndexCons(1, 0);
        setJacIndexCons(2, 0);

        setJacIndexCons(3, 1);
        setJacIndexCons(4, 1);
        setJacIndexCons(5, 1);
        setJacIndexCons(6, 1);

        setJacIndexCons(7, 2);
        setJacIndexCons(8, 2);

        setJacIndexCons(9, 3);
        setJacIndexCons(10, 3);

        setJacIndexCons(11, 4);
        setJacIndexCons(12, 4);
        setJacIndexCons(13, 4);

        setJacIndexCons(14, 5);
        setJacIndexCons(15, 5);

        setJacIndexVars(0, 0);
        setJacIndexVars(1, 1);
        setJacIndexVars(2, 2);

        setJacIndexVars(3, 0);
        setJacIndexVars(4, 1);
        setJacIndexVars(5, 2);

        setJacIndexVars(6, 5);
        setJacIndexVars(7, 0);
        setJacIndexVars(8, 1);
        setJacIndexVars(9, 1);

        setJacIndexVars(10, 3);
        setJacIndexVars(11, 0);
        setJacIndexVars(12, 1);

        setJacIndexVars(13, 4);
        setJacIndexVars(14, 3);
        setJacIndexVars(15, 4);

        setHessIndexRows(0, 0);
        setHessIndexRows(1, 0);
        setHessIndexRows(2, 1);

        setHessIndexCols(0, 0);
        setHessIndexCols(1, 1);
        setHessIndexCols(2, 1);
    }

    @Override
    public double evaluateFC(
            List<Double> x,
            List<Double> c,
            List<Double> objGrad,
            List<Double> jac) {
        double tmp1 = x.get(0) - x.get(1) + 1.0;
        double tmp2 = x.get(1) + 1.0;
        double obj = 5.0 * x.get(3) + 6.0 * x.get(4) + 8.0 * x.get(5) + 10.0 * x.get(0) - 7.0 * x.get(2)
                - 18.0 * Math.log(tmp2) - 19.2 * Math.log(tmp1) + 10.0;
        c.set(0, 0.8 * Math.log(tmp2) + 0.96 * Math.log(tmp1) - 0.8 * x.get(2));
        c.set(1, Math.log(tmp2) + 1.2 * Math.log(tmp1) - x.get(2) - 2 * x.get(5));
        c.set(2, x.get(1) - x.get(0));
        c.set(3, x.get(1) - 2 * x.get(3));
        c.set(4, x.get(0) - x.get(1) - 2 * x.get(4));
        c.set(5, x.get(3) + x.get(4));

        return obj;
    }

    @Override
    public int evaluateGA(
            List<Double> x,
            List<Double> objGrad,
            List<Double> jac) {
        double tmp1 = x.get(0) - x.get(1) + 1.0;
        double tmp2 = x.get(1) + 1.0;
        objGrad.set(0, 10.0 - (19.2 / tmp1));
        objGrad.set(1, (-18.0 / tmp2) + (19.2 / tmp1));
        objGrad.set(2, -7.0);
        objGrad.set(3, 5.0);
        objGrad.set(4, 6.0);
        objGrad.set(5, 8.0);

            /*---- GRADIENT OF CONSTRAINT 0. */
        jac.set(0, 0.96 / tmp1);
        jac.set(1, (-0.96 / tmp1) + (0.8 / tmp2));
        jac.set(2, -0.8);
            /*---- GRADIENT OF CONSTRAINT 1. */
        jac.set(3, 1.2 / tmp1);
        jac.set(4, (-1.2 / tmp1) + (1.0 / tmp2));
        jac.set(5, -1.0);
        jac.set(6, -2.0);
            /*---- GRADIENT OF CONSTRAINT 2. */
        jac.set(7, -1.0);
        jac.set(8, 1.0);
            /*---- GRADIENT OF CONSTRAINT 3. */
        jac.set(9, 1.0);
        jac.set(10, -2.0);
            /*---- GRADIENT OF CONSTRAINT 4. */
        jac.set(11, 1.0);
        jac.set(12, -1.0);
        jac.set(13, -2.0);
            /*---- GRADIENT OF CONSTRAINT 5. */
        jac.set(14, 1.0);
        jac.set(15, 1.0);

        return 0;
    }

    @Override
    public int evaluateHess(
            List<Double> x,
            double objScaler,
            List<Double> lambda,
            List<Double> hess) {
        double tmp1 = x.get(0) - x.get(1) + 1.0;
        double tmp2 = x.get(1) + 1.0;
        hess.set(0, objScaler * (19.2 / (tmp1 * tmp1))
                + lambda.get(0) * (-0.96 / (tmp1 * tmp1))
                + lambda.get(1) * (-1.2 / (tmp1 * tmp1)));
        hess.set(1, objScaler * (-19.2 / (tmp1 * tmp1))
                + lambda.get(0) * (0.96 / (tmp1 * tmp1))
                + lambda.get(1) * (1.2 / (tmp1 * tmp1)));
        hess.set(2, objScaler * ((19.2 / (tmp1 * tmp1)) + (18.0 / (tmp2 * tmp2)))
                + lambda.get(0) * ((-0.96 / (tmp1 * tmp1)) - (0.8 / (tmp2 * tmp2)))
                + lambda.get(1) * ((-1.2 / (tmp1 * tmp1)) - (1.0 / (tmp2 * tmp2))));

        return 0;
    }

    @Override
    public int evaluateHessianVector(
            List<Double> x,
            double objScaler,
            List<Double> lambda,
            List<Double> hessVector) {
        double[] tmpVec = new double[2];

        double tmp1 = x.get(0) - x.get(1) + 1.0;
        double tmp2 = x.get(1) + 1.0;

            /*---- H[0,0]*v[0] + H[0,1]*v[1]. */
        tmpVec[0] = (objScaler * (19.2 / (tmp1 * tmp1))
                + lambda.get(0) * (-0.96 / (tmp1 * tmp1))
                + lambda.get(1) * (-1.2 / (tmp1 * tmp1))) * hessVector.get(0)
                + (objScaler * (-19.2 / (tmp1 * tmp1))
                + lambda.get(0) * (0.96 / (tmp1 * tmp1))
                + lambda.get(1) * (1.2 / (tmp1 * tmp1))) * hessVector.get(1);

            /*---- H[1,0]*v[0] + H[1,1]*v[1]. */
        tmpVec[1] = (objScaler * (-19.2 / (tmp1 * tmp1))
                + lambda.get(0) * (0.96 / (tmp1 * tmp1))
                + lambda.get(1) * (1.2 / (tmp1 * tmp1))) * hessVector.get(0)
                + (objScaler * ((19.2 / (tmp1 * tmp1)) + (18.0 / (tmp2 * tmp2)))
                + lambda.get(0) * ((-0.96 / (tmp1 * tmp1)) - (0.8 / (tmp2 * tmp2)))
                + lambda.get(1) * ((-1.2 / (tmp1 * tmp1)) - (1.0 / (tmp2 * tmp2)))) * hessVector.get(1);

        hessVector.set(0, tmpVec[0]);
        hessVector.set(1, tmpVec[1]);
        hessVector.set(2, 0.0);
        hessVector.set(3, 0.0);
        hessVector.set(4, 0.0);
        hessVector.set(5, 0.0);
        return 0;
    }
}
