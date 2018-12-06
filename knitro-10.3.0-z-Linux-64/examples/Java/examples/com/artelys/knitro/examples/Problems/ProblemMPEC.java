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

public class ProblemMPEC extends KTRProblem
{
    public ProblemMPEC() throws KTRException
    {
        super(8, 4);
        setObjectiveProperties();
        setVariableProperties();
        setConstraintProperties();
        setComplementarityProperties();
    }

    private void setComplementarityProperties() throws KTRException {
        List<Integer> indexList1 = new ArrayList<Integer>();
        indexList1.add(2);
        indexList1.add(3);
        indexList1.add(4);

        List<Integer> indexList2 = new ArrayList<Integer>();
        indexList2.add(5);
        indexList2.add(6);
        indexList2.add(7);

        SetComplementarity(indexList1, indexList2);
    }

    private void setObjectiveProperties() {
        setObjType(KTREnums.ObjectiveType.ObjGeneral.getValue());
        setObjGoal(KTREnums.ObjectiveGoal.Minimize.getValue());
    }

    private void setConstraintProperties() {
        setConTypes(KTREnums.ConstraintType.ConLinear.getValue());

        setConLoBnds(0.0);
        setConUpBnds(0.0);
    }

    private void setVariableProperties() {
        setVarLoBnds(0.0);
        setVarUpBnds(KTRConstants.KTR_INFBOUND);
    }

    @Override
    public double evaluateFC(
            List<Double> x,
            List<Double> c,
            List<Double> objGrad,
            List<Double> jac) {

        c.set(0, 2 * (x.get(1) - 1) - 1.5 * x.get(0) + x.get(2) - 0.5 * x.get(3) + x.get(4));
        c.set(1, 3 * x.get(0) - x.get(1) - 3 - x.get(5));
        c.set(2, -x.get(0) + 0.5 * x.get(1) + 4 - x.get(6));
        c.set(3, -x.get(0) - x.get(1) + 7 - x.get(7));


        return (x.get(0) - 5) * (x.get(0) - 5) + (2 * x.get(1) + 1) * (2 * x.get(1) + 1);
    }
}
