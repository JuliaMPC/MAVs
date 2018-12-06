/*******************************************************/
/* Copyright (c) 2015 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/

package com.artelys.knitro.examples.Problems;

import com.artelys.knitro.api.KTRConstants;
import com.artelys.knitro.api.KTREnums;
import com.artelys.knitro.api.KTRProblemLSQ;

import java.util.List;
import java.lang.*;

public class ProblemLSQ extends KTRProblemLSQ {

	public ProblemLSQ() {
		super(2, 6, 12);	
		setObjectiveProperties();
		setVariableProperties();
		setDerivativeProperties();
	}

	@Override
	public int evaluateResidual(List<Double> x, List<Double> res) {
		res.set(0, x.get(0) * Math.pow(1.309, x.get(1)) - 2.138);
		res.set(1, x.get(0) * Math.pow(1.471, x.get(1)) - 3.421);
		res.set(2, x.get(0) * Math.pow(1.49, x.get(1)) - 3.597);
		res.set(3, x.get(0) * Math.pow(1.565, x.get(1)) - 4.34);
		res.set(4, x.get(0) * Math.pow(1.611, x.get(1)) - 4.882);
		res.set(5, x.get(0) * Math.pow(1.68, x.get(1)) - 5.66);
	
		return 0;
	}

	@Override
	public int evaluateJacobian(List<Double> x, List<Double> jac) {
  		jac.set(0, Math.pow(1.309, x.get(1)));
  		jac.set(1, x.get(0) * Math.log(1.309) * Math.pow(1.309, x.get(1)));

  		jac.set(2, Math.pow(1.471, x.get(1)));
  		jac.set(3, x.get(0) * Math.log(1.471) * Math.pow(1.471, x.get(1)));

  		jac.set(4, Math.pow(1.49, x.get(1)));
  		jac.set(5, x.get(0) * Math.log(1.49) * Math.pow(1.49, x.get(1)));

  		jac.set(6, Math.pow(1.565, x.get(1)));
  		jac.set(7, x.get(0) * Math.log(1.565) * Math.pow(1.565, x.get(1)));

  		jac.set(8, Math.pow(1.611, x.get(1)));
  		jac.set(9, x.get(0) * Math.log(1.611) * Math.pow(1.611, x.get(1)));

  		jac.set(10, Math.pow(1.68, x.get(1)));
  		jac.set(11, x.get(0) * Math.log(1.68) * Math.pow(1.68, x.get(1)));

		return 0;
	}
 
	private void setObjectiveProperties() {
        setObjGoal(KTREnums.ObjectiveGoal.Minimize.getValue());
    }

    private void setVariableProperties() {
    	setVarLoBnds(0, -KTRConstants.KTR_INFBOUND);
    	setVarLoBnds(1, -KTRConstants.KTR_INFBOUND);

    	setVarUpBnds(0, KTRConstants.KTR_INFBOUND);
    	setVarUpBnds(1, KTRConstants.KTR_INFBOUND);

    	setXInitial(0, 1.0);
    	setXInitial(1, 5.0);
    }

   	private void setDerivativeProperties() {
   		setJacIndexRes(0, 0); setJacIndexRes(1, 0);
   		setJacIndexRes(2, 1); setJacIndexRes(3, 1);
   		setJacIndexRes(4, 2); setJacIndexRes(5, 2);
   		setJacIndexRes(6, 3); setJacIndexRes(7, 3);
   		setJacIndexRes(8, 4); setJacIndexRes(9, 4);
   		setJacIndexRes(10, 5); setJacIndexRes(11, 5);


   		setJacIndexVars(0, 0); setJacIndexVars(1, 1);
   		setJacIndexVars(2, 0); setJacIndexVars(3, 1);
   		setJacIndexVars(4, 0); setJacIndexVars(5, 1);
   		setJacIndexVars(6, 0); setJacIndexVars(7, 1);
   		setJacIndexVars(8, 0); setJacIndexVars(9, 1);
   		setJacIndexVars(10, 0); setJacIndexVars(11, 1);
   	}

}


