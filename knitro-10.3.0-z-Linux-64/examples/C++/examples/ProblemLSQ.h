#pragma once
#include <math.h>

#include "KTRSolver.h"
#include "KTRProblemLSQ.h"

class ProblemLSQ : public knitro::KTRProblemLSQ {
	
public:
	ProblemLSQ() : KTRProblemLSQ(2, 6, 12) {
		setObjectiveProperties();
		setVariableProperties();
		setSparsityPattern();
	}

	int evaluateResidual(const std::vector<double>& x, std::vector<double>& residual) {
		residual[0] = x[0] * pow(1.309, x[1]) - 2.138;
		residual[1] = x[0] * pow(1.471, x[1]) - 3.421; 
		residual[2] = x[0] * pow(1.49, x[1]) - 3.597;
		residual[3] = x[0] * pow(1.565, x[1]) - 4.34;
		residual[4] = x[0] * pow(1.611, x[1]) - 4.882;
		residual[5] = x[0] * pow(1.68, x[1]) - 5.66;
		return( 0 );
	}

  	int evaluateJacobian(const std::vector<double>& x, std::vector<double>& jacobian) {
  		jacobian[0] = pow(1.309, x[1]);
	    jacobian[1] = x[0] * log(1.309) * pow(1.309, x[1]);
	    jacobian[2] = pow(1.471, x[1]);
	    jacobian[3] = x[0] * log(1.471) * pow(1.471, x[1]);
	    jacobian[4] = pow(1.49, x[1]);
	    jacobian[5] = x[0] * log(1.49) * pow(1.49, x[1]);
	    jacobian[6] = pow(1.565, x[1]);
	    jacobian[7] = x[0] * log(1.565) * pow(1.565, x[1]);
	    jacobian[8] = pow(1.611, x[1]);
	    jacobian[9] = x[0] * log(1.611) * pow(1.611, x[1]);
	    jacobian[10] = pow(1.68, x[1]);
	    jacobian[11] = x[0] * log(1.68) * pow(1.68, x[1]);
  		return( 0 );
  	}

private: 
  	void setObjectiveProperties() {
    	setObjType(knitro::KTREnums::ObjectiveType::ObjGeneral);
    	setObjGoal(knitro::KTREnums::ObjectiveGoal::Minimize);
  	}

  	void setVariableProperties() {
    	setVarLoBnds(0, -1.0 * KTR_INFBOUND);
    	setVarLoBnds(1, -1.0 * KTR_INFBOUND);

    	setVarUpBnds(0, KTR_INFBOUND);
    	setVarUpBnds(1, KTR_INFBOUND);

    	setXInitial(0, 1.0);
    	setXInitial(1, 5.0);
    }

    void setSparsityPattern() {
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

};
