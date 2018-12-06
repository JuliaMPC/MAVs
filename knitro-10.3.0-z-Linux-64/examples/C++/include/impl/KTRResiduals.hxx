#pragma once

#include "KTRResiduals.h"
#include "KTRException.h"

namespace knitro {
	
KTRResiduals::KTRResiduals(int n)
	: KTRElements(n), _types(n){
}

KTRResiduals::KTRResiduals(KTRTypes const & types) 
	: KTRElements((int)types.getTypes().size()), _types(types){
}

void KTRResiduals::setTypes(std::vector<int> const & types){
	if (size() != (int)types.size()){
		std::ostringstream msg;
    	msg << "Error: input vector types has length " << types.size()
    		<< " which is different from residuals size." << std::endl;
    	throw KTRException(msg.str(), "KTRResiduals::setTypes", __LINE__);
	}
	_types.setTypes(types);
}

}


