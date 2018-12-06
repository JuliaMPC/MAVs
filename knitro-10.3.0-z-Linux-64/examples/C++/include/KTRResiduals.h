#pragma once

#include "KTRElements.h"

namespace knitro {
	
class KTRResiduals : public KTRElements {
	
public:
	explicit KTRResiduals(int n);
	explicit KTRResiduals(KTRTypes const &);
	virtual ~KTRResiduals() {}

	KTRTypes const & getKTRTypes() const { return(_types); }
	void setTypes(std::vector<int> const & types); 

private:
	/**-- Types of residuals (general or linear) */
	KTRTypes _types;
};

}

#include "impl/KTRResiduals.hxx"

