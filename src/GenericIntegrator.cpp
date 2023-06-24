#include "rosneuro_integrator/GenericIntegrator.h"

namespace rosneuro {
	namespace integrator {

GenericIntegrator::GenericIntegrator(void) {
	this->name_ = "undefined";
}

GenericIntegrator::~GenericIntegrator(void) {}

std::string GenericIntegrator::name(void) {
	return this->name_;
}

void GenericIntegrator::setname(const std::string& name) {
	this->name_ = name;
}

	}

}
