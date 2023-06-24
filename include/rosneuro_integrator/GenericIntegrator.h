#ifndef ROSNEURO_INTEGRATOR_GENERICINTEGRATOR_H_
#define ROSNEURO_INTEGRATOR_GENERICINTEGRATOR_H_

#include <Eigen/Dense>
#include <ros/ros.h>

namespace rosneuro {
	namespace integrator {

class GenericIntegrator {

	public:
		GenericIntegrator(void);
		virtual ~GenericIntegrator(void);

		virtual bool configure(void) = 0;
		virtual Eigen::VectorXf apply(const Eigen::VectorXf& in) = 0;
		virtual bool reset(void) = 0;

		std::string name(void);
		void setname(const std::string& name);

	private:
		std::string name_;

};

	}

}


#endif
