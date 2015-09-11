// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-

#include <cmath>
#include <cstring>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <cnoid/MessageView>

#ifdef CNOID_BODY_CUSTOMIZER
#include <cnoid/BodyCustomizerInterface>
#else
#include <BodyCustomizerInterface.h>
#endif

#include <iostream>

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
#define DLL_EXPORT __declspec(dllexport)
#else 
#define DLL_EXPORT 
#endif /* Windows */

#if defined(HRPMODEL_VERSION_MAJOR) && defined(HRPMODEL_VERSION_MINOR)
#if HRPMODEL_VERSION_MAJOR >= 3 && HRPMODEL_VERSION_MINOR >= 1
#include <hrpUtil/Tvmet3dTypes.h>
#define NS_HRPMODEL hrp
#endif
#endif

#ifdef CNOID_BODY_CUSTOMIZER
#define NS_HRPMODEL cnoid
cnoid::Matrix3 trans(const cnoid::Matrix3& M) { return M.transpose(); }
double dot(const cnoid::Vector3& a, const cnoid::Vector3& b) { return a.dot(b); }
typedef cnoid::Matrix3 Matrix33;
#endif

#ifndef NS_HRPMODEL
#define NS_HRPMODEL OpenHRP
typedef OpenHRP::vector3 Vector3;
typedef OpenHRP::matrix33 Matrix33;
#endif

using namespace std;
using namespace boost;
using namespace NS_HRPMODEL;

static BodyInterface* bodyInterface = 0;

static BodyCustomizerInterface bodyCustomizerInterface;

struct JointValSet
{
    double* q_ptr;
    double* dq_ptr;
    double* u_ptr;
};

struct CabinetBoxCustomizer
{
    BodyHandle bodyHandle;
    JointValSet jointValSet[3];
};

static const char** getTargetModelNames()
{
    static const char* names[] = { 
        "box",
        0 };
	
    return names;
}

static BodyCustomizerHandle create(BodyHandle bodyHandle, const char* modelName)
{
    CabinetBoxCustomizer* customizer = 0;

    int jointIndices[3];
    jointIndices[0] = bodyInterface->getLinkIndexFromName(bodyHandle, "hinge_door");
	jointIndices[1] = bodyInterface->getLinkIndexFromName(bodyHandle, "hinge_doorknob");
	jointIndices[2] = bodyInterface->getLinkIndexFromName(bodyHandle, "hinge_valve");

    string name(modelName);

    MessageView* mv;
    mv = MessageView::instance();
    mv->putln("The cabinet box customizer is running");

    if (name == "box") {
        customizer = new CabinetBoxCustomizer;
        customizer->bodyHandle = bodyHandle;

		for (int i = 0; i < 3; i++) {
			
			int jointIndex = jointIndices[i];
			JointValSet& jointValSet = customizer->jointValSet[i];

			if (jointIndex >= 0) {
				jointValSet.q_ptr = bodyInterface->getJointValuePtr(bodyHandle, jointIndex);
				jointValSet.dq_ptr = bodyInterface->getJointVelocityPtr(bodyHandle, jointIndex);
				jointValSet.u_ptr = bodyInterface->getJointForcePtr(bodyHandle, jointIndex);
			}
			else {
				jointValSet.q_ptr = NULL;
				jointValSet.dq_ptr = NULL;
				jointValSet.u_ptr = NULL;
			}
		}
    }
    
    return static_cast<BodyCustomizerHandle>(customizer);
}

static void destroy(BodyCustomizerHandle customizerHandle)
{
	CabinetBoxCustomizer* customizer = static_cast<CabinetBoxCustomizer*>(customizerHandle);
    if(customizer){
        delete customizer;
    }
}

static void setVirtualJointForces(BodyCustomizerHandle customizerHandle)
{
    CabinetBoxCustomizer* customizer = static_cast<CabinetBoxCustomizer*>(customizerHandle);

	double q_ref, spring, damper, SpringTorque, DamperTorque;

    MessageView* mv;
    mv = MessageView::instance();

	for (int i = 0; i < 3; ++i) {

		JointValSet& theta = customizer->jointValSet[i];
		
		if (i == 0) {   // Door 

			if (*(theta.q_ptr) < 0.0) {
				q_ref = 0.0;
				spring = 50.0;
				damper = 30.0;
			}
			else if (*(theta.q_ptr) < 1.54) {
				q_ref = 0.0;
				spring = 0.5;
				damper = 1.5;
			}
			else {
				q_ref = 1.54;
				spring = 50.0;
				damper = 30.0;
			}
		}

		else if (i == 1) {   // Knob

			q_ref = 0.0;
			spring = 0.4;
			damper = 0.1;
		}

		else {   // Valve

			q_ref = 0.0;
			spring = 0.0;
			damper = 1.0;
		}

		SpringTorque = spring * (*(theta.q_ptr) - q_ref);
		DamperTorque = damper * (*(theta.dq_ptr));
		*(theta.u_ptr) = - SpringTorque - DamperTorque;
	}
}

extern "C" DLL_EXPORT
NS_HRPMODEL::BodyCustomizerInterface* getHrpBodyCustomizerInterface(NS_HRPMODEL::BodyInterface* bodyInterface_)
{
    bodyInterface = bodyInterface_;

    bodyCustomizerInterface.version = NS_HRPMODEL::BODY_CUSTOMIZER_INTERFACE_VERSION;
    bodyCustomizerInterface.getTargetModelNames = getTargetModelNames;
    bodyCustomizerInterface.create = create;
    bodyCustomizerInterface.destroy = destroy;
    bodyCustomizerInterface.initializeAnalyticIk = 0;
    bodyCustomizerInterface.calcAnalyticIk = 0;
    bodyCustomizerInterface.setVirtualJointForces = setVirtualJointForces;

    return &bodyCustomizerInterface;
}
