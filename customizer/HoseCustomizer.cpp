// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-

#include <cmath>
#include <cstring>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <cnoid/MessageView>

#include <cnoid/TimeBar> 

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
    double  e ;  /* ADDED BY KIKUUWE */ 
    double  p_prv ;  /* ADDED BY KIKUUWE */ 
};

struct HoseCustomizer
{
    BodyHandle bodyHandle;
    JointValSet jointValSet[53];
    double     SPTM;  /* ADDED BY KIKUUWE*/  
    int        first_time ;/* ADDED BY KIKUUWE*/ 
};

static const char** getTargetModelNames()
{
    static const char* names[] = { 
        "hose",
        0 };
	
    return names;
}

static BodyCustomizerHandle create(BodyHandle bodyHandle, const char* modelName)
{
    HoseCustomizer* customizer = 0;

    int jointIndices[53];
    jointIndices[ 0] = bodyInterface->getLinkIndexFromName(bodyHandle, "hinge_hose0");
	jointIndices[ 1] = bodyInterface->getLinkIndexFromName(bodyHandle, "hinge_1st2L11WD");
	jointIndices[ 2] = bodyInterface->getLinkIndexFromName(bodyHandle, "hinge_L11WD2L11ND");
    jointIndices[ 3] = bodyInterface->getLinkIndexFromName(bodyHandle, "hinge_L11ND2L12WD");
	jointIndices[ 4] = bodyInterface->getLinkIndexFromName(bodyHandle, "hinge_L12WD2L12ND");
	jointIndices[ 5] = bodyInterface->getLinkIndexFromName(bodyHandle, "hinge_L12ND2L13WD");
    jointIndices[ 6] = bodyInterface->getLinkIndexFromName(bodyHandle, "hinge_L13WD2L13ND");
	jointIndices[ 7] = bodyInterface->getLinkIndexFromName(bodyHandle, "hinge_L13ND2S1WD");
	jointIndices[ 8] = bodyInterface->getLinkIndexFromName(bodyHandle, "hinge_S1WD2S1ND");
    jointIndices[ 9] = bodyInterface->getLinkIndexFromName(bodyHandle, "hinge_S1ND2L21WD");
	jointIndices[10] = bodyInterface->getLinkIndexFromName(bodyHandle, "hinge_L21WD2L21ND");
	jointIndices[11] = bodyInterface->getLinkIndexFromName(bodyHandle, "hinge_L21ND2L22WD");
    jointIndices[12] = bodyInterface->getLinkIndexFromName(bodyHandle, "hinge_L22WD2L22ND");
	jointIndices[13] = bodyInterface->getLinkIndexFromName(bodyHandle, "hinge_L22ND2L23WD");
	jointIndices[14] = bodyInterface->getLinkIndexFromName(bodyHandle, "hinge_L23WD2L23ND");
    jointIndices[15] = bodyInterface->getLinkIndexFromName(bodyHandle, "hinge_L23ND2S2WD");
	jointIndices[16] = bodyInterface->getLinkIndexFromName(bodyHandle, "hinge_S2WD2S2ND");
	jointIndices[17] = bodyInterface->getLinkIndexFromName(bodyHandle, "hinge_S2ND2L31WD");
    jointIndices[18] = bodyInterface->getLinkIndexFromName(bodyHandle, "hinge_L31WD2L31ND");
	jointIndices[19] = bodyInterface->getLinkIndexFromName(bodyHandle, "hinge_L31ND2L32WD");
	jointIndices[20] = bodyInterface->getLinkIndexFromName(bodyHandle, "hinge_L32WD2L32ND");
    jointIndices[21] = bodyInterface->getLinkIndexFromName(bodyHandle, "hinge_L32ND2L33WD");
	jointIndices[22] = bodyInterface->getLinkIndexFromName(bodyHandle, "hinge_L33WD2L33ND");
	jointIndices[23] = bodyInterface->getLinkIndexFromName(bodyHandle, "hinge_L33ND2S3WD");
    jointIndices[24] = bodyInterface->getLinkIndexFromName(bodyHandle, "hinge_S3WD2S3ND");
	jointIndices[25] = bodyInterface->getLinkIndexFromName(bodyHandle, "hinge_S3ND2L41WD");
	jointIndices[26] = bodyInterface->getLinkIndexFromName(bodyHandle, "hinge_L41WD2L41ND");
    jointIndices[27] = bodyInterface->getLinkIndexFromName(bodyHandle, "hinge_L41ND2L42WD");
	jointIndices[28] = bodyInterface->getLinkIndexFromName(bodyHandle, "hinge_L42WD2L42ND");
	jointIndices[29] = bodyInterface->getLinkIndexFromName(bodyHandle, "hinge_L42ND2L43WD");
    jointIndices[30] = bodyInterface->getLinkIndexFromName(bodyHandle, "hinge_L43WD2L43ND");
	jointIndices[31] = bodyInterface->getLinkIndexFromName(bodyHandle, "hinge_L43ND2S4WD");
	jointIndices[32] = bodyInterface->getLinkIndexFromName(bodyHandle, "hinge_S4WD2S4ND");
    jointIndices[33] = bodyInterface->getLinkIndexFromName(bodyHandle, "hinge_S4ND2L51WD");
	jointIndices[34] = bodyInterface->getLinkIndexFromName(bodyHandle, "hinge_L51WD2L51ND");
	jointIndices[35] = bodyInterface->getLinkIndexFromName(bodyHandle, "hinge_L51ND2L52WD");
    jointIndices[36] = bodyInterface->getLinkIndexFromName(bodyHandle, "hinge_L52WD2L52ND");
	jointIndices[37] = bodyInterface->getLinkIndexFromName(bodyHandle, "hinge_L52ND2L53WD");
	jointIndices[38] = bodyInterface->getLinkIndexFromName(bodyHandle, "hinge_L53WD2L53ND");
	jointIndices[39] = bodyInterface->getLinkIndexFromName(bodyHandle, "hinge_L53ND2S5WD");
	jointIndices[40] = bodyInterface->getLinkIndexFromName(bodyHandle, "hinge_S5WD2S5ND");
	jointIndices[41] = bodyInterface->getLinkIndexFromName(bodyHandle, "hinge_S5ND2L61WD");
	jointIndices[42] = bodyInterface->getLinkIndexFromName(bodyHandle, "hinge_L61WD2L61ND");
	jointIndices[43] = bodyInterface->getLinkIndexFromName(bodyHandle, "hinge_L61ND2L62WD");
	jointIndices[44] = bodyInterface->getLinkIndexFromName(bodyHandle, "hinge_L62WD2L62ND");
	jointIndices[45] = bodyInterface->getLinkIndexFromName(bodyHandle, "hinge_L62ND2L63WD");
	jointIndices[46] = bodyInterface->getLinkIndexFromName(bodyHandle, "hinge_L63WD2L63ND");
	jointIndices[47] = bodyInterface->getLinkIndexFromName(bodyHandle, "hinge_L63ND2S6WD");
	jointIndices[48] = bodyInterface->getLinkIndexFromName(bodyHandle, "hinge_S6WD2S6ND");
	jointIndices[49] = bodyInterface->getLinkIndexFromName(bodyHandle, "hinge_S6ND2L71WD");
	jointIndices[50] = bodyInterface->getLinkIndexFromName(bodyHandle, "hinge_L71WD2L71ND");
	jointIndices[51] = bodyInterface->getLinkIndexFromName(bodyHandle, "hinge_L71ND2L72WD");
	jointIndices[52] = bodyInterface->getLinkIndexFromName(bodyHandle, "hinge_L72WD2L72ND");

    string name(modelName);

    MessageView* mv;
    mv = MessageView::instance();
    mv->putln("The hose customizer is running");

    if (name == "hose") {
        customizer = new HoseCustomizer;
        customizer->bodyHandle = bodyHandle;
		customizer->SPTM       = TimeBar::instance()->timeStep(); /* ADDED BY KIKUUWE*/  
		customizer->first_time = 1;      /* ADDED BY KIKUUWE*/

		for (int i = 0; i < 53; i++) {
			
			int jointIndex = jointIndices[i];
			JointValSet& jointValSet = customizer->jointValSet[i];

			if (jointIndex >= 0) {
				jointValSet.q_ptr = bodyInterface->getJointValuePtr(bodyHandle, jointIndex);
				jointValSet.dq_ptr = bodyInterface->getJointVelocityPtr(bodyHandle, jointIndex);
				jointValSet.u_ptr = bodyInterface->getJointForcePtr(bodyHandle, jointIndex);
				jointValSet.e      = 0; // added by KIKUUWE
				jointValSet.p_prv  = 0; // added by KIKUUWE
			}
			else {
				jointValSet.q_ptr = NULL;
				jointValSet.dq_ptr = NULL;
				jointValSet.u_ptr = NULL;
				jointValSet.e  = 0; // added by KIKUUWE
				jointValSet.p_prv  = 0; // added by KIKUUWE
			}
		}
    }
    
    return static_cast<BodyCustomizerHandle>(customizer);
}

static void destroy(BodyCustomizerHandle customizerHandle)
{
	HoseCustomizer* customizer = static_cast<HoseCustomizer*>(customizerHandle);
    if(customizer){
        delete customizer;
    }
}

static void setVirtualJointForces(BodyCustomizerHandle customizerHandle)
{
    HoseCustomizer* customizer = static_cast<HoseCustomizer*>(customizerHandle);

	double q_act, q_ref, spring, damper, SpringTorque, DamperTorque, torque, bounded_torque, maxTorque;

    MessageView* mv;
    mv = MessageView::instance();

	for (int i = 0; i < 53; ++i) {

		JointValSet& theta = customizer->jointValSet[i];
		
		if (*(theta.q_ptr) < -2.618) {
			q_ref = -2.618;  // -2.618
			spring = 0.05;   // 50.0
			damper = 0.0;    // 0.0
		}
		else if (*(theta.q_ptr) < 2.618) {
			q_ref = 0.0;    // 0.0
			spring = 0.0;   // 0.1
			damper = 0.001;   // 1.0
		}
		else {
			q_ref = 2.618;  // -2.618
			spring = 0.05;  // 50.0
			damper = 0.0;   // 0.0
		}
		
		double  FK = 0.01  ; // 0.01 // 100 /* presliding stiffness in Nm/rad ; This should be as high as possible. */ 
		double  FB = FK*0.5 ; // FK*0.5 /* presliding viscosity in Nms/rad; This should be around 0.05 * FK or lower. */
		double  FF = 10.0 ; // 10 // 0.01 /* friction torque in Nm; This is the static & kinetic friction force.  */ 
		double  T  = customizer->SPTM  ;
		double  v  = (customizer->first_time)?(0):(((*(theta.q_ptr))-theta.p_prv )/T);
	//	double  v  = (*(theta.dq_ptr)) ; 
		double& e  = theta.e   ; 
		double  va = v + (FK*e)/(FK*T+FB);
		double  fa = (FK*T+FB)*va ;
		double  ff = (fa>FF)?FF:((fa>-FF)?fa:(-FF));
		e = (FB*e + T*ff)/(FK*T+FB) ;  
		theta.p_prv =  (*(theta.q_ptr));
		
		SpringTorque = spring * (*(theta.q_ptr) - q_ref);
		DamperTorque = damper * (*(theta.dq_ptr));
		torque = - SpringTorque - DamperTorque;
		
		*(theta.u_ptr) = torque - ff;
	}
	customizer->first_time = 0; // ADDED BY KIKUUWE
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
