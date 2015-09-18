// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-

#include <cmath>
#include <cstring>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <cnoid/MessageView>
#include <cnoid/EigenUtil>
/** ADDED BY KIKUUWE from here **/
#include <cnoid/TimeBar> 
/** ADDED BY KIKUUWE down to here **/

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

struct CabinetBoxCustomizer
{
    BodyHandle bodyHandle;
    JointValSet jointValSet[3];
    double     SPTM;  /* ADDED BY KIKUUWE*/  
    int        first_time ;/* ADDED BY KIKUUWE*/  
};

static const char** getTargetModelNames()
{
    static const char* names[] = { 
        "CABINETBOX",
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

    if (name == "CABINETBOX") {
        customizer = new CabinetBoxCustomizer;
        customizer->bodyHandle = bodyHandle;
        customizer->SPTM       = TimeBar::instance()->timeStep(); /* ADDED BY KIKUUWE*/
        customizer->first_time = 1;      /* ADDED BY KIKUUWE*/

        for (int i = 0; i < 3; i++) {

            int jointIndex = jointIndices[i];
            JointValSet& jointValSet = customizer->jointValSet[i];

            if (jointIndex >= 0) {
                jointValSet.q_ptr = bodyInterface->getJointValuePtr(bodyHandle, jointIndex);
                jointValSet.dq_ptr = bodyInterface->getJointVelocityPtr(bodyHandle, jointIndex);
                jointValSet.u_ptr = bodyInterface->getJointForcePtr(bodyHandle, jointIndex);
                jointValSet.e  = 0; // added by KIKUUWE
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
    CabinetBoxCustomizer* customizer = static_cast<CabinetBoxCustomizer*>(customizerHandle);
    if(customizer){
        delete customizer;
    }
}

static void setVirtualJointForces(BodyCustomizerHandle customizerHandle)
{
    CabinetBoxCustomizer* customizer = static_cast<CabinetBoxCustomizer*>(customizerHandle);

    double q_ref, spring, damper, SpringTorque, DamperTorque;
    double  FK, FB, FF ; // added by KIKUUWE
    MessageView* mv;
    mv = MessageView::instance();

    JointValSet& hingeDoor = customizer->jointValSet[0];
    JointValSet& doornob = customizer->jointValSet[1];

    bool latch = false;
    if( radian(-60.0) < *(doornob.q_ptr) &&  *(doornob.q_ptr) < radian(60.0))
        latch = true;

    if( (0.0 < *(hingeDoor.q_ptr) && *(hingeDoor.q_ptr) < radian(5.0) && !latch) ||
            (radian(5.0) <= *(hingeDoor.q_ptr) && *(hingeDoor.q_ptr) < 1.54) ){
        *(hingeDoor.u_ptr) = -1 * (*(hingeDoor.dq_ptr));
    }else{
        if(*(hingeDoor.q_ptr) >= 1.54)
            q_ref = 1.54;
        else
            q_ref = 0.0;
        SpringTorque = 5000 * (*(hingeDoor.q_ptr) - q_ref);
        DamperTorque = 100 * (*(hingeDoor.dq_ptr));
        *(hingeDoor.u_ptr) = - SpringTorque - DamperTorque;
    }

    for (int i = 1; i < 3; ++i) {

        JointValSet& theta = customizer->jointValSet[i];

        if (i == 1) {   // Knob
            FK = 18;//15; /* added by KIKUUWE, presliding stiffness in Nm/rad */
            FB = FK * 0.05 ; /* added by KIKUUWE, presliding viscosity in Nms/rad */
            FF =  2.5; /* added by KIKUUWE, friction torque in Nm */
        } else {   // Valve
            FK = 100.  ; /* added by KIKUUWE, presliding stiffness in Nm/rad */
            FB = 0.1;//FK * 0.005;//FK * 0.05 ; /* added by KIKUUWE, presliding viscosity in Nms/rad */
            FF = 5.0 ; /* added by KIKUUWE, friction torque in Nm */
        }

        //  added by KIKUUWE
            double  T  = customizer->SPTM  ;
            double  v  = (customizer->first_time)?(0):(((*(theta.q_ptr))-theta.p_prv )/T);
        //    double  v  = (*(theta.dq_ptr)) ;
            double& e  = theta.e   ;
            double  va = v + (FK*e)/(FK*T+FB);
            double  fa = (FK*T+FB)*va ;
            double  ff = (fa>FF)?FF:((fa>-FF)?fa:(-FF));
            e = (FB*e + T*ff)/(FK*T+FB) ;
           // ff = FK * *(theta.q_ptr) + FB * v ;
            *(theta.u_ptr) = -ff ;
            theta.p_prv =  (*(theta.q_ptr));
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
