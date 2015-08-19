/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "SphereMarkerDevice.h"
#include <cnoid/SceneDevice>
#include <cnoid/SceneMarker>
#include <boost/bind.hpp>

using namespace std;
using namespace cnoid;

namespace {

struct SceneSphereMarkerDevice : public SceneDevice
{
    SphereMarkerDevice* device;
    SphereMarkerPtr marker;
    bool isShown;
public:
    SceneSphereMarkerDevice(Device* device_)
        : SceneDevice(device_),
          device(static_cast<SphereMarkerDevice*>(device_)){
        marker = new SphereMarker(device->radius(), device->color(), device->transparency());
        isShown = false;
        setSceneUpdateFunction(boost::bind(&SceneSphereMarkerDevice::updateScene, this));
    }
    void updateScene(){
        if(device->on() != isShown){
            if(device->on()){
                addChildOnce(marker);
            } else {
                removeChild(marker);
            }
            isShown = device->on();
        }
        marker->setRadius(device->radius());
        marker->setColor(device->color());
    }
};
        
SceneDevice* createSceneSphereMarkerDevice(Device* device)
{
    return new SceneSphereMarkerDevice(device);
}

struct SceneDeviceFactoryRegistration
{
    SceneDeviceFactoryRegistration(){
        SceneDevice::registerSceneDeviceFactory<SphereMarkerDevice>(createSceneSphereMarkerDevice);
    }
};
SceneDeviceFactoryRegistration sceneDeviceFactoryRegistration;

}
    

SphereMarkerDevice::SphereMarkerDevice()
{
    on_ = true;
    radius_ = 0.1;
    color_ << 1.0f, 1.0f, 0.0f;
    transparency_ = 0.5f;
}


void SphereMarkerDevice::copyStateFrom(const SphereMarkerDevice& other)
{
    on_ = other.on_;
    radius_ = other.radius_;
    color_ = other.color_;
    transparency_ = other.transparency_;
}


void SphereMarkerDevice::copyStateFrom(const DeviceState& other)
{
    if(typeid(other) != typeid(SphereMarkerDevice)){
        throw std::invalid_argument("Type mismatch in the Device::copyStateFrom function");
    }
    copyStateFrom(static_cast<const SphereMarkerDevice&>(other));
}



SphereMarkerDevice::SphereMarkerDevice(const SphereMarkerDevice& org, bool copyAll)
    : ActiveDevice(org, copyAll)
{
    copyStateFrom(org);
}


DeviceState* SphereMarkerDevice::cloneState() const
{
    return new SphereMarkerDevice(*this, false);
}


Device* SphereMarkerDevice::clone() const
{
    return new SphereMarkerDevice(*this);
}

void SphereMarkerDevice::forEachActualType(boost::function<bool(const std::type_info& type)> func)
{
    if(!func(typeid(SphereMarkerDevice))){
        ActiveDevice::forEachActualType(func);
    }
}


int SphereMarkerDevice::stateSize() const
{
    return 5;
}


const double* SphereMarkerDevice::readState(const double* buf)
{
    on_ = buf[0];
    color_ = Eigen::Map<const Vector3>(buf + 1).cast<float>();
    radius_ = buf[4];
    return buf + 5;
}


double* SphereMarkerDevice::writeState(double* out_buf) const
{
    out_buf[0] = on_ ? 1.0 : 0.0;
    out_buf[1] = color_[0];
    out_buf[2] = color_[1];
    out_buf[3] = color_[2];
    out_buf[4] = radius_;
    return out_buf + 5;
}
