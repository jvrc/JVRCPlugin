/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "SphereMarkerDevice.h"
#include <cnoid/SceneDevice>
#include <cnoid/SceneMarker>
#include <boost/bind.hpp>

using namespace cnoid;

namespace {

void updateSphereMarker(SphereMarkerDevice* device, SphereMarker* marker)
{
    marker->setRadius(device->radius());
    marker->setColor(device->color());
}

SgNode* createSphereMarkerDevice(SceneDevice* sdev)
{
    SphereMarkerDevice* device = sdev->device<SphereMarkerDevice>();
    SphereMarker* marker = new SphereMarker;
    sdev->setSceneUpdateFunction(boost::bind(updateSphereMarker, device, marker));
    return marker;
}

struct DeviceNodeFactoryRegistration
{
    DeviceNodeFactoryRegistration(){
        SceneDevice::registerDeviceNodeFactory<SphereMarkerDevice>(createSphereMarkerDevice);
    }
};
DeviceNodeFactoryRegistration deviceNodeFactoryRegistration;

}
    

SphereMarkerDevice::SphereMarkerDevice()
{
    on_ = false;
    color_ << 1.0f, 1.0f, 0.0f;
    radius_ = 0.1;
}


void SphereMarkerDevice::copyStateFrom(const SphereMarkerDevice& other)
{
    on_ = other.on_;
    color_ = other.color_;
    radius_ = other.radius_;
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
