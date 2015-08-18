/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "SphereMarkerDevice.h"

using namespace cnoid;


SphereMarkerDevice::SphereMarkerDevice()
{
    on_ = false;
    color_ << 1.0f, 1.0f, 0.0f;
    size_ = 0.1;
}


void SphereMarkerDevice::copyStateFrom(const SphereMarkerDevice& other)
{
    on_ = other.on_;
    color_ = other.color_;
    size_ = other.size_;
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
    size_ = buf[4];
    return buf + 5;
}


double* SphereMarkerDevice::writeState(double* out_buf) const
{
    out_buf[0] = on_ ? 1.0 : 0.0;
    out_buf[1] = color_[0];
    out_buf[2] = color_[1];
    out_buf[3] = color_[2];
    out_buf[4] = size_;
    return out_buf + 5;
}
