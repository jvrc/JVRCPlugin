/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_JVRC_PLUGIN_SPHERE_MARKER_DEVICE_H
#define CNOID_JVRC_PLUGIN_SPHERE_MARKER_DEVICE_H

#include <cnoid/Device>

namespace cnoid {

class SphereMarkerDevice : public ActiveDevice
{
public:
    SphereMarkerDevice();
    SphereMarkerDevice(const SphereMarkerDevice& org, bool copyState = true);
    void copyStateFrom(const SphereMarkerDevice& other);
    virtual void copyStateFrom(const DeviceState& other);
    virtual DeviceState* cloneState() const;
    virtual Device* clone() const;
    virtual void forEachActualType(boost::function<bool(const std::type_info& type)> func);
    virtual int stateSize() const;
    virtual const double* readState(const double* buf);
    virtual double* writeState(double* out_buf) const;

    bool on() const { return on_; }
    void on(bool on) { on_ = on; }
    const Vector3f& color() const { return color_; }
    void setColor(const Vector3f& c) { color_ = c; }
    double size() const { return size_; }
    void setSize(double s) { size_ = s; }
        
private:
    Vector3f color_;
    double size_;
    bool on_;
};

typedef ref_ptr<SphereMarkerDevice> SphereMarkerDevicePtr;

}

#endif
