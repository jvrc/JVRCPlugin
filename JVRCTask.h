/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_JVRC_PLUGIN_JVRC_TASK_INFO_H
#define CNOID_JVRC_PLUGIN_JVRC_TASK_INFO_H

#include <cnoid/Referenced>
#include <cnoid/EigenTypes>
#include <cnoid/ValueTree>
#include <boost/optional.hpp>
#include <string>
#include <vector>

namespace cnoid {

class Mapping;
class Listing;
class JVRCTask;

class JVRCEvent : public Referenced
{
public:
    JVRCEvent(const std::string& type, JVRCTask* task);
    JVRCEvent(const std::string& type, JVRCTask* task, Mapping* info);
    JVRCEvent(const JVRCEvent& org);

    virtual JVRCEvent* clone();

    virtual bool isSameAs(JVRCEvent* event);

    JVRCTask* task() { return task_.lock(); }
    const JVRCTask* task() const { return task_.lock(); }
    const std::string& type() const { return type_; }
    void setLabel(const std::string& label);
    const std::string& label() const { return label_; }
    int point() const { return point_; }
    void setPoint(int p) { point_ = p; }
    void setLevel(int level) { level_ = level; }
    int level() const { return level_; }

    boost::optional<double> automaticRecordTime() const { return automaticRecordTime_; }
    void setAutomaticRecordTime(double t) { automaticRecordTime_ = t; }
    boost::optional<double> manualRecordTime() const { return manualRecordTime_; }
    void setManualRecordTime(double t) { manualRecordTime_ = t; }
    void clearManualRecordTime() { manualRecordTime_ = boost::none; }
    bool isTimeRecorded() const { return manualRecordTime_ || automaticRecordTime_; }
    
    double time() const;

    virtual void write(YAMLWriter& writer);

private:
    std::string type_;
    std::string label_;
    int point_;
    int level_;
    boost::optional<double> automaticRecordTime_;
    boost::optional<double> manualRecordTime_;
    weak_ref_ptr<JVRCTask> task_;
};

typedef ref_ptr<JVRCEvent> JVRCEventPtr;


class JVRCGateEvent : public JVRCEvent
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    JVRCGateEvent(JVRCTask* task);
    JVRCGateEvent(JVRCTask* task, Mapping* info);
    JVRCGateEvent(const JVRCGateEvent& org);
    virtual JVRCEvent* clone();
    virtual bool isSameAs(JVRCEvent* event);
    const Vector3& location(int which) const { return locations[which]; }
    void setLocation(int which, const Vector3& p) { locations[which] = p; }
    int index() const { return index_; }
    void setIndex(int i);
    bool isGoal() const;

    virtual void write(YAMLWriter& writer);

private:
    int index_;
    bool isLabelSpecified;
    Vector3 locations[2];

    void initialize();
};

typedef ref_ptr<JVRCGateEvent> JVRCGateEventPtr;


class JVRCTask : public Referenced
{
public:
    JVRCTask(Mapping* info);

    const std::string& name() const { return name_; }
    void addEvent(JVRCEvent* event);
    int numEvents() const { return events.size(); }
    JVRCEvent* event(int index) { return events[index]; }

    int numGates() const { return gates.size(); }
    JVRCGateEvent* gate(int index) { return gates[index]; }

    double timeLimit() const { return timeLimit_; }

    const Mapping* info() const { return info_; }

private:
    std::string name_;
    std::vector<JVRCEventPtr> events;
    std::vector<JVRCGateEventPtr> gates;
    double timeLimit_;
    MappingPtr info_;
};

typedef ref_ptr<JVRCTask> JVRCTaskPtr;
    
}

#endif
