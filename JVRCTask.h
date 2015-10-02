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
    const std::string& subTaskLabel() const { return subTaskLabel_; }
    int point() const { return point_; }
    void setPoint(int p) { point_ = p; }
    void setLevel(int level) { level_ = level; }
    int level() const { return level_; }

    
    boost::optional<double> detectedTime() const { return detectedTime_; }
    void setDetectedTime(double t) { detectedTime_ = t; }
    boost::optional<double> judgedTime() const { return judgedTime_; }
    void setJudgedTime(double t) { judgedTime_ = t; }
    void clearTimes();
    void clearManualRecordTime();
    bool isTimeRecorded() const { return judgedTime_ || detectedTime_; }
    
    double time() const;

    virtual void write(YAMLWriter& writer);

private:
    std::string type_;
    std::string label_;
    std::string subTaskLabel_;
    int point_;
    int level_;
    boost::optional<double> detectedTime_;
    boost::optional<double> judgedTime_;
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
    int gateIndex() const { return gateIndex_; }
    void setGateIndex(int i);
    bool isGoal() const;

    virtual void write(YAMLWriter& writer);

private:
    int gateIndex_;
    bool isLabelSpecified;
    Vector3 locations[2];

    void initialize();
};

typedef ref_ptr<JVRCGateEvent> JVRCGateEventPtr;


class JVRCActionEvent : public JVRCEvent
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    JVRCActionEvent(JVRCTask* task);
    JVRCActionEvent(JVRCTask* task, Mapping* info);
    JVRCActionEvent(const JVRCActionEvent& org);
    virtual JVRCEvent* clone();
    virtual bool isSameAs(JVRCEvent* event);

    int actionIndex() const { return actionIndex_; }
    void setActionIndex(int i);

    virtual void write(YAMLWriter& writer);

private:
    int actionIndex_;

    void initialize();
};

typedef ref_ptr<JVRCActionEvent> JVRCActionEventPtr;


class JVRCTask : public Referenced
{
public:
    JVRCTask(Mapping* info);

    const std::string& name() const { return name_; }
    const std::string& label() const { return label_; }
    void addEvent(JVRCEvent* event);
    int numEvents() const { return events.size(); }
    JVRCEvent* event(int index) { return events[index]; }

    int numGates() const { return gates.size(); }
    JVRCGateEvent* gate(int index) { return gates[index]; }

    int numActions() const { return actions.size(); }
    JVRCEvent* action(int index) { return actions[index]; }

    double timeLimit() const { return timeLimit_; }

    const Mapping* info() const { return info_; }

private:
    std::string name_;
    std::string label_;
    std::vector<JVRCEventPtr> events;
    std::vector<JVRCGateEventPtr> gates;
    std::vector<JVRCEventPtr> actions;
    double timeLimit_;
    MappingPtr info_;
};

typedef ref_ptr<JVRCTask> JVRCTaskPtr;
    
}

#endif
