/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_JVRC_PLUGIN_JVRC_TASK_INFO_H
#define CNOID_JVRC_PLUGIN_JVRC_TASK_INFO_H

#include <cnoid/Referenced>
#include <cnoid/EigenTypes>
#include <string>
#include <vector>

namespace cnoid {

class Mapping;
class Listing;
class JVRCTask;


class JVRCEvent : public Referenced
{
public:
    JVRCEvent(const std::string& type, JVRCTask* task, const Mapping& eventNode);
    JVRCEvent(const JVRCEvent& org);

    virtual JVRCEvent* clone();

    JVRCTask* task() { return task_.lock(); }
    const JVRCTask* task() const { return task_.lock(); }
    const std::string& type() const { return type_; }
    void setLabel(const std::string& label);
    const std::string& label() const { return label_; }
    void setLevel(int level) { level_ = level; }
    int level() const { return level_; }

    double time() const { return time_; }
    void setTime(double t) { time_ = t; }

    //JVRCEventRecord* createRecord(double time);

private:
    std::string type_;
    std::string label_;
    int level_;
    double time_;
    weak_ref_ptr<JVRCTask> task_;
};

typedef ref_ptr<JVRCEvent> JVRCEventPtr;


class JVRCGateEvent : public JVRCEvent
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    JVRCGateEvent(JVRCTask* task, const Mapping& eventNode);
    JVRCGateEvent(const JVRCGateEvent& org);
    virtual JVRCEvent* clone();
    const Vector2& location(int which) const { return locations[which]; }
    void setLocation(int which, const Vector2& p) { locations[which] = p; }
    int index() const { return index_; }
    void setIndex(int i) { index_ = i; }
    bool isGoal() const;

private:
    int index_;
    Vector2 locations[2];
};

typedef ref_ptr<JVRCGateEvent> JVRCGateEventPtr;


/*
class JVRCEventRecord : public JVRCEvent
{
public:
    double time() const { return time_; }

private:
    JVRCEventRecord(JVRCEvent* event, double time);

    double time_;

    friend class JVRCEvent;
};
*/


class JVRCTask : public Referenced
{
public:
    JVRCTask(const std::string& name);

    const std::string& name() const { return name_; }
    void addEvent(JVRCEvent* event);
    int numEvents() const { return events.size(); }
    JVRCEvent* event(int index) { return events[index]; }

    int numGates() const { return gates.size(); }
    JVRCGateEvent* gate(int index) { return gates[index]; }

private:
    std::string name_;
    std::vector<JVRCEventPtr> events;
    std::vector<JVRCGateEventPtr> gates;
};

typedef ref_ptr<JVRCTask> JVRCTaskPtr;
    
   
class JVRCTaskInfo : public Referenced
{
public:
    bool load(const std::string& filename);

    int numTasks() const { return tasks.size(); }
    JVRCTaskPtr task(int index) { return tasks[index]; }

private:
    std::vector<JVRCTaskPtr> tasks;

    void readTasks(const Listing& taskNodes);
};

typedef ref_ptr<JVRCTaskInfo> JVRCTaskInfoPtr;

}

#endif
