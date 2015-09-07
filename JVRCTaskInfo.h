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

/*
enum JVRCTaskID {
    JVRC_TASK_O1,
    JVRC_TASK_O2,
    JVRC_TASK_R1,
    JVRC_TASK_R2,
    JVRC_TASK_R3,
    JVRC_TASK_R4,
    JVRC_TASK_R5,
    JVRC_TASK_R6
};

enum JVRCEventID {
    JVRC_EVENT_START,
    JVRC_EVENT_TARGET,
    JVRC_EVENT_GATE_2,
    JVRC_EVENT_GATE_3,
    JVRC_EVENT_OBSTACLE,
    JVRC_EVENT_DOOR,
    JVRC_EVENT_HOSE,
    JVRC_EVENT_NOZZLE,
    JVRC_EVENT_CONNECT,
    JVRC_EVENT_VALVE,
    JVRC_EVENT_GOAL
};
*/

class Listing;

class JVRCEvent;
typedef ref_ptr<JVRCEvent> JVRCEventPtr;

class JVRCEventRecord;
typedef ref_ptr<JVRCEventRecord> JVRCEventRecordPtr;


class JVRCTask : public Referenced
{
public:
    JVRCTask(const std::string& name);

    const std::string& name() const { return name_; }
    void addEvent(JVRCEventPtr event);
    int numEvents() const { return events.size(); }
    JVRCEventPtr event(int index) { return events[index]; }

private:
    std::string name_;
    std::vector<JVRCEventPtr> events;
};

typedef ref_ptr<JVRCTask> JVRCTaskPtr;
    
   
class JVRCEvent : public Referenced
{
public:
    JVRCEvent(JVRCTaskPtr task, const std::string& type);
    JVRCEvent(const JVRCEvent& event);

    virtual JVRCEvent* clone();

    JVRCTaskPtr task() { return task_.lock(); }
    const std::string& type() const { return type_; }
    void setLabel(const std::string& label);
    const std::string& label() const { return label_; }
    void setLevel(int level) { level_ = level; }
    int level() const { return level_; }
    //void setMaxNumEvents(int n);
    //int maxNumEvents() const { return maxNumEvents_; }

    double time() const { return time_; }
    void setTime(double t) { time_ = t; }

    //JVRCEventRecord* createRecord(double time);

private:
    weak_ref_ptr<JVRCTask> task_;
    std::string type_;
    std::string label_;
    int level_;
    //int maxNumEvents_;
    double time_;
};


class JVRCGateEvent : public JVRCEvent
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    JVRCGateEvent(JVRCTaskPtr task);
    JVRCGateEvent(const JVRCGateEvent& org);
    virtual JVRCEvent* clone();
    const Vector2& location(int which) const { return locations[which]; }
    int index() const { return index_; }
    void setIndex(int i) { index_ = i; }
    bool isGoal() const { return isGoal_; }
    void setGoal() { isGoal_ = true; }

private:
    Vector2 locations[2];
    int index_;
    bool isGoal_;
};


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
