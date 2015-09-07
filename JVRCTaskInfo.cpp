/**
   @author Shin'ichiro Nakaoka
*/

#include "JVRCTaskInfo.h"
#include <cnoid/YAMLReader>

using namespace std;
using namespace cnoid;


JVRCTask::JVRCTask(const std::string& name)
    : name_(name)
{

}


void JVRCTask::addEvent(JVRCEventPtr event)
{
    events.push_back(event);
}


JVRCEvent::JVRCEvent(JVRCTaskPtr task, const std::string& type)
    : task_(task),
      type_(type),
      label_(type)
{
    level_ = 0;
    //maxNumEvents_ = 1;
    time_ = 0.0;
}


JVRCEvent::JVRCEvent(const JVRCEvent& org)
    : task_(org.task_),
      type_(org.type_),
      label_(org.label_),
      level_(org.level_)
      //maxNumEvents_(org.maxNumEvents_)
{

}


void JVRCEvent::setLabel(const std::string& label)
{
    label_ = label;
}


/*
JVRCEventRecord* JVRCEvent::createRecord(double time)
{
    return new JVRCEventRecord(this, time);
}
*/


JVRCEvent* JVRCEvent::clone()
{
    return new JVRCEvent(*this);
}


/*
void JVRCEvent::setMaxNumEvents(int n)
{
    maxNumEvents_ = n;
}
*/


JVRCGateEvent::JVRCGateEvent(JVRCTaskPtr task)
    : JVRCEvent(task, "gate")
{
    locations[0].setZero();
    locations[1].setZero();
    index_ = 0;
    isGoal_ = false;
}
    

JVRCGateEvent::JVRCGateEvent(const JVRCGateEvent& org)
    : JVRCEvent(org)
{
    locations[0] = org.locations[0];
    locations[1] = org.locations[1];
    index_ = org.index_;
    isGoal_ = org.isGoal_;
}


JVRCEvent* JVRCGateEvent::clone()
{
    return new JVRCGateEvent(*this);
}


/*
JVRCEventRecord::JVRCEventRecord(JVRCEvent* event, double time)
    : JVRCEvent(*event)
{
    time_ = time;
}
*/


bool JVRCTaskInfo::load(const std::string& filename)
{
    YAMLReader reader;
    MappingPtr info = reader.loadDocument(filename)->toMapping();
    const Listing& taskNodes = *info->findListing("tasks");
    if(taskNodes.isValid()){
        readTasks(taskNodes);
    }
    return true;
}


void JVRCTaskInfo::readTasks(const Listing& taskNodes)
{
    tasks.clear();
    
    for(int i=0; i < taskNodes.size(); ++i){
        const Mapping& taskNode = *taskNodes[i].toMapping();
        JVRCTaskPtr task = new JVRCTask(taskNode.read<string>("name"));
        const Listing& eventNodes = *taskNode.findListing("events");
        if(eventNodes.isValid()){
            int gateIndex = 0;
            JVRCGateEvent* gate = 0;
            for(int j=0; j < eventNodes.size(); ++j){
                const Mapping& eventNode = *eventNodes[j].toMapping();
                string type = eventNode.read<string>("type");
                JVRCEventPtr event;
                if(type == "gate"){
                    gate = new JVRCGateEvent(task);
                    gate->setIndex(gateIndex);
                    event = gate;
                } else {
                    event = new JVRCEvent(task, type);
                }
                string label;
                if(eventNode.read("label", label)){
                    event->setLabel(label);
                }
                int level;
                if(eventNode.read("level", level)){
                    event->setLevel(level);
                }
                //event->setMaxNumEvents(eventNode.get("maxCount", 1));
                task->addEvent(event);
            }
            if(gate){
                gate->setGoal();
            }
        }
        tasks.push_back(task);
    }
}


    
