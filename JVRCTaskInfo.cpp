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
    maxNumEvents_ = 1;
}


JVRCEvent::JVRCEvent(const JVRCEvent& org)
    : task_(org.task_),
      type_(org.type_),
      label_(org.label_),
      level_(org.level_),
      maxNumEvents_(org.maxNumEvents_)
{

}


void JVRCEvent::setLabel(const std::string& label)
{
    label_ = label;
}


JVRCEventRecordPtr JVRCEvent::createRecord(double time)
{
    return new JVRCEventRecord(this, time);
}


void JVRCEvent::setMaxNumEvents(int n)
{
    maxNumEvents_ = n;
}


JVRCEventRecord::JVRCEventRecord(JVRCEvent* event, double time)
    : JVRCEvent(*event)
{
    recordTime_ = time;
    eventTime_ = time;
}


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
            for(int j=0; j < eventNodes.size(); ++j){
                const Mapping& eventNode = *eventNodes[j].toMapping();
                JVRCEventPtr event = new JVRCEvent(task, eventNode.read<string>("type"));
                string label;
                if(eventNode.read("label", label)){
                    event->setLabel(label);
                }
                int level;
                if(eventNode.read("level", level)){
                    event->setLevel(level);
                }
                event->setMaxNumEvents(eventNode.get("maxCount", 1));
                task->addEvent(event);
            }
        }
        tasks.push_back(task);
    }
}


    
