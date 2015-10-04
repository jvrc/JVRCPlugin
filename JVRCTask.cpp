/**
   @author Shin'ichiro Nakaoka
*/

#include "JVRCTask.h"
#include <cnoid/YAMLReader>
#include <cnoid/YAMLWriter>
#include <boost/format.hpp>

using namespace std;
using namespace cnoid;


JVRCEvent::JVRCEvent(const std::string& type, JVRCTask* task)
    : type_(type),
      label_(type),
      task_(task)
{
    point_ = 0;
    level_ = 0;
}


JVRCEvent::JVRCEvent(const std::string& type, JVRCTask* task, Mapping* info)
    : type_(type),
      label_(type),
      task_(task)
{
    info->read("label", label_);
    subTaskLabel_ = info->get("subTaskLabel",label_);
    point_ = info->get("point", 0);
    level_ = info->get("level", 0);
}


JVRCEvent::JVRCEvent(const JVRCEvent& org)
    : type_(org.type_),
      label_(org.label_),
      subTaskLabel_(org.subTaskLabel_),
      point_(org.point_),
      level_(org.level_),
      detectedTime_(org.detectedTime_),
      judgedTime_(org.judgedTime_),
      task_(org.task_)
{

}


void JVRCEvent::setLabel(const std::string& label)
{
    label_ = label;
}


JVRCEvent* JVRCEvent::clone()
{
    return new JVRCEvent(*this);
}


bool JVRCEvent::isSameAs(JVRCEvent* event)
{
    return (type_ == event->type_ && label_ == event->label_ && task_ == event->task_);
}


void JVRCEvent::clearTimes()
{
    detectedTime_ = boost::none;
    judgedTime_ = boost::none;
}
    

void JVRCEvent::clearManualRecordTime()
{
    judgedTime_ = boost::none;
}


double JVRCEvent::time() const
{
    if(judgedTime_){
        return *judgedTime_;
    }
    if(detectedTime_){
        return *detectedTime_;
    }
    return 0.0;
}


void JVRCEvent::write(YAMLWriter& writer)
{
    writer.putKeyValue("type", type_);
    writer.putKeyValue("label", label_);
    JVRCTaskPtr task = task_.lock();
    if(task){
        writer.putKeyValue("task", task->name());
    }
    if(detectedTime_){
        writer.putKeyValue("detectedTime", *detectedTime_);
    }
    if(judgedTime_){
        writer.putKeyValue("judgedTime", *judgedTime_);
    }
}


JVRCGateEvent::JVRCGateEvent(JVRCTask* task)
    : JVRCEvent("gate", task)
{
    initialize();
}


JVRCGateEvent::JVRCGateEvent(JVRCTask* task, Mapping* info)
    : JVRCEvent("gate", task, info)
{
    initialize();

    isLabelSpecified = info->find("label")->isValid();

    const Listing& location = *info->findListing("location");
    if(location.isValid()){
        if(location.size() == 2){
            for(int i=0; i < 2; ++i){
                const Listing& p = *location[i].toListing();
                int n = std::min(3, p.size());
                for(int j=0; j < n; ++j){
                    locations[i][j] = p[j].toDouble();
                }
            }
        }
    }
}


void JVRCGateEvent::initialize()
{
    gateIndex_ = 0;
    isLabelSpecified = true;
    locations[0].setZero();
    locations[1].setZero();
}


JVRCGateEvent::JVRCGateEvent(const JVRCGateEvent& org)
    : JVRCEvent(org)
{
    gateIndex_ = org.gateIndex_;
    locations[0] = org.locations[0];
    locations[1] = org.locations[1];
}


JVRCEvent* JVRCGateEvent::clone()
{
    return new JVRCGateEvent(*this);
}


bool JVRCGateEvent::isSameAs(JVRCEvent* event)
{
    JVRCGateEvent* gate = dynamic_cast<JVRCGateEvent*>(event);
    if(gate){
        if(gateIndex_ == gate->gateIndex_){
            return JVRCEvent::isSameAs(event);
        }
    }
    return false;
}


void JVRCGateEvent::setGateIndex(int i)
{
    gateIndex_ = i;

    if(!isLabelSpecified){
        setLabel(str(boost::format("Gate %1%") % (i + 1)));
    }
}


bool JVRCGateEvent::isGoal() const
{
    const JVRCTask* t = task();
    if(t){
        return gateIndex() == (t->numGates() - 1);
    }
    return false;
}


void JVRCGateEvent::write(YAMLWriter& writer)
{
    JVRCEvent::write(writer);
    writer.putKeyValue("gateIndex", gateIndex_);
}









JVRCActionEvent::JVRCActionEvent(JVRCTask* task)
    : JVRCEvent("action", task)
{
    actionIndex_ = 0;
}


JVRCActionEvent::JVRCActionEvent(JVRCTask* task, Mapping* info)
    : JVRCEvent("action", task, info)
{
    actionIndex_ = 0;
}


JVRCActionEvent::JVRCActionEvent(const JVRCActionEvent& org)
    : JVRCEvent(org)
{
    actionIndex_ = org.actionIndex_;
}


JVRCEvent* JVRCActionEvent::clone()
{
    return new JVRCActionEvent(*this);
}


bool JVRCActionEvent::isSameAs(JVRCEvent* event)
{
    JVRCActionEvent* action = dynamic_cast<JVRCActionEvent*>(event);
    if(action){
        if(actionIndex_ == action->actionIndex_){
            return JVRCEvent::isSameAs(event);
        }
    }
    return false;
}


void JVRCActionEvent::setActionIndex(int i)
{
    actionIndex_ = i;
}


void JVRCActionEvent::write(YAMLWriter& writer)
{
    JVRCEvent::write(writer);
    writer.putKeyValue("actionIndex", actionIndex_);
}


JVRCTask::JVRCTask(Mapping* info)
    : info_(info)
{
    name_ = info->read<string>("name");
    label_ = info->get("label", name_);

    double m = info->get("timeLimit", 10.0);
    timeLimit_ = m * 60.0;
        
    Listing& eventNodes = *info->findListing("events");
    if(eventNodes.isValid()){
        for(int j=0; j < eventNodes.size(); ++j){
            Mapping* eventInfo = eventNodes[j].toMapping();
            string type = eventInfo->read<string>("type");
            if(type == "gate"){
                addEvent(new JVRCGateEvent(this, eventInfo));
            } else if(type == "action"){
                addEvent(new JVRCActionEvent(this ,eventInfo));
            } else {
                addEvent(new JVRCEvent(type, this, eventInfo));
            }
        }
    }
}


void JVRCTask::addEvent(JVRCEvent* event)
{
    events.push_back(event);

    if(JVRCGateEvent* gate = dynamic_cast<JVRCGateEvent*>(event)){
        gate->setGateIndex(gates.size());
        gates.push_back(gate);
    } else if(JVRCActionEvent* action = dynamic_cast<JVRCActionEvent*>(event)){
        action->setActionIndex(actions.size());
        actions.push_back(action);
    }
}


JVRCActionEvent* JVRCTask::findAction(const std::string& label)
{
    for(size_t i=0; i < actions.size(); ++i){
        if(actions[i]->label() == label){
            return actions[i];
        }
    }
    return 0;
}
