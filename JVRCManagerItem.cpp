/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "JVRCManagerItem.h"
#include "JVRCTaskInfo.h"
#include <cnoid/SimulatorItem>
#include <cnoid/WorldItem>
#include <cnoid/BodyItem>
#include <cnoid/ItemManager>
#include <cnoid/MessageView>
#include <cnoid/Archive>
#include <cnoid/Body>
#include <boost/tokenizer.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/bind.hpp>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using boost::format;

namespace {

JVRCManagerItemPtr instance_;

string getNameListString(const vector<string>& names)
{
    string nameList;
    if(!names.empty()){
        size_t n = names.size() - 1;
        for(size_t i=0; i < n; ++i){
            nameList += names[i];
            nameList += ", ";
        }
        nameList += names.back();
    }
    return nameList;
}

bool updateNames(const string& nameListString, string& newNameListString, vector<string>& names)
{
    using boost::tokenizer;
    using boost::char_separator;
    
    names.clear();
    char_separator<char> sep(",");
    tokenizer< char_separator<char> > tok(nameListString, sep);
    for(tokenizer< char_separator<char> >::iterator p = tok.begin(); p != tok.end(); ++p){
        string name = boost::trim_copy(*p);
        if(!name.empty()){
            names.push_back(name);
        }
    }
    newNameListString = nameListString;
    return true;
}

}

namespace cnoid {

class JVRCManagerItemImpl
{
public:
    JVRCManagerItem* self;
    ostream& os;
    SimulatorItem* simulatorItem;
    bool isEnabled;

    JVRCTaskInfoPtr taskInfo;
    Signal<void()> sigTaskInfoUpdated;

    WorldItem* worldItem;
    BodyItem* robotItem;
    Signal<void()> sigRobotDetected;

    JVRCManagerItemImpl(JVRCManagerItem* self);
    JVRCManagerItemImpl(JVRCManagerItem* self, const JVRCManagerItemImpl& org);
    ~JVRCManagerItemImpl();
    void detectRobotItem();
    bool initializeSimulation(SimulatorItem* simulatorItem);
    void onPreDynamics();
    void onPostDynamics();
    void finalizeSimulation();
    void doPutProperties(PutPropertyFunction& putProperty);
    bool store(Archive& archive);
    bool restore(const Archive& archive);
};

}


void JVRCManagerItem::initializeClass(ExtensionManager* ext)
{
    ItemManager& im = ext->itemManager();
    instance_ = new JVRCManagerItem;
    im.registerClass<JVRCManagerItem>(N_("JVRCManagerItem"), instance_);
    im.addLoader<JVRCManagerItem>(_("JVRC Info"), "JVRC-INFO", "yaml",
                                  boost::bind(&JVRCManagerItem::loadJVRCInfo, _1, _2));
}


JVRCManagerItem* JVRCManagerItem::instance()
{
    return instance_;
}


JVRCManagerItem::JVRCManagerItem()
{
    impl = new JVRCManagerItemImpl(this);
}


JVRCManagerItemImpl::JVRCManagerItemImpl(JVRCManagerItem* self)
    : self(self),
      os(MessageView::instance()->cout())
{
    simulatorItem = 0;
    worldItem = 0;
    robotItem = 0;
    isEnabled = true;
    taskInfo = new JVRCTaskInfo();
}


JVRCManagerItem::JVRCManagerItem(const JVRCManagerItem& org)
    : SubSimulatorItem(org)
{
    impl = new JVRCManagerItemImpl(this, *org.impl);
}


JVRCManagerItemImpl::JVRCManagerItemImpl(JVRCManagerItem* self, const JVRCManagerItemImpl& org)
    : self(self),
      os(MessageView::instance()->cout())
{
    simulatorItem = 0;
    worldItem = 0;
    robotItem = 0;
    isEnabled = org.isEnabled;
    taskInfo = new JVRCTaskInfo();
}


JVRCTaskInfoPtr JVRCManagerItem::taskInfo()
{
    return impl->taskInfo;
}


SignalProxy<void()> JVRCManagerItem::sigTaskInfoUpdated()
{
    return impl->sigTaskInfoUpdated;
}


bool JVRCManagerItem::isEnabled()
{
    return impl->isEnabled;
}


ItemPtr JVRCManagerItem::doDuplicate() const
{
    return new JVRCManagerItem(*this);
}


JVRCManagerItem::~JVRCManagerItem()
{
    delete impl;
}


JVRCManagerItemImpl::~JVRCManagerItemImpl()
{

}


void JVRCManagerItem::onPositionChanged()
{
    impl->detectRobotItem();
}


void JVRCManagerItemImpl::detectRobotItem()
{
    worldItem = self->findOwnerItem<WorldItem>();
    if(worldItem){
        ItemList<BodyItem> bodyItems;
        BodyItem* bodyItem = 0;
        if(bodyItems.extractChildItems(worldItem)){
            bodyItem = bodyItems.front();
        }
        if(bodyItem != robotItem){
            robotItem = bodyItem;
            sigRobotDetected();
        }
    }
}


BodyItem* JVRCManagerItem::robotItem()
{
    return impl->robotItem;
}


SignalProxy<void()> JVRCManagerItem::sigRobotDetected()
{
    return impl->sigRobotDetected;
}


bool JVRCManagerItem::loadJVRCInfo(const std::string& filename)
{
    if(impl->taskInfo->load(filename)){
        impl->sigTaskInfoUpdated();
        return true;
    }
    return false;
}


bool JVRCManagerItem::initializeSimulation(SimulatorItem* simulatorItem)
{
    return impl->initializeSimulation(simulatorItem);
}


bool JVRCManagerItemImpl::initializeSimulation(SimulatorItem* simulatorItem)
{
    this->simulatorItem = simulatorItem;
    return true;
}


void JVRCManagerItemImpl::onPreDynamics()
{
    
}


void JVRCManagerItemImpl::onPostDynamics()
{

}


void JVRCManagerItem::finalizeSimulation()
{
    impl->finalizeSimulation();
}


void JVRCManagerItemImpl::finalizeSimulation()
{

}


void JVRCManagerItem::doPutProperties(PutPropertyFunction& putProperty)
{
    impl->doPutProperties(putProperty);
}


void JVRCManagerItemImpl::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty("Enabled", isEnabled, changeProperty(isEnabled));
}


bool JVRCManagerItem::store(Archive& archive)
{
    SubSimulatorItem::store(archive);
    return impl->store(archive);
}


bool JVRCManagerItemImpl::store(Archive& archive)
{
    archive.writeRelocatablePath("info", self->filePath());    
    return true;
}


bool JVRCManagerItem::restore(const Archive& archive)
{
    SubSimulatorItem::restore(archive);
    return impl->restore(archive);
}


bool JVRCManagerItemImpl::restore(const Archive& archive)
{
    string filename;
    if(archive.readRelocatablePath("info", filename)){
        return self->load(filename);
    }
    return false;
}
