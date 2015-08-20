/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "JVRCManagerItem.h"
#include "JVRCTaskInfo.h"
#include "SphereMarkerDevice.h"
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

    ScopedConnection worldItemConnection;
    WorldItem* worldItem;
    BodyItem* robotItem;
    Signal<void()> sigRobotDetected;

    BodyItem* spreaderItem;
    SphereMarkerDevicePtr spreaderHitMarker;
    Body* spreader;
    Body* door;
    std::vector<Vector3, Eigen::aligned_allocator<Vector3> > doorTargetPoints;
    
    JVRCManagerItemImpl(JVRCManagerItem* self);
    JVRCManagerItemImpl(JVRCManagerItem* self, const JVRCManagerItemImpl& org);
    ~JVRCManagerItemImpl();
    void initialize();
    void onPositionChanged();
    void onItemsInWorldChanged();
    bool initializeSimulation(SimulatorItem* simulatorItem);
    void checkHitBetweenSpreaderAndDoor();
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
    initialize();
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
    initialize();
    isEnabled = org.isEnabled;
}


void JVRCManagerItemImpl::initialize()
{
    simulatorItem = 0;
    worldItem = 0;
    robotItem = 0;
    spreaderItem = 0;
    isEnabled = true;
    taskInfo = new JVRCTaskInfo();

    doorTargetPoints.push_back(Vector3(5.154, 9.012, 1.090));
}


JVRCManagerItem::~JVRCManagerItem()
{
    delete impl;
}


JVRCManagerItemImpl::~JVRCManagerItemImpl()
{

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


void JVRCManagerItem::onPositionChanged()
{
    impl->onPositionChanged();
}


void JVRCManagerItemImpl::onPositionChanged()
{
    worldItemConnection.disconnect();
    worldItem = self->findOwnerItem<WorldItem>();
    if(worldItem){
        worldItemConnection.reset(
            worldItem->sigSubTreeChanged().connect(
                boost::bind(&JVRCManagerItemImpl::onItemsInWorldChanged, this)));
    }
}


void JVRCManagerItemImpl::onItemsInWorldChanged()
{
    ItemList<BodyItem> bodyItems;
    BodyItem* bodyItem = 0;
    if(bodyItems.extractChildItems(worldItem)){
        bodyItem = bodyItems.front();
    }
    if(bodyItem != robotItem){
        robotItem = bodyItem;
        sigRobotDetected();
    }
    
    spreaderItem = worldItem->findItem<BodyItem>("Task_R4A-spreader");
    if(spreaderItem){
        os << (format(_("The spreader \"%1%\" of the task R4A has been detected.")) % spreaderItem->name()) << endl;
        Body* spreader = spreaderItem->body();
        SphereMarkerDevice* spreaderHitMarker = spreader->findDevice<SphereMarkerDevice>("HitMarker");
        if(!spreaderHitMarker){
            spreaderHitMarker = new SphereMarkerDevice();
            spreaderHitMarker->setId(0);
            spreaderHitMarker->setName("HitMarker");
            spreaderHitMarker->setLink(spreader->rootLink());
            spreaderHitMarker->setLocalTranslation(Vector3(0.16, 0.0, 0.0));
            spreaderHitMarker->on(false);
            spreaderHitMarker->setRadius(0.12);
            spreaderHitMarker->setColor(Vector3f(1.0f, 1.0f, 0.0f));
            spreaderHitMarker->setTransparency(0.4f);
            spreader->addDevice(spreaderHitMarker);
            os <<_("A virtual device to visualize the hits of the spreader's blades has been added to \"Task_R4A_spreader.\"") << endl;
            spreaderItem->notifyModelUpdate();
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

    if(spreaderItem){
        BodyItem* doorItem = worldItem->findItem<BodyItem>("Task_R4A-Door-Task_R4A-visual");
        if(doorItem){
            SimulationBody* simDoor = simulatorItem->findSimulationBody(doorItem);
            SimulationBody* simSpreader = simulatorItem->findSimulationBody(spreaderItem);
            if(simDoor && simSpreader){
                door = simDoor->body();
                spreader = simSpreader->body();
                spreaderHitMarker = spreader->findDevice<SphereMarkerDevice>("HitMarker");
                if(spreaderHitMarker){
                    os << "The spreader and the car door of Task R4A has been detected." << endl;
                    simulatorItem->addPostDynamicsFunction(
                        boost::bind(&JVRCManagerItemImpl::checkHitBetweenSpreaderAndDoor, this));
                }
            }
        }
    }
    
    return true;
}


void JVRCManagerItemImpl::checkHitBetweenSpreaderAndDoor()
{
    bool isHitting = false;
    Link* spreaderLink = spreader->rootLink();
    const Vector3 p = spreaderLink->T() * Vector3(0.165, 0.0, 0.0);
    Link* doorRoot = door->rootLink();
    for(size_t i=0; i < doorTargetPoints.size(); ++i){
        const Vector3 q = doorRoot->T() * doorTargetPoints[i];
        isHitting = (p - q).norm() < 0.05;
        if(isHitting){
            break;
        }
    }
    if(isHitting != spreaderHitMarker->on()){
        spreaderHitMarker->on(isHitting);
        spreaderHitMarker->notifyStateChange();
        if(isHitting){
            os << "The spreader is hiting to a target point." << endl;
        } else {
            os << "The spreader is not hitting to any target points." << endl;
        }
    }
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
