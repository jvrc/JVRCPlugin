/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "JVRCManagerItem.h"
#include "JVRCTaskInfo.h"
#include "SphereMarkerDevice.h"
#include "SceneNodeFinder.h"
#include <cnoid/SimulatorItem>
#include <cnoid/WorldItem>
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

const double minMarkerRadius = 0.01;
const double maxMarkerRadius = 0.15;

JVRCManagerItemPtr instance_;

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
    Link* robotMarkerLink;
    Position robotMarkerLocalPosition;
    Signal<void()> sigRobotDetected;
    SphereMarkerDevicePtr robotMarker;

    BodyItem* spreaderItem;
    SphereMarkerDevicePtr spreaderHitMarker;
    Body* spreader;
    Body* door;
    std::vector<Vector3, Eigen::aligned_allocator<Vector3> > doorTargetPoints;
    int hitCount;
    bool isDoorDestroyed;

    Body* hose;
    Body* nozzle;
    
    JVRCManagerItemImpl(JVRCManagerItem* self);
    JVRCManagerItemImpl(JVRCManagerItem* self, const JVRCManagerItemImpl& org);
    ~JVRCManagerItemImpl();
    void initialize();
    void onPositionChanged();
    void onItemsInWorldChanged();
    bool initializeSimulation(SimulatorItem* simulatorItem);
    static int checkPositionalRelationshipWithGate(
        Vector3 p, const Vector3& g1, const Vector3& g2, double distanceThresh);
    void checkRobotMarkerPosition();
    void checkHitBetweenSpreaderAndDoor();
    void checkConnectionBetweenHoseAndNozzle();
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
    robotMarkerLink = 0;
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
        robotItem = 0;
        robotMarkerLink = 0;
        if(bodyItem){
            Body* body = bodyItem->body();
            SceneNodeFinder finder;
            SgNode* markerNode = 0;
            for(int i=0; i < body->numLinks(); ++i){
                Link* link = body->link(i);
                markerNode = finder.find(link->visualShape(), "JVRC-Robot-Marker");
                if(markerNode){
                    robotItem = bodyItem;
                    robotMarkerLink = link;
                    robotMarkerLocalPosition = finder.position();
                    break;
                }
            }
            if(!markerNode){
                os << (format(_("Warning: \"%1%\" does not have the JVRC marker.")) % bodyItem->name()) << endl;
            } else {
                Body* robot = robotItem->body();
                SphereMarkerDevice* robotMarker = robot->findDevice<SphereMarkerDevice>("JVRCRobotMarker");
                if(!robotMarker){
                    robotMarker = new SphereMarkerDevice();
                    robotMarker->setId(0);
                    robotMarker->setName("JVRCRobotMarker");
                    robotMarker->setLink(robotMarkerLink);
                    robotMarker->setLocalTranslation(robotMarkerLocalPosition.translation());
                    robotMarker->on(true);
                    robotMarker->setRadius(0.055);
                    robotMarker->setTransparency(0.0);
                    robotMarker->setColor(Vector3f(1.0f, 0.0f, 0.0f));
                    robot->addDevice(robotMarker);
                    robotItem->notifyModelUpdate();
                }
                os << (format(_("JVRC Robot \"%1%\" has been detected.")) % robotItem->name()) << endl;
            }
        }
        sigRobotDetected();
    }
    
    spreaderItem = worldItem->findItem<BodyItem>("Task_R4A-spreader");
    if(spreaderItem){
        os << (format(_("The spreader \"%1%\" of the task R4A has been detected.")) % spreaderItem->name()) << endl;
        Body* spreader = spreaderItem->body();
        SphereMarkerDevice* marker = spreader->findDevice<SphereMarkerDevice>("HitMarker");
        if(!marker){
            marker = new SphereMarkerDevice();
            marker->setId(0);
            marker->setName("HitMarker");
            marker->setLink(spreader->rootLink());
            marker->setLocalTranslation(Vector3(0.16, 0.0, 0.0));
            marker->on(false);
            marker->setRadius(minMarkerRadius);
            marker->setColor(Vector3f(1.0f, 1.0f, 0.0f));
            marker->setTransparency(0.4f);
            spreader->addDevice(marker);
            os <<_("A virtual device to visualize the hits of the spreader's blades has been added to \"Task_R4A_spreader.\"") << endl;
            spreaderItem->notifyModelUpdate();
        }
    }

}


BodyItem* JVRCManagerItem::robotItem()
{
    return impl->robotItem;
}


Position JVRCManagerItem::robotMarkerPosition() const
{
    Position T;
    if(impl->robotMarkerLink){
        T = impl->robotMarkerLink->T() * impl->robotMarkerLocalPosition;
    } else {
        T.setIdentity();
    }
    return T;
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

    robotMarker = 0;
    if(robotItem){
        SimulationBody* simRobot = simulatorItem->findSimulationBody(robotItem);
        if(simRobot){
            robotMarker = simRobot->body()->findDevice<SphereMarkerDevice>("JVRCRobotMarker");
            if(robotMarker){
                robotMarker->setColor(Vector3f(1.0f, 0.0f, 0.0f));
                simulatorItem->addPostDynamicsFunction(
                    boost::bind(&JVRCManagerItemImpl::checkRobotMarkerPosition, this));
            }
        }
    }

    spreader = 0;
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
                    hitCount = 0;
                    isDoorDestroyed = false;
                    simDoor->setActive(true);
                    spreaderHitMarker->setRadius(minMarkerRadius);
                    os << "The spreader and the car door of Task R4A has been detected." << endl;
                    simulatorItem->addPostDynamicsFunction(
                        boost::bind(&JVRCManagerItemImpl::checkHitBetweenSpreaderAndDoor, this));
                }
            }
        }
    }

    SimulationBody* simHose = simulatorItem->findSimulationBody("HOSE_TRIM_OBJ");
    if(simHose){
        SimulationBody* simNozzle = simulatorItem->findSimulationBody("NOZZLE_OBJ");
        if(simNozzle){
            hose = simHose->body();
            nozzle = simNozzle->body();
            os << "The hose and nozzle of the task R6 have been detected." << endl;
            simulatorItem->addPreDynamicsFunction(
                boost::bind(&JVRCManagerItemImpl::checkConnectionBetweenHoseAndNozzle, this));
        }
    }
    
    return true;
}


int JVRCManagerItemImpl::checkPositionalRelationshipWithGate
(Vector3 p, const Vector3& g1, const Vector3& g2, double distanceThresh)
{
    p.z() = 0.0;
    
    if((g2 - g1).dot(p - g1) < 0.0){
        return 0;
    }
    if((g1 - g2).dot(p - g2) < 0.0){
        return 0;
    }
    Vector3 a = g2 - g1;
    Vector3 c = a.cross(p - g1);
    double d = c.norm() / a.norm();
    if(d <= distanceThresh){
        if(c.z() > 0.0){
            return -1;
        } else {
            return 1;
        }
    }
    return 0;
}


void JVRCManagerItemImpl::checkRobotMarkerPosition()
{
    static const Vector3 g1(2.0, -1.1, 0.0);
    static const Vector3 g2(2.0, 1.7, 0.0);
    
    if(robotMarker){
        Vector3 p = (robotMarker->link()->T() * robotMarker->T_local()).translation();
        int r = checkPositionalRelationshipWithGate(p, g1, g2, 0.25);
        if(r < 0){
            robotMarker->setColor(Vector3f(0.0f, 1.0f, 0.0f));
        } else if(r > 0){
            robotMarker->setColor(Vector3f(0.0f, 0.0f, 1.0f));
        } else {
            robotMarker->setColor(Vector3f(1.0f, 0.0f, 0.0f));
        }
        robotMarker->notifyStateChange();
    }
}


void JVRCManagerItemImpl::checkHitBetweenSpreaderAndDoor()
{
    if(isDoorDestroyed){
        return;
    }
    
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
    bool changed = false;
    if(isHitting){
        double r = spreaderHitMarker->radius() + 0.0005;
        if(r > maxMarkerRadius){
            ++hitCount;
            if(hitCount == 10){
                doorRoot->T().translation().z() -= 2.0;
                isDoorDestroyed = true;
                isHitting = false;
            }
            r = minMarkerRadius;
        }
        spreaderHitMarker->setRadius(r);
        changed = true;
    }
    if(isHitting != spreaderHitMarker->on()){
        spreaderHitMarker->on(isHitting);
        if(isHitting){
            os << "The spreader is hiting to a target point." << endl;
        } else {
            os << "The spreader is not hitting to any target points." << endl;
        }
        changed = true;
    }
    if(changed){
        spreaderHitMarker->notifyStateChange();
    }
}


void JVRCManagerItemImpl::checkConnectionBetweenHoseAndNozzle()
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
