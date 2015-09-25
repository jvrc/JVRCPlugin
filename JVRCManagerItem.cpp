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
#include <cnoid/LazyCaller>
#include <cnoid/EigenUtil>
#include <boost/dynamic_bitset.hpp>
#include <boost/bind.hpp>

#include <iostream>

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

    JVRCTaskPtr currentTask;
    Signal<void()> sigCurrentTaskChanged;

    double startingTime;

    typedef std::vector<JVRCEventPtr> RecordList;
    RecordList records;
    Signal<void()> sigRecordsUpdated;
    
    int nextGateIndex;
    bool isInFrontOfGate;
    bool isPassingGate;

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
    Vector3 spreaderEndPosition;
    std::vector<Vector3, Eigen::aligned_allocator<Vector3> > doorTargetPoints;
    std::vector<Vector3, Eigen::aligned_allocator<Vector3> > doorTargetNormals;
    boost::dynamic_bitset<> doorDestroyFlags;
    int hitCount;
    int hitIndex;

    Link* hoseEndLink;
    Link* nozzleLink;
    
    JVRCManagerItemImpl(JVRCManagerItem* self);
    JVRCManagerItemImpl(JVRCManagerItem* self, const JVRCManagerItemImpl& org);
    ~JVRCManagerItemImpl();
    void initialize();
    void setCurrentTask(JVRCTask* task);
    double currentTime() const;
    bool loadJVRCInfo(const std::string& filename);
    void recordEvent(JVRCEventPtr event, double time, bool isManual);
    JVRCEvent* findRecord(JVRCEvent* event);
    void onPositionChanged();
    void onItemsInWorldChanged();
    void initializeTask_R3_A();
    bool initializeSimulation(SimulatorItem* simulatorItem);
    void startTask_R3_A();
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
    startingTime = 0.0;
    simulatorItem = 0;
    worldItem = 0;
    robotItem = 0;
    robotMarkerLink = 0;
    spreaderItem = 0;
    isEnabled = true;
    taskInfo = new JVRCTaskInfo();
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


void JVRCManagerItem::clearRecords()
{
    impl->records.clear();
    impl->sigRecordsUpdated();
}


int JVRCManagerItem::numRecords() const
{
    return impl->records.size();
}


JVRCEvent* JVRCManagerItem::record(int index)
{
    return impl->records[index];
}


void JVRCManagerItem::recordEvent(JVRCEvent* event, double time, bool isManual)
{
    impl->recordEvent(event, time, isManual);
}


void JVRCManagerItemImpl::recordEvent(JVRCEventPtr event, double time, bool isManual)
{
    JVRCEvent* record = findRecord(event);
    if(!record){
        record = event->clone();
        records.push_back(record);
    }
    if(isManual){
        record->setManualRecordTime(time);
    } else {
        record->setAutomaticRecordTime(time);
    }

    JVRCGateEvent* gate = dynamic_cast<JVRCGateEvent*>(event.get());
    if(gate){
        if(gate->index() == 0){
            startingTime = simulatorItem->simulationTime();
        }
    }
    
    sigRecordsUpdated();
}


JVRCEvent* JVRCManagerItemImpl::findRecord(JVRCEvent* event)
{
    for(RecordList::reverse_iterator p = records.rbegin(); p != records.rend(); ++p){
        JVRCEvent* record = *p;
        if(record->isSameAs(event)){
            return record;
        }
    }
    return 0;
}


void JVRCManagerItem::removeManualRecord(int index)
{
    JVRCEvent* record = impl->records[index];
    record->clearManualRecordTime();
    if(!record->isTimeRecorded()){
        impl->records.erase(impl->records.begin() + index);
    }
    sigRecordsUpdated();
}


SignalProxy<void()> JVRCManagerItem::sigRecordsUpdated()
{
    return impl->sigRecordsUpdated;
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
            double markerRadius = 0.0255;
            for(int i=0; i < body->numLinks(); ++i){
                Link* link = body->link(i);
                markerNode = finder.find(link->visualShape(), "JVRC-Robot-Marker");
                if(markerNode){
                    robotItem = bodyItem;
                    robotMarkerLink = link;
                    robotMarkerLocalPosition = finder.position();
                    //markerRadius = markerNode->boundingBox().boundingSphereRadius();
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
                    robotMarker->setRadius(markerRadius);
                    robotMarker->setTransparency(0.0);
                    //robotMarker->setColor(Vector3f(1.0f, 0.0f, 0.0f));
                    robotMarker->setColor(Vector3f(0.0f, 0.0f, 1.0f));
                    robot->addDevice(robotMarker);
                    robotItem->notifyModelUpdate();
                }
                os << (format(_("JVRC Robot \"%1%\" has been detected.")) % robotItem->name()) << endl;
            }
        }
        sigRobotDetected();
    }

    initializeTask_R3_A();

}


void JVRCManagerItemImpl::initializeTask_R3_A()
{
    JVRCTask* task = taskInfo->findTask("R3_A");
    if(!task){
        return;
    }

    spreaderEndPosition << 0.0, -0.415, 0.016;

    doorTargetPoints.clear();
    Listing& doorPointListing = *task->info()->findListing("doorTargetPoints");
    if(doorPointListing.isValid()){
        for(int i=0; i < doorPointListing.size(); ++i){
            Listing& p = *doorPointListing[i].toListing();
            if(p.size() == 6){
                Vector3 point(p[0].toDouble(), p[1].toDouble(), p[2].toDouble());
                Vector3 normal(rotFromRpy(p[3].toDouble(), p[4].toDouble(), p[5].toDouble()).col(0));
                doorTargetPoints.push_back(point);
                doorTargetNormals.push_back(normal);
            }
        }
    }
    if(doorTargetPoints.empty()){
        return;
    }

    spreaderItem = worldItem->findItem<BodyItem>("spreader");
    if(spreaderItem){
        os << _("The spreader of the R3A task has been detected.") << endl;
        Body* spreader = spreaderItem->body();
        SphereMarkerDevice* marker = spreader->findDevice<SphereMarkerDevice>("HitMarker");
        if(!marker){
            marker = new SphereMarkerDevice();
            marker->setId(0);
            marker->setName("HitMarker");
            marker->setLink(spreader->rootLink());
            marker->setLocalTranslation(spreaderEndPosition);
            marker->on(false);
            marker->setRadius(minMarkerRadius);
            marker->setColor(Vector3f(1.0f, 1.0f, 0.0f));
            marker->setTransparency(0.4f);
            spreader->addDevice(marker);
            os <<_("A virtual device to visualize the hits of the blades has been added to the spreader model.\"") << endl;
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


JVRCTask* JVRCManagerItem::currentTask()
{
    return impl->currentTask;
}


SignalProxy<void()> JVRCManagerItem::sigCurrentTaskChanged()
{
    return impl->sigCurrentTaskChanged;
}


void JVRCManagerItemImpl::setCurrentTask(JVRCTask* task)
{
    if(task != currentTask){
        currentTask = task;
        nextGateIndex = 0;
        sigCurrentTaskChanged();
    }
}


double JVRCManagerItem::startingTime() const
{
    return impl->startingTime;
}


double JVRCManagerItemImpl::currentTime() const
{
    return simulatorItem->simulationTime() - startingTime;
}


bool JVRCManagerItem::loadJVRCInfo(const std::string& filename)
{
    return impl->loadJVRCInfo(filename);
}


bool JVRCManagerItemImpl::loadJVRCInfo(const std::string& filename)
{
    setCurrentTask(0);
    
    if(taskInfo->load(filename)){
        sigTaskInfoUpdated();
        if(taskInfo->numTasks() > 0){
            setCurrentTask(taskInfo->task(0));
        }
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
    startingTime = 0.0;
    
    this->simulatorItem = simulatorItem;

    robotMarker = 0;
    if(robotItem){
        SimulationBody* simRobot = simulatorItem->findSimulationBody(robotItem);
        if(simRobot){
            robotMarker = simRobot->body()->findDevice<SphereMarkerDevice>("JVRCRobotMarker");
            if(robotMarker){
                robotMarker->setColor(Vector3f(1.0f, 0.0f, 0.0f));
                nextGateIndex = 0;
                isInFrontOfGate = false;
                isPassingGate = false;
                simulatorItem->addPostDynamicsFunction(
                    boost::bind(&JVRCManagerItemImpl::checkRobotMarkerPosition, this));
            }
        }
    }

    startTask_R3_A();


    SimulationBody* simHose = simulatorItem->findSimulationBody("HOSE_TRIM_OBJ");
    if(simHose){
        SimulationBody* simNozzle = simulatorItem->findSimulationBody("NOZZLE_OBJ");
        if(simNozzle){
            hoseEndLink = simHose->body()->link("HOSE_TRIM_JOINT12Z");
            nozzleLink = simNozzle->body()->rootLink();
            if(hoseEndLink && nozzleLink){
                os << "The hose and nozzle of the task R6 have been detected." << endl;
                simulatorItem->addPreDynamicsFunction(
                    boost::bind(&JVRCManagerItemImpl::checkConnectionBetweenHoseAndNozzle, this));
            }
        }
    }
    
    return true;
}


void JVRCManagerItemImpl::startTask_R3_A()
{
    spreader = 0;
    if(spreaderItem){
        BodyItem* doorItem = worldItem->findItem<BodyItem>("door");
        if(doorItem){
            SimulationBody* simDoor = simulatorItem->findSimulationBody(doorItem);
            SimulationBody* simSpreader = simulatorItem->findSimulationBody(spreaderItem);
            if(simDoor && simSpreader){
                door = simDoor->body();
                spreader = simSpreader->body();
                spreaderHitMarker = spreader->findDevice<SphereMarkerDevice>("HitMarker");
                if(spreaderHitMarker){
                    doorDestroyFlags.clear();
                    doorDestroyFlags.resize(doorTargetPoints.size());
                    hitCount = 0;
                    hitIndex = -1;
                    //isDoorDestroyed = false;
                    simDoor->setActive(true);
                    spreaderHitMarker->setRadius(minMarkerRadius);
                    os << "The spreader and the car door of Task R3A has been detected." << endl;
                    simulatorItem->addPostDynamicsFunction(
                        boost::bind(&JVRCManagerItemImpl::checkHitBetweenSpreaderAndDoor, this));
                }
            }
        }
    }
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
    if(!currentTask){
        return;
    }
    
    if(nextGateIndex < currentTask->numGates()){
        Vector3 p = (robotMarker->link()->T() * robotMarker->T_local()).translation();
        JVRCGateEvent* gate = currentTask->gate(nextGateIndex);
        int r = checkPositionalRelationshipWithGate(p, gate->location(0), gate->location(1), 0.25);
        if(r < 0){
            robotMarker->setColor(Vector3f(0.0f, 1.0f, 0.0f));
            isInFrontOfGate = true;
            
        } else {
            if(r > 0){
                robotMarker->setColor(Vector3f(0.0f, 0.0f, 1.0f));
                if(isInFrontOfGate){
                    os << "Gate " << gate->index() << " has been passed." << endl;
                    callLater(boost::bind(&JVRCManagerItemImpl::recordEvent, this, gate, currentTime(), false));
                    ++nextGateIndex;
                }
            } else {
                robotMarker->setColor(Vector3f(1.0f, 0.0f, 0.0f));
            }
            isInFrontOfGate = false;
        }
        
        robotMarker->notifyStateChange();
    }
}


void JVRCManagerItemImpl::checkHitBetweenSpreaderAndDoor()
{
    if(doorDestroyFlags.count() == doorDestroyFlags.size()){
        return;
    }
    
    bool isHitting = false;
    Link* spreaderLink = spreader->rootLink();
    const Vector3 orientation = -spreaderLink->T().linear().col(1);
    const Vector3 p = spreaderLink->T() * spreaderEndPosition;
    Link* doorRoot = door->rootLink();
    const Vector3 doorNormal = doorRoot->T().linear().col(0);
    
    for(size_t i=0; i < doorTargetPoints.size(); ++i){
        const Vector3 q = doorRoot->T() * doorTargetPoints[i];
        double distance = (p - q).norm();
        if(distance < 0.03){
            double theta = acos(orientation.dot(doorTargetNormals[i]));
            static const double thresh = (180.0 - 5.0) * M_PI / 180.0;
            if(theta > thresh){
                isHitting = true;
                if(i != hitIndex){
                    hitCount = 0;
                    hitIndex = i;
                }
                break;
            }
        }
    }

    bool changed = false;
    if(isHitting){
        double r = spreaderHitMarker->radius() + 0.0005;
        if(r > maxMarkerRadius){
            if(!doorDestroyFlags[hitIndex]){
                ++hitCount;
                if(hitCount == 10){
                    doorDestroyFlags[hitIndex] = true;

                    JVRCEvent* event = new JVRCEvent("action", currentTask);
                    event->setLabel(str(format("Spreader %1%") % hitIndex));
                    callLater(boost::bind(&JVRCManagerItemImpl::recordEvent, this, event, currentTime(), false));
                    
                    if(doorDestroyFlags.count() == doorDestroyFlags.size()){
                        doorRoot->T().translation().z() -= 2.0;

                        JVRCEvent* event = new JVRCEvent("action", currentTask);
                        event->setLabel("Door");
                        callLater(boost::bind(&JVRCManagerItemImpl::recordEvent, this, event, currentTime(), false));
                    }
                    hitCount = 0;
                }
            }
            r = minMarkerRadius;
        }
        if(doorDestroyFlags[hitIndex]){
            spreaderHitMarker->setColor(Vector3f(1.0f, 0.0f, 0.0f));
        } else {
            spreaderHitMarker->setColor(Vector3f(1.0f, 1.0f, 0.0f));
        }
        spreaderHitMarker->setRadius(r);
        changed = true;
    }
    if(isHitting != spreaderHitMarker->on()){
        spreaderHitMarker->on(isHitting);
        /*
        if(isHitting){
            os << "The spreader is hiting to a target point." << endl;
        } else {
            os << "The spreader is not hitting to any target points." << endl;
        }
        */
        changed = true;
    }
    if(changed){
        spreaderHitMarker->notifyStateChange();
    }
}


void JVRCManagerItemImpl::checkConnectionBetweenHoseAndNozzle()
{
    //cout << "JVRCManagerItemImpl::checkConnectionBetweenHoseAndNozzle()" << endl;
    
    static const Vector3 hoseEndPos(0.0, 0.28, 0.0);
    static const Vector3 forceOnHose(0.0, 200.0, 0.0);
    static const Vector3 nozzleEndPos(0.0, 0.0, -0.55);
    static const Vector3 forceOnNozzle(0.0, 0.0, -200.0);

    Vector3 ph = hoseEndLink->T() * hoseEndPos;
    Vector3 pn = nozzleLink->T() * nozzleEndPos;

    //cout << "distance = " << (ph - pn).norm() << endl;
    
    if((ph - pn).norm() < 0.01){
        Vector3 fh = hoseEndLink->T() * forceOnHose;
        hoseEndLink->f_ext() += fh;
        hoseEndLink->tau_ext() += ph.cross(fh);

        Vector3 fn = nozzleLink->T() * forceOnNozzle;
        nozzleLink->f_ext() += fn;
        nozzleLink->tau_ext() += pn.cross(fn);

        //cout << "connection force is being applied" << endl;
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
