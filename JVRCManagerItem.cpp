/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "JVRCManagerItem.h"
#include "SphereMarkerDevice.h"
#include "SceneNodeFinder.h"
#include <cnoid/SimulatorItem>
#include <cnoid/WorldItem>
#include <cnoid/ItemManager>
#include <cnoid/MainWindow>
#include <cnoid/MessageView>
#include <cnoid/Archive>
#include <cnoid/Body>
#include <cnoid/LazyCaller>
#include <cnoid/EigenUtil>
#include <cnoid/FileUtil>
#include <cnoid/YAMLReader>
#include <cnoid/ConnectionSet>
#include <cnoid/AppConfig>
#include <cnoid/MenuManager>
#include <cnoid/Timer>
#include <QFileDialog>
#include <boost/dynamic_bitset.hpp>
#include <boost/bind.hpp>
#include <algorithm>

#include <iostream>

#include "gettext.h"

using namespace std;
using namespace cnoid;
using namespace boost;

namespace {

bool START_COMPETITION_AT_STARTING_SIMULATION = true;

Action* autoCopyFromDetectedToJudgedCheck;

void onAutoCopyFromDetectedToJudgedCheckToggled(bool on)
{
    AppConfig::archive()->openMapping("JVRCPlugin")->write("autoCopyFromDetectedToJudged", on);
}


const double minMarkerRadius = 0.01;
const double maxMarkerRadius = 0.15;

JVRCManagerItemPtr instance_;

struct EventTimeCmp {
    bool operator()(const JVRCEventPtr& r1, const JVRCEventPtr& r2) const {
        return r1->time() < r2->time();
    }
};

}

namespace cnoid {

class JVRCManagerItemImpl
{
public:
    JVRCManagerItem* self;
    MessageView* mv;
    ostream& os;

    std::vector<JVRCTaskPtr> tasks;
    Signal<void()> sigTasksUpdated;
    JVRCTaskPtr currentTask;
    Signal<void()> sigCurrentTaskChanged;

    SimulatorItem* simulatorItem;
    ScopedConnectionSet simulatorConnections;
    double timeStep;
    double offsetTime;
    optional<double> startingTime;
    optional<double> goalTime;
    string remainingTimeOutputDirectory;
    std::ofstream remainingTimeOutputStream;
    LazyCaller checkRemainingTimeLater;
    double lastRemainingTimeOutput;
    Signal<void(bool isDoingSimulation)> sigSimulationStateChanged;

    typedef std::vector<JVRCEventPtr> RecordList;
    RecordList records;

    vector< shared_ptr<RecordList> > recordHistory;
    int currentHistoryIndex;
    
    Signal<void()> sigRecordUpdated; 
    int score;
    string recordOutputDirectory;
    string recordFileNameBase;
    
    int nextGateIndex;
    bool isInFrontOfGate;
    bool isPassingGate;

    ScopedConnection worldItemConnection;
    WorldItem* worldItem;

    ItemList<BodyItem> bodyItems;

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
    double hitTimeCount;
    int hitIndex;

    Link* hoseEndLink;
    Link* nozzleLink;
    
    JVRCManagerItemImpl(JVRCManagerItem* self);
    JVRCManagerItemImpl(JVRCManagerItem* self, const JVRCManagerItemImpl& org);
    ~JVRCManagerItemImpl();
    void initialize();
    bool loadRecords(const std::string& filename);
    void setCurrentTask(JVRCTask* task);
    static std::string toTimeString(double time, int formatType = 0);
    double elapsedTime(bool isGoalTimeMax = false) const;
    void startRemainingTimeChecker();
    void checkRemainingTime();
    void stopRemainingTimeChecker();
    void requestToAbort();
    bool loadJVRCInfo(const std::string& filename);
    void addRecord(JVRCEventPtr event, double time, bool isJudged);
    JVRCEvent* findRecord(JVRCEvent* event);
    void notifyRecordUpdate();
    void storeRecordsToHistory(bool isNewEditing);
    bool restoreRecordsFromHistory(int index);
    void resetRecordFileName();
    void saveRecords();
    void saveRecordsAsYAML();
    void saveRecordsAsCSV();
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

    MenuManager& mm = ext->menuManager();
    MappingPtr config = AppConfig::archive()->openMapping("JVRCPlugin");
    mm.setPath("/Options").setPath(N_("JVRC Plugin"));
    autoCopyFromDetectedToJudgedCheck = mm.addCheckItem(_("Automatic copy from the detected time to the judged time"));
    autoCopyFromDetectedToJudgedCheck->setChecked(config->get("autoCopyFromDetectedToJudged", true));
    autoCopyFromDetectedToJudgedCheck->sigToggled().connect(onAutoCopyFromDetectedToJudgedCheckToggled);
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
      mv(MessageView::instance()),
      os(mv->cout())
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
      mv(MessageView::instance()),
      os(mv->cout())
{
    initialize();
}


void JVRCManagerItemImpl::initialize()
{
    remainingTimeOutputDirectory = "/home/samba/Time/";
    checkRemainingTimeLater.setFunction(
        boost::bind(&JVRCManagerItemImpl::checkRemainingTime, this));

    recordOutputDirectory = "/media/player/JVRC4GU/.hidden/Score_JSS/";

    currentHistoryIndex = 0;
    offsetTime = 0.0;
    simulatorItem = 0;
    score = 0;
    worldItem = 0;
    robotItem = 0;
    robotMarkerLink = 0;
    spreaderItem = 0;

    storeRecordsToHistory(true);
}


JVRCManagerItem::~JVRCManagerItem()
{
    delete impl;
}


JVRCManagerItemImpl::~JVRCManagerItemImpl()
{

}


int JVRCManagerItem::numTasks() const
{
    return impl->tasks.size();
}


JVRCTask* JVRCManagerItem::task(int index)
{
    return impl->tasks[index];
}


JVRCTask* JVRCManagerItem::findTask(const std::string& name)
{
    for(size_t i=0; i < impl->tasks.size(); ++i){
        if(impl->tasks[i]->name() == name){
            return impl->tasks[i];
        }
    }
    return 0;
}


SignalProxy<void()> JVRCManagerItem::sigTasksUpdated()
{
    return impl->sigTasksUpdated;
}


Position JVRCManagerItem::startingPosition() const
{
    Position T = Position::Identity();
    if(!impl->tasks.empty()){
        JVRCTask* task = impl->tasks.front();
        if(task->numGates() > 0){
            JVRCGateEvent* gate = task->gate(0);
            Vector3 c = (gate->location(0) + gate->location(1)) / 2.0;
            Vector3 y = (gate->location(1) - gate->location(0)).normalized();
            Vector3 x = y.cross(Vector3::UnitZ());
            T.translation() = c - x;
            T.linear().col(0) = x;
            T.linear().col(1) = y;
            T.linear().col(2) = Vector3::UnitZ();
        }
    }
    return T;
}
            

Item* JVRCManagerItem::doDuplicate() const
{
    return new JVRCManagerItem(*this);
}


void JVRCManagerItem::clearRecords(bool doNotify)
{
    impl->records.clear();
    if(doNotify){
        notifyRecordUpdate();
    }
}


bool JVRCManagerItem::showDialogToLoadRecords()
{
    QString directory;
    if(filesystem::exists(impl->recordOutputDirectory)){
        directory = impl->recordOutputDirectory.c_str();
    } else {
        directory = QDir::currentPath();
    }
    
    QString filename =
        QFileDialog::getOpenFileName(
            MainWindow::instance(),
            _("Load a JVRC event record file"),
            directory, _("JVRC event record files (*.yaml)"));

    if(!filename.isNull()){
        return loadRecords(filename.toStdString());
    }
    return false;
}


bool JVRCManagerItem::loadRecords(const std::string& filename)
{
    return impl->loadRecords(filename);
}


bool JVRCManagerItemImpl::loadRecords(const std::string& filename)
{
    self->clearRecords(false);
    
    YAMLReader reader;
    MappingPtr info = reader.loadDocument(filename)->toMapping();

    offsetTime = info->get("elapsedTime", 0.0);

    const Listing& recordNodes = *info->findListing("records");
    if(recordNodes.isValid()){
        for(int i=0; i < recordNodes.size(); ++i){
            const Mapping& node = *recordNodes[i].toMapping();
            JVRCEvent* record = 0;
            string label = node.read<string>("label");
            string taskName = node.read<string>("task");
            JVRCTask* task = self->findTask(taskName);
            if(!task){
                mv->putln(MessageView::WARNING,
                          format("Task \"%1%\" for record \"%2%\" is not found.")
                          % taskName % label);
            } else {
                string type = node.read<string>("type");
                if(type == "gate"){
                    JVRCGateEvent* gate = new JVRCGateEvent(task);
                    gate->setGateIndex(node.read<int>("gateIndex"));
                    record = gate;
                } else if(type == "action"){
                    JVRCActionEvent* action = new JVRCActionEvent(task);
                    action->setActionIndex(node.read<int>("actionIndex"));
                    record = action;
                } else {
                    record = new JVRCEvent(type, task);
                }
                record->setLabel(label);
                double time;
                if(node.read("detectedTime", time)){
                    record->setDetectedTime(time);
                }
                if(node.read("judgedTime", time)){
                    record->setJudgedTime(time);
                }
                records.push_back(record);
            }
        }
    }

    notifyRecordUpdate();
    
    return true;
}


int JVRCManagerItem::numRecords() const
{
    return impl->records.size();
}


JVRCEvent* JVRCManagerItem::record(int index)
{
    return impl->records[index];
}


void JVRCManagerItem::addRecord(JVRCEvent* event, double time, bool isJudged)
{
    impl->addRecord(event, time, isJudged);
}


void JVRCManagerItemImpl::addRecord(JVRCEventPtr event, double time, bool isJudged)
{
    JVRCEvent* record = findRecord(event);
    if(!record){
        record = event->clone();
        records.push_back(record);
    }
    if(isJudged){
        record->setJudgedTime(time);
    } else {
        record->setDetectedTime(time);
        if(autoCopyFromDetectedToJudgedCheck->isChecked()){
            if(!record->judgedTime()){
                record->setJudgedTime(time);
            }
        }
    }

    notifyRecordUpdate();
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


SignalProxy<void()> JVRCManagerItem::sigRecordUpdated()
{
    return impl->sigRecordUpdated;
}


void JVRCManagerItem::notifyRecordUpdate()
{
    impl->notifyRecordUpdate();
}


void JVRCManagerItemImpl::notifyRecordUpdate()
{
    goalTime = boost::none;
    score = 0;

    RecordList::iterator iter = records.begin();
    while(iter != records.end()){
        JVRCEvent* record = *iter;
        if(!record->isTimeRecorded()){
            iter = records.erase(iter);
        } else {
            if(record->judgedTime()){
                JVRCGateEvent* gate = dynamic_cast<JVRCGateEvent*>(record);
                if(gate && gate->isGoal()){
                    goalTime = *gate->judgedTime();
                }
                score += record->point();
            }
            ++iter;
        }
    }

    std::sort(records.begin(), records.end(), EventTimeCmp());

    storeRecordsToHistory(true);
    saveRecords();
    sigRecordUpdated();
}


void JVRCManagerItemImpl::storeRecordsToHistory(bool isNewEditing)
{
    shared_ptr<RecordList> stored(new RecordList());
    stored->reserve(records.size());
    for(size_t i=0; i < records.size(); ++i){
        stored->push_back(records[i]->clone());
    }
    if(isNewEditing){
        currentHistoryIndex = recordHistory.size();
    }
    recordHistory.push_back(stored);
}


bool JVRCManagerItemImpl::restoreRecordsFromHistory(int index)
{
    if(index >= 0 && index < recordHistory.size()){
        shared_ptr<RecordList> stored = recordHistory[index];
        records.clear();
        for(size_t i=0; i < stored->size(); ++i){
            records.push_back((*stored)[i]->clone());
        }
        return true;
    }
    return false;
}


void JVRCManagerItem::undoRecordEditing()
{
    if(impl->restoreRecordsFromHistory(impl->currentHistoryIndex - 1)){
        --impl->currentHistoryIndex;
        impl->saveRecords();
        impl->sigRecordUpdated();
    }
}


void JVRCManagerItem::redoRecordEditing()
{
    if(impl->restoreRecordsFromHistory(impl->currentHistoryIndex + 1)){
        ++impl->currentHistoryIndex;
        impl->saveRecords();
        impl->sigRecordUpdated();
    }
}
        

int JVRCManagerItem::score() const
{
    return impl->score;
}


void JVRCManagerItemImpl::resetRecordFileName()
{
    recordFileNameBase.clear();

    if(!currentTask){
        return;
    }

    bool recordOutputDirectoryExists = filesystem::exists(recordOutputDirectory);

    string teamNameFileName = "jvrc-team-name.txt";
    if(recordOutputDirectoryExists){
        teamNameFileName = recordOutputDirectory + teamNameFileName;
    }
    
    string teamName;
    ifstream teamNameFile(teamNameFileName.c_str());
    teamNameFile >> teamName;

    if(teamName.empty()){
        mv->putln(MessageView::WARNING,
                  format("\"%1%\" is not found. Outputting record files is not available.")
                  % teamNameFileName);
        return;
    }
    os << (format("Team name \"%1%\" was obtained from \"%2%\".") % teamName % teamNameFileName) << endl;
    
    string prefix = str(format("R_%1%_%2%") % teamName % currentTask->name());

    if(recordOutputDirectoryExists){
        prefix = recordOutputDirectory + prefix;
    }

    for(int i=0; i < 100; ++i){
        string basename = str(format("%1%_%2$02d") % prefix % i);
        string yamlfile = basename + ".yaml";
        string csvfile = basename + ".csv";
        if(!filesystem::exists(yamlfile) && !filesystem::exists(csvfile)){
            recordFileNameBase = basename;
            os << (format("Records are saved to \"%1%.(yaml, csv)\".") % basename) << endl;
            break;
        }
    }

    if(recordFileNameBase.empty()){
        mv->putln(MessageView::WARNING,
                  "There is no empty number for the record file suffix. Outputting record files is not available.");
    }
}


void JVRCManagerItemImpl::saveRecords()
{
    if(recordFileNameBase.empty()){
        return;
    }

    saveRecordsAsYAML();
    saveRecordsAsCSV();
}


void JVRCManagerItemImpl::saveRecordsAsYAML()
{
    YAMLWriter writer(recordFileNameBase + ".yaml");
    writer.setKeyOrderPreservationMode(true);
    writer.putComment("JVRC Score Record File\n");

    writer.startMapping();

    if(!records.empty()){
        writer.putKey("records");
        writer.startListing();
        for(size_t i=0; i < records.size(); ++i){
            writer.startMapping();
            records[i]->write(writer);
            writer.endMapping();
        }
        writer.endListing();
    }

    writer.putKeyValue("elapsedTime", elapsedTime(true));

    writer.endMapping();
}


void JVRCManagerItemImpl::saveRecordsAsCSV()
{
    ofstream ofs((recordFileNameBase + ".csv").c_str());

    optional<double> endTime;
    for(RecordList::reverse_iterator p = records.rbegin(); p != records.rend(); ++p){
        if(JVRCGateEvent* gate = dynamic_cast<JVRCGateEvent*>(p->get())){
            if(gate->isGoal() && gate->judgedTime()){
                endTime = *gate->judgedTime();
                break;
            }
        }
    }

    ofs << "ST-GT," << toTimeString(0.0, 1);
    if(endTime){
        ofs << "," << toTimeString(*endTime, 1);
    }
    ofs << "\n";

    if(currentTask->numGates() >= 3){
        vector<optional<double> > timeRecords(currentTask->numGates());
        for(size_t i=0; i < records.size(); ++i){
            if(JVRCGateEvent* gate = dynamic_cast<JVRCGateEvent*>(records[i].get())){
                timeRecords[gate->gateIndex()] = gate->judgedTime();
            }
        }
        for(size_t i=0; i < timeRecords.size() - 1; ++i){
            ofs << currentTask->gate(i)->subTaskLabel() << ",";
            int passed = (timeRecords[i] && timeRecords[i+1]);
            ofs << passed;
            if(timeRecords[i]){
                ofs << "," << toTimeString(*timeRecords[i], 1);
            }
            if(timeRecords[i+1]){
                ofs << "," << toTimeString(*timeRecords[i+1], 1);
            }
            ofs << "\n";
        }
    } else if(currentTask->numActions() > 0){
        dynamic_bitset<> clearFlags;
        clearFlags.resize(currentTask->numActions());
        for(size_t i=0; i < records.size(); ++i){
            if(JVRCActionEvent* action = dynamic_cast<JVRCActionEvent*>(records[i].get())){
                clearFlags[action->actionIndex()] = action->judgedTime();
            }
        }
        for(size_t i=0; i < clearFlags.size(); ++i){
            int passed = clearFlags[i] ? 1 : 0;
            ofs << currentTask->action(i)->subTaskLabel() << "," << passed << "\n";
        }
    }
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
    BodyItem* bodyItem = 0;

    ItemList<BodyItem> newBodyItems;
    newBodyItems.extractChildItems(worldItem);

    if(newBodyItems == bodyItems){
        return;
    }
    bodyItems = newBodyItems;

    if(!bodyItems.empty()){
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
    JVRCTask* task = self->findTask("R3-A");
    if(!task){
        task = self->findTask("R3_A");
    }
    if(!task){
        return;
    }

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
        mv->putln(MessageView::WARNING,
                  "There is no information on the door target points of the task R3-A.");
        return;
    }

    spreaderEndPosition << 0.0, -0.415, 0.016;
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


std::string JVRCManagerItemImpl::toTimeString(double time, int formatType)
{
    int hour = floor(time / 60.0 / 60.0);
    time -= hour * 60.0 * 60.0;
    int min = floor(time / 60.0);
    time -= min * 60.0;
    switch(formatType){
    case 0:
        return str(format("%1$02d:%2$02i") % min % time);
    case 1:
        return str(format("%1$01d:%2$02d:%3$02i") % hour % min % floor(time));
    default:
        return "";
    }
}


QString JVRCManagerItem::toTimeQString(double time)
{
    int hour = floor(time / 60.0 / 60.0);
    time -= hour * 60.0 * 60.0;
    int min = floor(time / 60.0);
    time -= min * 60.0;
    return QString("%2:%3")
        .arg(min, 2, 10, QLatin1Char('0'))
        .arg(time, 5, 'f', 2, QLatin1Char('0'));
}


boost::optional<double> JVRCManagerItem::startTimeCount()
{
    if(impl->simulatorItem){
        impl->startingTime = impl->simulatorItem->simulationTime();
        impl->saveRecords();
    }
    return impl->startingTime;
}


boost::optional<double> JVRCManagerItem::startingTime() const
{
    return impl->startingTime;
}


boost::optional<double> JVRCManagerItem::goalTime() const
{
    return impl->goalTime;
}


double JVRCManagerItem::elapsedTime(double simulationTime, bool isGoalTimeMax) const
{
    if(impl->goalTime && isGoalTimeMax){
        return *impl->goalTime;
    } else if(impl->startingTime){
        return std::max(0.0, simulationTime - *impl->startingTime + impl->offsetTime);
    } else {
        return impl->offsetTime;
    }
}


double JVRCManagerItemImpl::elapsedTime(bool isGoalTimeMax) const
{
    if(simulatorItem){
        return self->elapsedTime(simulatorItem->simulationTime(), isGoalTimeMax);
    } else {
        return self->elapsedTime(0.0, false);
    }
}


double JVRCManagerItem::remainingTime(double elapsedTime) const
{
    if(impl->currentTask){
        return std::max(0.0, impl->currentTask->timeLimit() - elapsedTime);
    } else {
        return std::max(0.0, 10.0 * 60.0 - elapsedTime);
    }
}
    

void JVRCManagerItemImpl::startRemainingTimeChecker()
{
    if(!filesystem::exists(remainingTimeOutputDirectory)){
        remainingTimeOutputDirectory.clear();
    }
    string filename = remainingTimeOutputDirectory + "jvrc-remaining-time.txt";
    remainingTimeOutputStream.open(filename.c_str());
    os << (format("Remaining time is output to \"%1%\".") % filename) << endl;
        
    lastRemainingTimeOutput = std::numeric_limits<double>::max();
    simulatorItem->addPreDynamicsFunction(checkRemainingTimeLater);
}


void JVRCManagerItemImpl::checkRemainingTime()
{
    double t = ceil(self->remainingTime(elapsedTime()));
    if(t != lastRemainingTimeOutput){
        if(t <= 0.0){
            saveRecords();
        }
        remainingTimeOutputStream.seekp(0);
        remainingTimeOutputStream << toTimeString(t) << endl;
        lastRemainingTimeOutput = t;
    }
}


void JVRCManagerItemImpl::stopRemainingTimeChecker()
{
    remainingTimeOutputStream.close();
}


void JVRCManagerItem::requestToAbort()
{
    impl->requestToAbort();
}


void JVRCManagerItemImpl::requestToAbort()
{
    if(simulatorItem){
        simulatorItem->pauseSimulation();
        if(showConfirmDialog("Abort", "Do you really want to abort the task?")){
            saveRecords();
            simulatorItem->stopSimulation();
            showMessageBox(format("The aborting time has been written to \"%1%\".")
                           % (recordFileNameBase + ".yaml"));
        } else {
            simulatorItem->restartSimulation();
        }
    }
}


SignalProxy<void(bool isDoingSimulation)> JVRCManagerItem::sigSimulationStateChanged()
{
    return impl->sigSimulationStateChanged;
}


bool JVRCManagerItem::loadJVRCInfo(const std::string& filename)
{
    return impl->loadJVRCInfo(filename);
}


bool JVRCManagerItemImpl::loadJVRCInfo(const std::string& filename)
{
    tasks.clear();
    setCurrentTask(0);

    YAMLReader reader;
    MappingPtr info = reader.loadDocument(filename)->toMapping();
    const Listing& taskNodes = *info->findListing("tasks");
    if(taskNodes.isValid()){
        for(int i=0; i < taskNodes.size(); ++i){
            tasks.push_back(new JVRCTask(taskNodes[i].toMapping()));
        }
    }

    sigTasksUpdated();

    if(tasks.size() > 0){
        setCurrentTask(tasks[0]);
    }

    return true;
}


bool JVRCManagerItem::initializeSimulation(SimulatorItem* simulatorItem)
{
    return impl->initializeSimulation(simulatorItem);
}


bool JVRCManagerItemImpl::initializeSimulation(SimulatorItem* simulatorItem)
{
    startingTime = boost::none;
    goalTime = boost::none;
    
    this->simulatorItem = simulatorItem;
    timeStep = simulatorItem->worldTimeStep();

    simulatorConnections.disconnect();
    simulatorConnections.add(
        simulatorItem->sigSimulationStarted().connect(
            boost::bind(boost::ref(sigSimulationStateChanged), true)));

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

    SimulationBody* simHose = simulatorItem->findSimulationBody("hose");

    if(simHose){
        SimulationBody* simNozzle = simulatorItem->findSimulationBody("nozzle");
        if(simNozzle){
            hoseEndLink = simHose->body()->link("hinge_L72WD2L72ND");
            nozzleLink = simNozzle->body()->rootLink();
            if(hoseEndLink && nozzleLink){
                os << "The hose and nozzle of the task R6 have been detected." << endl;
                simulatorItem->addPreDynamicsFunction(
                    boost::bind(&JVRCManagerItemImpl::checkConnectionBetweenHoseAndNozzle, this));
            }
        }
    }

    resetRecordFileName();

    if(START_COMPETITION_AT_STARTING_SIMULATION){
        self->startTimeCount();
    }

    startRemainingTimeChecker();

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
                    hitTimeCount = 0.0;
                    hitIndex = -1;
                    //isDoorDestroyed = false;
                    simDoor->setActive(true);
                    spreaderHitMarker->setRadius(minMarkerRadius);
                    os << "The spreader and the car door of Task R3A have been detected." << endl;
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
    if(p.z() < g1.z()){
        return 0;
    }
    p.z() = g1.z();
    
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
                    os << "Gate " << gate->gateIndex() << " has been passed." << endl;
                    callLater(boost::bind(&JVRCManagerItemImpl::addRecord, this, gate, elapsedTime(), false));
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
                    hitTimeCount = 0.0;
                    hitIndex = i;
                }
                break;
            }
        }
    }

    static const double requiredTimeForDestruction = 3.0;

    bool changed = false;
    if(isHitting){
        if(!doorDestroyFlags[hitIndex]){
            hitTimeCount += timeStep;
            if(hitTimeCount >= requiredTimeForDestruction){
                doorDestroyFlags[hitIndex] = true;
                hitTimeCount = 0.0;

                JVRCEvent* event = new JVRCEvent("internal", currentTask);
                event->setLabel(str(format("Cut %1%") % hitIndex));
                callLater(boost::bind(&JVRCManagerItemImpl::addRecord, this, event, elapsedTime(), false));
                    
                if(doorDestroyFlags.count() == doorDestroyFlags.size()){
                    doorRoot->T().translation().z() -= 2.0;
                    if(currentTask){
                        JVRCEvent* event = currentTask->findAction("Remove Door");
                        if(event){
                            callLater(boost::bind(&JVRCManagerItemImpl::addRecord, this, event, elapsedTime(), false));
                        }
                    }
                }
            }
        }
        
        static const double deltaRadius =
            9.0 * (maxMarkerRadius - minMarkerRadius) / (requiredTimeForDestruction / timeStep);
        double r = spreaderHitMarker->radius() + deltaRadius;
        if(r > maxMarkerRadius){
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
        changed = true;
    }
    if(changed){
        spreaderHitMarker->notifyStateChange();
    }
}


void JVRCManagerItemImpl::checkConnectionBetweenHoseAndNozzle()
{
    //cout << "JVRCManagerItemImpl::checkConnectionBetweenHoseAndNozzle()" << endl;
    
    static const Vector3 hoseEndPos(0.0, 0.0, -0.2);
    static const Vector3 forceOnHose(0.0, 0.0, -10.0); // +200
    static const Vector3 nozzleEndPos(0.540, 6.893, 2.200);
    static const Vector3 forceOnNozzle(0.0, 10.0, 0.0); // -200

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
    saveRecords();
    stopRemainingTimeChecker();
    sigSimulationStateChanged(false);
    simulatorConnections.disconnect();
    simulatorItem = 0;
}


void JVRCManagerItem::doPutProperties(PutPropertyFunction& putProperty)
{
    SubSimulatorItem::doPutProperties(putProperty);
    impl->doPutProperties(putProperty);
}


void JVRCManagerItemImpl::doPutProperties(PutPropertyFunction& putProperty)
{

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
