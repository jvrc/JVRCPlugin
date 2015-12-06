/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_JVRC_PLUGIN_JVRC_MANAGER_ITEM_H
#define CNOID_JVRC_PLUGIN_JVRC_MANAGER_ITEM_H

#include "JVRCTask.h"
#include <cnoid/SubSimulatorItem>
#include <cnoid/BodyItem>
#include <QString>
#include "exportdecl.h"

namespace cnoid {

class JVRCManagerItemImpl;

class CNOID_EXPORT JVRCManagerItem : public SubSimulatorItem
{
public:
    static void initializeClass(ExtensionManager* ext);
    static JVRCManagerItem* instance();
        
    JVRCManagerItem();
    JVRCManagerItem(const JVRCManagerItem& org);
    ~JVRCManagerItem();

    bool loadJVRCInfo(const std::string& filename);

    int numTasks() const;
    JVRCTask* task(int index);
    JVRCTask* findTask(const std::string& name);
    SignalProxy<void()> sigTasksUpdated();
    Position startingPosition() const;
    
    BodyItem* robotItem();
    Position robotMarkerPosition() const;
    SignalProxy<void()> sigRobotDetected();

    JVRCTask* currentTask();
    SignalProxy<void()> sigCurrentTaskChanged();

    boost::optional<double> startTimeCount();
    boost::optional<double> startingTime() const;
    boost::optional<double> goalTime() const;
    double elapsedTime(double simulationTime, bool isGoalTimeMax = true) const;
    double remainingTime(double elapsedTime) const;
    static std::string toTimeString(double time, int formatType = 0);
    static QString toTimeQString(double time);

    void requestToAbort();
    
    SignalProxy<void(bool isDoingSimulation)> sigSimulationStateChanged();

    void clearRecords(bool doNotify);
    bool showDialogToLoadRecords();
    bool loadRecords(const std::string& filename);
    int numRecords() const;
    JVRCEvent* record(int index);
    void addRecord(JVRCEvent* event, double time, bool isJudged = true);
    void undoRecordEditing();
    void redoRecordEditing();
    SignalProxy<void()> sigRecordUpdated();
    void notifyRecordUpdate();
    
    int score() const;

    virtual bool initializeSimulation(SimulatorItem* simulatorItem);
    virtual void finalizeSimulation();

protected:
    virtual Item* doDuplicate() const;
    virtual void onPositionChanged();
    virtual void doPutProperties(PutPropertyFunction& putProperty);
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);

private:
    JVRCManagerItemImpl* impl;
};

typedef ref_ptr<JVRCManagerItem> JVRCManagerItemPtr;

}

#endif
