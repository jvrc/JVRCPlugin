/**
   @author Shin'ichiro Nakaoka
*/

#include "JVRCScoreView.h"
#include "JVRCManagerItem.h"
#include <cnoid/BodyItem>
#include <cnoid/ViewManager>
#include <cnoid/TimeBar>
#include <cnoid/Button>
#include <cnoid/MessageView>
#include <QBoxLayout>
#include <QFormLayout>
#include <QLabel>
#include <QTableWidget>
#include <QHeaderView>
#include <QKeyEvent>
#include <boost/bind.hpp>
#include <set>
#include <cmath>
#include <iostream>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using namespace boost;

namespace {

bool TRACE_FUNCTIONS = false;

bool ENABLE_START_BUTTON = false;

class RecordTableWidget : public QTableWidget
{
public:
    RecordTableWidget(JVRCScoreViewImpl* impl) : scoreViewImpl(impl) { }
    virtual void keyPressEvent(QKeyEvent* event);

    JVRCScoreViewImpl* scoreViewImpl;
};

class RecordItem : public QTableWidgetItem
{
public:
    int index;
    RecordItem(int index, const QString& text, bool isSelectable = false)
        : index(index),
          QTableWidgetItem(text)
   {
       if(isSelectable){
           setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
       } else {
           setFlags(Qt::ItemIsEnabled);
       }
        setTextAlignment(Qt::AlignCenter);
    }
};

}

namespace cnoid {

class JVRCScoreViewImpl
{
public:
    JVRCScoreViewImpl(JVRCScoreView* self);
    ~JVRCScoreViewImpl();

    JVRCManagerItem* manager;
    BodyItem* robotItem;
    ScopedConnection robotConnection;
    int currentTaskIndex;
    TimeBar* timeBar;
    double startTime;
    double timeLimit;
    
    QLabel scoreLabel;
    QLabel timeLabel;
    QLabel remainingTimeLabel;
    QLabel positionLabel;
    QLabel taskLabel;
    
    PushButton prevButton;
    PushButton nextButton;
    QVBoxLayout buttonVBox;
    QHBoxLayout buttonHBox1;
    QHBoxLayout buttonHBox2;
    QSpacerItem* buttonVBoxSpacer;
    vector<PushButton*> buttons;
    PushButton startButton;
    PushButton loadButton;
    PushButton abortButton;

    RecordTableWidget recordTable;
    int noColumn;
    int taskColumn;
    int eventColumn;
    int autoTimeColumn;
    int adoptDetectedTimeButtonColumn;
    int manualTimeColumn;
    int pointColumn;
    int numColumns;

    bool onTimeChanged(double time);
    void updateRobot();
    void onRobotPositionChanged();
    void onNextOrPrevButtonClicked(int direction);
    void updateTasks();
    void setCurrentTask(int taskIndex);
    void onSimulationStateChanged(bool isDoingSimulation);
    void onStartButtonClicked();
    void onEventButtonClicked(int index);
    void onRecordUpdated();
    void onAdoptDetectedTimeButtonClicked(int recordIndex);
    void deleteSelectedTableItems();
};

}


void RecordTableWidget::keyPressEvent(QKeyEvent* event)
{
    bool handled = false;
    
    switch(event->key()){

    case Qt::Key_Delete:
        scoreViewImpl->deleteSelectedTableItems();
        handled = true;
        break;

    defaut:
        break;
    }

    if(!handled){
        QTableWidget::keyPressEvent(event);
    }
}


void JVRCScoreView::initializeClass(ExtensionManager* ext)
{
    ext->viewManager().registerClass<JVRCScoreView>(
        "JVRCScoreView", N_("JVRC Score"), ViewManager::SINGLE_OPTIONAL);
}


JVRCScoreView::JVRCScoreView()
{
    impl = new JVRCScoreViewImpl(this);
}


JVRCScoreViewImpl::JVRCScoreViewImpl(JVRCScoreView* self)
    : recordTable(this)
{
    manager = JVRCManagerItem::instance();

    self->setDefaultLayoutArea(View::LEFT_BOTTOM);
    self->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    
    QVBoxLayout* vbox = new QVBoxLayout();
    QLabel* label;
    QFont font = scoreLabel.font();
    font.setPointSize(font.pointSize() + 6);

    QVBoxLayout* panelVBox = new QVBoxLayout();
    panelVBox->setContentsMargins(4, 4, 4, 0);

    QHBoxLayout* hbox = new QHBoxLayout;
    hbox->addStretch();
    
    QFormLayout* form = new QFormLayout;
    form->setRowWrapPolicy(QFormLayout::DontWrapRows);
    form->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
    form->setFormAlignment(Qt::AlignHCenter | Qt::AlignTop);
    form->setLabelAlignment(Qt::AlignLeft);

    label = new QLabel("Score:");
    label->setFont(font);
    scoreLabel.setFont(font);
    scoreLabel.setText("0");
    scoreLabel.setAlignment(Qt::AlignHCenter);
    form->addRow(label, &scoreLabel);

    label = new QLabel("Time:");
    label->setFont(font);
    timeLabel.setFont(font);
    timeLabel.setText("0:00.00");
    timeLabel.setAlignment(Qt::AlignHCenter);
    form->addRow(label, &timeLabel);
    
    label = new QLabel("Time Left:");
    label->setFont(font);
    remainingTimeLabel.setFont(font);
    remainingTimeLabel.setText("0:00.00");
    remainingTimeLabel.setAlignment(Qt::AlignHCenter);
    form->addRow(label, &remainingTimeLabel);
    
    label = new QLabel("Position:");
    positionLabel.setText("0.0, 0.0, 0.0");
    positionLabel.setAlignment(Qt::AlignHCenter);
    form->addRow(label, &positionLabel);

    hbox->addLayout(form);
    hbox->addStretch();
    panelVBox->addLayout(hbox);

    hbox = new QHBoxLayout();
    prevButton.setText("<");
    prevButton.setToolTip(_("Go back to the previous task"));
    prevButton.sigClicked().connect(boost::bind(&JVRCScoreViewImpl::onNextOrPrevButtonClicked, this, -1));
    hbox->addWidget(&prevButton, 0);

    taskLabel.setFrameStyle(QFrame::Box|QFrame::Sunken);
    taskLabel.setLineWidth(1);
    taskLabel.setMidLineWidth(1);
    taskLabel.setAlignment(Qt::AlignCenter);
    hbox->addWidget(&taskLabel, 1);

    nextButton.setText(">");
    nextButton.setToolTip(_("Skip to the next task"));
    nextButton.sigClicked().connect(boost::bind(&JVRCScoreViewImpl::onNextOrPrevButtonClicked, this, +1));
    hbox->addWidget(&nextButton, 0);
    panelVBox->addLayout(hbox);

    QFrame* frame = new QFrame;
    frame->setFrameStyle(QFrame::Box|QFrame::Sunken);
    buttonVBox.setContentsMargins(4, 4, 4, 4);
    buttonVBox.addLayout(&buttonHBox1);
    buttonVBox.addLayout(&buttonHBox2);
    buttonVBoxSpacer = new QSpacerItem(0, 0);
    buttonVBox.addSpacerItem(buttonVBoxSpacer);
    frame->setLayout(&buttonVBox);
    panelVBox->addWidget(frame);

    vbox->addLayout(panelVBox);

    recordTable.setSelectionBehavior(QAbstractItemView::SelectItems);
    recordTable.setSelectionMode(QAbstractItemView::ExtendedSelection);

    recordTable.verticalHeader()->hide();
    QHeaderView* header = recordTable.horizontalHeader();
    header->setMinimumSectionSize(24);

    noColumn = 0;
    taskColumn = 1;
    eventColumn = 2;
    autoTimeColumn = 3;
    adoptDetectedTimeButtonColumn = 4;
    manualTimeColumn = 5;
    pointColumn = 6;
    numColumns = 7;
    recordTable.setColumnCount(numColumns);

    recordTable.setHorizontalHeaderItem(noColumn, new QTableWidgetItem("No."));
    header->setResizeMode(noColumn, QHeaderView::ResizeToContents);

    recordTable.setHorizontalHeaderItem(taskColumn, new QTableWidgetItem("Task"));
    header->setResizeMode(taskColumn, QHeaderView::ResizeToContents);
    recordTable.setColumnHidden(taskColumn, true);
    
    recordTable.setHorizontalHeaderItem(eventColumn, new QTableWidgetItem("Event"));
    header->setResizeMode(eventColumn, QHeaderView::ResizeToContents);

    recordTable.setHorizontalHeaderItem(autoTimeColumn, new QTableWidgetItem("Detected Time"));
    header->setResizeMode(autoTimeColumn, QHeaderView::ResizeToContents);

    recordTable.setHorizontalHeaderItem(adoptDetectedTimeButtonColumn, new QTableWidgetItem(""));
    header->setResizeMode(adoptDetectedTimeButtonColumn, QHeaderView::ResizeToContents);
    
    recordTable.setHorizontalHeaderItem(manualTimeColumn, new QTableWidgetItem("Judged Time"));
    header->setResizeMode(manualTimeColumn, QHeaderView::ResizeToContents);

    recordTable.setHorizontalHeaderItem(pointColumn, new QTableWidgetItem("Point"));
    header->setResizeMode(pointColumn, QHeaderView::Stretch);
    
    vbox->addWidget(&recordTable);

    hbox = new QHBoxLayout;

    if(ENABLE_START_BUTTON){
        startButton.setText("Start");
        startButton.sigClicked().connect(
            boost::bind(&JVRCScoreViewImpl::onStartButtonClicked, this));
        hbox->addWidget(&startButton);
    }
    
    loadButton.setText("Load");
    loadButton.sigClicked().connect(
        boost::bind(&JVRCManagerItem::showDialogToLoadRecords, manager));
    hbox->addWidget(&loadButton);
    abortButton.setText("Abort");
    abortButton.sigClicked().connect(
        boost::bind(&JVRCManagerItem::requestToAbort, manager));
    hbox->addWidget(&abortButton);
    vbox->addLayout(hbox);
    
    self->setLayout(vbox);

    startTime = 0.0;
    timeLimit = 10.0 * 60.0;
    
    robotItem = 0;
    manager->sigRobotDetected().connect(
        boost::bind(&JVRCScoreViewImpl::updateRobot, this));
    updateRobot();

    manager->sigTasksUpdated().connect(
        boost::bind(&JVRCScoreViewImpl::updateTasks, this));
    updateTasks();

    manager->sigRecordUpdated().connect(
        boost::bind(&JVRCScoreViewImpl::onRecordUpdated, this));

    timeBar = TimeBar::instance();
    timeBar->sigTimeChanged().connect(
        boost::bind(&JVRCScoreViewImpl::onTimeChanged, this, _1));

    manager->sigSimulationStateChanged().connect(
        boost::bind(&JVRCScoreViewImpl::onSimulationStateChanged, this, _1));
    onSimulationStateChanged(false);
}


JVRCScoreView::~JVRCScoreView()
{
    delete impl;
}


JVRCScoreViewImpl::~JVRCScoreViewImpl()
{

}


bool JVRCScoreViewImpl::onTimeChanged(double time)
{
    double elapsedTime = manager->elapsedTime(time);
    timeLabel.setText(JVRCManagerItem::toTimeQString(elapsedTime));
    remainingTimeLabel.setText(JVRCManagerItem::toTimeQString(manager->remainingTime(elapsedTime)));
    return false;
}


void JVRCScoreViewImpl::updateRobot()
{
    robotConnection.disconnect();
    robotItem = manager->robotItem();
    if(robotItem){
        robotConnection.reset(
            robotItem->sigKinematicStateChanged().connect(
                boost::bind(&JVRCScoreViewImpl::onRobotPositionChanged, this)));
        onRobotPositionChanged();
    }
}


void JVRCScoreViewImpl::onRobotPositionChanged()
{
    const Vector3 p = manager->robotMarkerPosition().translation();
    positionLabel.setText(QString("%1, %2, %3").arg(p.x(), 5, 'f', 2).arg(p.y(), 5, 'f', 2).arg(p.z(), 5, 'f', 2));
}


void JVRCScoreViewImpl::onNextOrPrevButtonClicked(int direction)
{
    int taskIndex = std::max(0, std::min(currentTaskIndex + direction, manager->numTasks() - 1));
    setCurrentTask(taskIndex);
}


void JVRCScoreViewImpl::updateTasks()
{
    setCurrentTask(0);
}


void JVRCScoreViewImpl::setCurrentTask(int taskIndex)
{
    for(size_t i=0; i < buttons.size(); ++i){
        delete buttons[i];
    }
    buttons.clear();

    bool isButtonBox2Used = false;
    
    if(taskIndex >= manager->numTasks()){
        taskLabel.setText("");
        currentTaskIndex = 0;

    } else {
        JVRCTask* task = manager->task(taskIndex);
        taskLabel.setText(QString("Task ") + task->label().c_str());
        timeLimit = task->timeLimit();
        const int n = task->numEvents();
        for(int i=0; i < n; ++i){
            JVRCEvent* event = task->event(i);
            PushButton* button = new PushButton(event->label().c_str());
            button->sigClicked().connect(
                boost::bind(&JVRCScoreViewImpl::onEventButtonClicked, this, i));
            if(event->level() == 0){
                buttonHBox1.addWidget(button);
            } else {
                buttonHBox2.addWidget(button);
                isButtonBox2Used = true;
            }
            buttons.push_back(button);
        }
        currentTaskIndex = taskIndex;
    }

    if(isButtonBox2Used){
        buttonVBoxSpacer->changeSize(0, 0);
    } else {
        if(!buttons.empty()){
            int h = buttons.front()->sizeHint().height() + buttonVBox.spacing();
            buttonVBoxSpacer->changeSize(0, h);
        }
    }

    prevButton.setEnabled(currentTaskIndex > 0);
    nextButton.setEnabled(currentTaskIndex < manager->numTasks() - 1);
}


void JVRCScoreViewImpl::onSimulationStateChanged(bool isDoingSimulation)
{
    startButton.setEnabled(isDoingSimulation);
    loadButton.setEnabled(!isDoingSimulation);
    abortButton.setEnabled(isDoingSimulation);
}


void JVRCScoreViewImpl::onStartButtonClicked()
{
    if(manager->startTimeCount()){
        startButton.setEnabled(false);
    }
}


void JVRCScoreViewImpl::onEventButtonClicked(int index)
{
    JVRCTask* task = manager->task(currentTaskIndex);
    double elapsedTime = manager->elapsedTime(timeBar->time());
    manager->addRecord(task->event(index), elapsedTime);

    if(currentTaskIndex + 1 < manager->numTasks()){
        setCurrentTask(currentTaskIndex + 1);
    }
}


void JVRCScoreViewImpl::onRecordUpdated()
{
    scoreLabel.setText(QString("%1").arg(manager->score(), 2, 10));
    
    const int n = manager->numRecords();
    recordTable.setSortingEnabled(false);
    recordTable.setRowCount(0);

    for(int index=0; index < n; ++index){
        JVRCEvent* record = manager->record(index);
        if(record->isTimeRecorded()){
            recordTable.insertRow(0);
            int row = 0;
            QString no = QString("%1").arg(index, 2, 10, QLatin1Char('0'));
            recordTable.setItem(row, noColumn, new RecordItem(index, no, true));
            recordTable.setItem(row, taskColumn, new RecordItem(index, record->task()->name().c_str()));
            recordTable.setItem(row, eventColumn, new RecordItem(index, record->label().c_str()));
            
            if(record->detectedTime()){
                QString timestr = JVRCManagerItem::toTimeQString(*record->detectedTime());
                recordTable.setItem(row, autoTimeColumn, new RecordItem(index, timestr));
                ToolButton* button = new ToolButton;
                button->setText(">>");
                button->sigClicked().connect(
                    boost::bind(&JVRCScoreViewImpl::onAdoptDetectedTimeButtonClicked, this, index));
                recordTable.setCellWidget(row, adoptDetectedTimeButtonColumn, button);
            } else {
                recordTable.setItem(row, autoTimeColumn, new RecordItem(index, ""));
                recordTable.setItem(row, adoptDetectedTimeButtonColumn, new RecordItem(index, ""));
            }
            RecordItem* pointItem = 0;
            if(record->judgedTime()){
                QString timestr = JVRCManagerItem::toTimeQString(*record->judgedTime());
                recordTable.setItem(row, manualTimeColumn, new RecordItem(index, timestr, true));
                if(record->point() > 0){
                    pointItem = new RecordItem(index, QString("%1").arg(record->point()));
                }
            } else {
                recordTable.setItem(row, manualTimeColumn, new RecordItem(index, ""));
            }

            if(!pointItem){
                pointItem = new RecordItem(index, "");
            }
            recordTable.setItem(row, pointColumn, pointItem);
        }
    }

    recordTable.setSortingEnabled(true);

    onTimeChanged(TimeBar::instance()->time());
}


void JVRCScoreViewImpl::onAdoptDetectedTimeButtonClicked(int recordIndex)
{
    JVRCEvent* record = manager->record(recordIndex);
    if(record && record->detectedTime()){
        record->setJudgedTime(*record->detectedTime());
        manager->notifyRecordUpdate();
    }
}


void JVRCScoreViewImpl::deleteSelectedTableItems()
{
    set<int> rows;
    QList<QTableWidgetItem*> selected = recordTable.selectedItems();
    for(int i=0; i < selected.size(); ++i){
        if(RecordItem* recordItem = dynamic_cast<RecordItem*>(selected[i])){
            if(JVRCEvent* record = manager->record(recordItem->index)){
                int column = recordItem->column();
                if(column == noColumn){
                    record->clearTimes();
                } else if(column == manualTimeColumn){
                    record->clearManualRecordTime();
                }
            }
        }
    }
    manager->notifyRecordUpdate();
}
