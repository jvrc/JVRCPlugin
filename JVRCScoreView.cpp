/**
   @author Shin'ichiro Nakaoka
*/

#include "JVRCScoreView.h"
#include "JVRCManagerItem.h"
#include <cnoid/BodyItem>
#include <cnoid/ViewManager>
#include <cnoid/TimeBar>
#include <cnoid/Button>
#include <QBoxLayout>
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

QString toTimeString(double time)
{
    int hour = floor(time / 60.0 / 60.0);
    time -= hour * 60.0 * 60.0;
    int min = floor(time / 60.0);
    time -= min * 60.0;
    return QString("%1:%2:%3")
        .arg(hour, 2, 10, QLatin1Char('0'))
        .arg(min, 2, 10, QLatin1Char('0'))
        .arg(time, 5, 'f', 2, QLatin1Char('0'));
}


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
    
    QLabel scoreLabel;
    QLabel timeLabel;
    QLabel positionLabel;
    QLabel taskLabel;
    
    PushButton startButton;
    PushButton prevButton;
    PushButton nextButton;
    QVBoxLayout buttonVBox;
    QHBoxLayout buttonHBox1;
    QHBoxLayout buttonHBox2;
    QSpacerItem* buttonVBoxSpacer;
    vector<PushButton*> buttons;
    PushButton resetButton;
    PushButton giveupButton;
    PushButton restartButton;

    RecordTableWidget recordTable;
    int noColumn;
    int taskColumn;
    int eventColumn;
    int autoTimeColumn;
    int adoptAutoTimeButtonColumn;
    int manualTimeColumn;
    int numColumns;

    double currentTime() const;
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
    void onAdoptAutoTimeButtonClicked(int recordIndex);
    void clearSelectedManualTimes();
};

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
    self->setDefaultLayoutArea(View::LEFT_BOTTOM);
    self->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    
    QVBoxLayout* vbox = new QVBoxLayout();
    QLabel* label;
    QFont font = scoreLabel.font();
    font.setPointSize(font.pointSize() + 6);

    QVBoxLayout* panelVBox = new QVBoxLayout();
    panelVBox->setContentsMargins(4, 4, 4, 0);

    QHBoxLayout* hbox = new QHBoxLayout();
    label = new QLabel("Score: ");
    label->setFont(font);
    hbox->addWidget(label);
    scoreLabel.setFont(font);
    scoreLabel.setText("0");
    hbox->addWidget(&scoreLabel);
    hbox->addStretch();
    panelVBox->addLayout(hbox);

    hbox = new QHBoxLayout();
    label = new QLabel("Time: ");
    label->setFont(font);
    hbox->addWidget(label);
    timeLabel.setFont(font);
    timeLabel.setText("0:00.00");
    hbox->addWidget(&timeLabel);
    hbox->addStretch();
    panelVBox->addLayout(hbox);

    hbox = new QHBoxLayout();
    label = new QLabel("Position: ");
    hbox->addWidget(label);
    positionLabel.setText("0.0, 0.0, 0.0");
    hbox->addWidget(&positionLabel);
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
    //recordTable.setSelectionBehavior(QAbstractItemView::SelectRows);
    recordTable.setSelectionMode(QAbstractItemView::SingleSelection);
    //recordTable.setSelectionMode(QAbstractItemView::ExtendedSelection);

    noColumn = 0;
    taskColumn = 1;
    eventColumn = 2;
    autoTimeColumn = 3;
    adoptAutoTimeButtonColumn = 4;
    manualTimeColumn = 5;
    numColumns = 6;
    recordTable.setColumnCount(numColumns);
    recordTable.setHorizontalHeaderItem(noColumn, new QTableWidgetItem("No."));
    recordTable.setHorizontalHeaderItem(taskColumn, new QTableWidgetItem("Task"));
    recordTable.setHorizontalHeaderItem(eventColumn, new QTableWidgetItem("Event"));
    recordTable.setHorizontalHeaderItem(autoTimeColumn, new QTableWidgetItem("Detected Time"));
    recordTable.setHorizontalHeaderItem(adoptAutoTimeButtonColumn, new QTableWidgetItem(""));
    recordTable.setHorizontalHeaderItem(manualTimeColumn, new QTableWidgetItem("Judged Time"));

    recordTable.verticalHeader()->hide();
    QHeaderView* hh = recordTable.horizontalHeader();
    hh->setResizeMode(QHeaderView::ResizeToContents);
    hh->setStretchLastSection(true);
    
    vbox->addWidget(&recordTable);

    hbox = new QHBoxLayout;
    startButton.setText("Start");
    startButton.sigClicked().connect(
        boost::bind(&JVRCScoreViewImpl::onStartButtonClicked, this));
    hbox->addWidget(&startButton);
    resetButton.setText("Reset");
    hbox->addWidget(&resetButton);
    restartButton.setText("Restart");
    hbox->addWidget(&restartButton);
    giveupButton.setText("Give Up");
    hbox->addWidget(&giveupButton);
    vbox->addLayout(hbox);
    
    self->setLayout(vbox);

    manager = JVRCManagerItem::instance();

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


double JVRCScoreViewImpl::currentTime() const
{
    double t = 0.0;
    optional<double> s = manager->startingTime();
    if(s){
        t = (*s > 0.0) ? timeBar->time() - *s : 0.0;
        if(t < 0.0){
            t = 0.0;
        }
    }
    return t;
}    
    

bool JVRCScoreViewImpl::onTimeChanged(double time)
{
    timeLabel.setText(toTimeString(currentTime()));
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
        taskLabel.setText(QString("Task ") + task->name().c_str());
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
    resetButton.setEnabled(isDoingSimulation);
    restartButton.setEnabled(!isDoingSimulation);
    giveupButton.setEnabled(isDoingSimulation);
}


void JVRCScoreViewImpl::onStartButtonClicked()
{
    if(manager->startTimer()){
        startButton.setEnabled(false);
    }
}


void JVRCScoreViewImpl::onEventButtonClicked(int index)
{
    JVRCTask* task = manager->task(currentTaskIndex);
    manager->addRecord(task->event(index), currentTime());

    if(currentTaskIndex + 1 < manager->numTasks()){
        setCurrentTask(currentTaskIndex + 1);
    }
}


void JVRCScoreViewImpl::onRecordUpdated()
{
    const int n = manager->numRecords();
    recordTable.setSortingEnabled(false);
    recordTable.setRowCount(0);

    for(int index=0; index < n; ++index){
        JVRCEvent* record = manager->record(index);
        if(record->isTimeRecorded()){
            recordTable.insertRow(0);
            int row = 0;
            QString no = QString("%1").arg(index, 2, 10, QLatin1Char('0'));
            recordTable.setItem(row, noColumn, new RecordItem(index, no));
            recordTable.setItem(row, taskColumn, new RecordItem(index, record->task()->name().c_str()));
            recordTable.setItem(row, eventColumn, new RecordItem(index, record->label().c_str()));
            
            if(record->automaticRecordTime()){
                recordTable.setItem(row, autoTimeColumn, new RecordItem(index, toTimeString(*record->automaticRecordTime())));
                ToolButton* button = new ToolButton;
                button->setText("->");
                button->sigClicked().connect(
                    boost::bind(&JVRCScoreViewImpl::onAdoptAutoTimeButtonClicked, this, index));
                recordTable.setCellWidget(row, adoptAutoTimeButtonColumn, button);
            }
            if(record->manualRecordTime()){
                recordTable.setItem(row, manualTimeColumn, new RecordItem(index, toTimeString(*record->manualRecordTime()), true));
            } else {
                recordTable.setItem(row, manualTimeColumn, new RecordItem(index, ""));
            }
        }
    }

    recordTable.setSortingEnabled(true);
}


void JVRCScoreViewImpl::onAdoptAutoTimeButtonClicked(int recordIndex)
{
    JVRCEvent* record = manager->record(recordIndex);
    if(record && record->automaticRecordTime()){
        record->setManualRecordTime(*record->automaticRecordTime());
        manager->notifyRecordUpdate();
    }
}


void JVRCScoreViewImpl::clearSelectedManualTimes()
{
    set<int> rows;
    QList<QTableWidgetItem*> selected = recordTable.selectedItems();
    for(int i=0; i < selected.size(); ++i){
        RecordItem* recordItem = dynamic_cast<RecordItem*>(selected[i]);
        if(recordItem){
            JVRCEvent* record = manager->record(recordItem->index);
            if(record){
                record->clearManualRecordTime();
            }
        }
    }
    manager->notifyRecordUpdate();
}


void RecordTableWidget::keyPressEvent(QKeyEvent* event)
{
    bool handled = false;
    
    switch(event->key()){

    case Qt::Key_Delete:
        scoreViewImpl->clearSelectedManualTimes();
        handled = true;
        break;

    defaut:
        break;
    }

    if(!handled){
        QTableWidget::keyPressEvent(event);
    }
}
