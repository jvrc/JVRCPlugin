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


class EventListWidget : public QTableWidget
{
public:
    EventListWidget(JVRCScoreViewImpl* impl) : scoreViewImpl(impl) { }
    virtual void keyPressEvent(QKeyEvent* event);

    JVRCScoreViewImpl* scoreViewImpl;
};

class EventItem : public QTableWidgetItem
{
public:
    JVRCEventPtr record;
    EventItem(const QString& text, JVRCEvent* record = 0) : QTableWidgetItem(text), record(record) {
        setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
        //setFlags(Qt::ItemIsEnabled I Qt::temIsSelectable | Qt::ItemIsEditable);
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
    TimeBar* timeBar;
    BodyItem* robotItem;
    ScopedConnection robotConnection;
    JVRCTaskInfoPtr taskInfo;
    int currentTaskIndex;
    double startTime;
    
    QLabel scoreLabel;
    QLabel timeLabel;
    QLabel positionLabel;
    QLabel taskLabel;
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

    EventListWidget eventList;
    int noColumn;
    int taskColumn;
    int eventColumn;
    int autoTimeColumn;
    int manualTimeColumn;
    int numColumns;

    bool onTimeChanged(double time);
    void updateRobot();
    void onRobotPositionChanged();
    void onNextOrPrevButtonClicked(int direction);
    void updateTasks();
    void setCurrentTask(int taskIndex);
    void onEventButtonClicked(int index);
    void onRecordsUpdated();
    void removeSelectedEvents();
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
    : eventList(this)
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

    eventList.setSelectionBehavior(QAbstractItemView::SelectRows);
    eventList.setSelectionMode(QAbstractItemView::ExtendedSelection);

    noColumn = 0;
    taskColumn = 1;
    eventColumn = 2;
    autoTimeColumn = 3;
    manualTimeColumn = 4;
    numColumns = 5;
    eventList.setColumnCount(numColumns);
    eventList.setHorizontalHeaderItem(noColumn, new EventItem("No."));
    eventList.setHorizontalHeaderItem(taskColumn, new EventItem("Task"));
    eventList.setHorizontalHeaderItem(eventColumn, new EventItem("Event"));
    eventList.setHorizontalHeaderItem(autoTimeColumn, new EventItem("Time (Auto)"));
    eventList.setHorizontalHeaderItem(manualTimeColumn, new EventItem("Time (Manual)"));

    eventList.verticalHeader()->hide();
    QHeaderView* hh = eventList.horizontalHeader();
    hh->setResizeMode(QHeaderView::ResizeToContents);
    hh->setStretchLastSection(true);
    
    vbox->addWidget(&eventList);

    hbox = new QHBoxLayout;
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

    taskInfo = manager->taskInfo();
    manager->sigTaskInfoUpdated().connect(
        boost::bind(&JVRCScoreViewImpl::updateTasks, this));
    updateTasks();

    manager->sigRecordsUpdated().connect(
        boost::bind(&JVRCScoreViewImpl::onRecordsUpdated, this));

    timeBar = TimeBar::instance();
    timeBar->sigTimeChanged().connect(
        boost::bind(&JVRCScoreViewImpl::onTimeChanged, this, _1));
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
    double s = manager->startingTime();
    double t = (s > 0.0) ? time - s : 0.0;
    timeLabel.setText(toTimeString(t));
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
    int taskIndex = std::max(0, std::min(currentTaskIndex + direction, taskInfo->numTasks() - 1));
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
    
    if(taskIndex >= taskInfo->numTasks()){
        taskLabel.setText("");
        currentTaskIndex = 0;

    } else {
        JVRCTask* task = taskInfo->task(taskIndex);
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
    nextButton.setEnabled(currentTaskIndex < taskInfo->numTasks() - 1);
}


void JVRCScoreViewImpl::onEventButtonClicked(int index)
{
    JVRCTask* task = taskInfo->task(currentTaskIndex);
    manager->recordEvent(task->event(index), timeBar->time());

    if(currentTaskIndex + 1 < taskInfo->numTasks()){
        setCurrentTask(currentTaskIndex + 1);
    }
}


void JVRCScoreViewImpl::onRecordsUpdated()
{
    const int n = manager->numRecords();
    eventList.setSortingEnabled(false);
    eventList.setRowCount(0);

    for(int index=0; index < n; ++index){
        JVRCEvent* record = manager->record(index);
        if(record->isTimeRecorded()){
            eventList.insertRow(0);
            int row = 0;
            eventList.setItem(row, noColumn, new EventItem(QString("%1").arg(index, 2, 10, QLatin1Char('0')), record));
            eventList.setItem(row, taskColumn, new EventItem(record->task()->name().c_str()));
            eventList.setItem(row, eventColumn, new EventItem(record->label().c_str()));
            
            if(record->automaticRecordTime()){
                eventList.setItem(row, autoTimeColumn, new EventItem(toTimeString(*record->automaticRecordTime())));
            }
            if(record->manualRecordTime()){
                eventList.setItem(row, manualTimeColumn, new EventItem(toTimeString(*record->manualRecordTime())));
            }
        }
    }

    eventList.setSortingEnabled(true);
}


void JVRCScoreViewImpl::removeSelectedEvents()
{
    set<int> rows;
    QList<QTableWidgetItem*> selected = eventList.selectedItems();
    for(int i=0; i < selected.size(); ++i){
        QTableWidgetItem* item = selected[i];
        rows.insert(item->row());
    }
    for(set<int>::reverse_iterator p = rows.rbegin(); p != rows.rend(); ++p){
        eventList.removeRow(*p);
    }
}


void EventListWidget::keyPressEvent(QKeyEvent* event)
{
    bool handled = false;
    
    switch(event->key()){

    case Qt::Key_Delete:
        scoreViewImpl->removeSelectedEvents();
        handled = true;
        break;

    defaut:
        break;
    }

    if(!handled){
        QTableWidget::keyPressEvent(event);
    }
}
