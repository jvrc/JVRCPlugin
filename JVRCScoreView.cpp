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
#include <boost/bind.hpp>
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
    virtual void keyPressEvent(QKeyEvent* event);
};

class EventItem : public QTableWidgetItem
{
public:
    EventItem(const QString& text) : QTableWidgetItem(text) {
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

    BodyItem* robotItem;
    ScopedConnection robotConnection;
    JVRCTaskInfoPtr taskInfo;
    int currentTaskIndex;

    QLabel scoreLabel;
    QLabel timeLabel;
    QLabel positionLabel;
    QLabel taskLabel;
    PushButton prevButton;
    PushButton nextButton;
    QHBoxLayout buttonBox1;
    QHBoxLayout buttonBox2;
    vector<PushButton*> buttons;
    EventListWidget eventList;

    bool onTimeChanged(double time);
    void updateRobot();
    void onRobotPositionChanged();
    void onNextOrPrevButtonClicked(int direction);
    void updateTasks();
    void setCurrentTask(int taskIndex);
    void onEventButtonClicked(int index);
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
{
    self->setDefaultLayoutArea(View::LEFT_BOTTOM);
    self->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    
    QVBoxLayout* vbox = new QVBoxLayout();

    QHBoxLayout* hbox = new QHBoxLayout();
    hbox->addWidget(new QLabel("Score: "));
    scoreLabel.setText("0");
    hbox->addWidget(&scoreLabel);
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout();
    hbox->addWidget(new QLabel("Time: "));
    timeLabel.setText("0:00.00");
    hbox->addWidget(&timeLabel);
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout();
    hbox->addWidget(new QLabel("Position: "));
    positionLabel.setText("0.0, 0.0, 0.0");
    hbox->addWidget(&positionLabel);
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout();
    prevButton.setText("<");
    prevButton.setToolTip(_("Go back to the previous task"));
    prevButton.sigClicked().connect(boost::bind(&JVRCScoreViewImpl::onNextOrPrevButtonClicked, this, -1));
    hbox->addWidget(&prevButton, 0);

    taskLabel.setAlignment(Qt::AlignCenter);
    hbox->addWidget(&taskLabel, 1);

    nextButton.setText(">");
    nextButton.setToolTip(_("Skip to the next task"));
    nextButton.sigClicked().connect(boost::bind(&JVRCScoreViewImpl::onNextOrPrevButtonClicked, this, +1));
    hbox->addWidget(&nextButton, 0);
    vbox->addLayout(hbox);

    vbox->addLayout(&buttonBox1);
    vbox->addLayout(&buttonBox2);

    eventList.setColumnCount(3);
    eventList.setSelectionBehavior(QAbstractItemView::SelectRows);
    eventList.setSelectionMode(QAbstractItemView::ExtendedSelection);
    eventList.setColumnCount(4);
    eventList.verticalHeader()->hide();
    QHeaderView* hh = eventList.horizontalHeader();
    hh->setResizeMode(QHeaderView::ResizeToContents);
    hh->setStretchLastSection(true);
    eventList.setHorizontalHeaderItem(0, new EventItem("No"));
    eventList.setHorizontalHeaderItem(1, new EventItem("Time"));
    eventList.setHorizontalHeaderItem(2, new EventItem("Task"));
    eventList.setHorizontalHeaderItem(3, new EventItem("Event"));
    vbox->addWidget(&eventList);
    self->setLayout(vbox);

    JVRCManagerItem* manager = JVRCManagerItem::instance();

    robotItem = 0;
    manager->sigRobotDetected().connect(
        boost::bind(&JVRCScoreViewImpl::updateRobot, this));
    updateRobot();

    taskInfo = manager->taskInfo();
    manager->sigTaskInfoUpdated().connect(
        boost::bind(&JVRCScoreViewImpl::updateTasks, this));
    updateTasks();

    TimeBar::instance()->sigTimeChanged().connect(
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
    timeLabel.setText(toTimeString(time));
    return false;
}


void JVRCScoreViewImpl::updateRobot()
{
    robotConnection.disconnect();
    robotItem = JVRCManagerItem::instance()->robotItem();
    if(robotItem){
        robotConnection.reset(
            robotItem->sigKinematicStateChanged().connect(
                boost::bind(&JVRCScoreViewImpl::onRobotPositionChanged, this)));
        onRobotPositionChanged();
    }
}


void JVRCScoreViewImpl::onRobotPositionChanged()
{
    Vector3 p = robotItem->body()->rootLink()->translation();
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
    
    if(taskIndex >= taskInfo->numTasks()){
        taskLabel.setText("");
        currentTaskIndex = 0;

    } else {
        JVRCTask* task = taskInfo->task(taskIndex);
        taskLabel.setText(task->name().c_str());
        const int n = task->numEvents();
        for(int i=0; i < n; ++i){
            JVRCEvent* event = task->event(i);
            PushButton* button = new PushButton(event->label().c_str());
            button->sigClicked().connect(
                boost::bind(&JVRCScoreViewImpl::onEventButtonClicked, this, i));
            if(event->level() == 0){
                buttonBox1.addWidget(button);
            } else {
                buttonBox2.addWidget(button);
            }
            buttons.push_back(button);
        }
        currentTaskIndex = taskIndex;
    }
}


void JVRCScoreViewImpl::onEventButtonClicked(int index)
{
    JVRCTask* task = taskInfo->task(currentTaskIndex);
    JVRCEvent* event = task->event(index);
    int eventIndex = eventList.rowCount();
    eventList.insertRow(0);
    eventList.setItem(0, 0, new EventItem(QString("%1").arg(eventIndex, 2, 10, QLatin1Char('0'))));
    eventList.setItem(0, 1, new EventItem(toTimeString(TimeBar::instance()->time())));
    eventList.setItem(0, 2, new EventItem(task->name().c_str()));
    eventList.setItem(0, 3, new EventItem(event->label().c_str()));

    if(event->type() == "goal"){
        if(currentTaskIndex + 1 < taskInfo->numTasks()){
            setCurrentTask(currentTaskIndex + 1);
        }
    }
}


void EventListWidget::keyPressEvent(QKeyEvent* event)
{
    QTableWidget::keyPressEvent(event);
}
