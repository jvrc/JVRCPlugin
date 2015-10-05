/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_JVRC_PLUGIN_JVRC_SCORE_VIEW_H
#define CNOID_JVRC_PLUGIN_JVRC_SCORE_VIEW_H

#include <cnoid/View>

namespace cnoid {

class JVRCScoreViewImpl;

class JVRCScoreView : public View
{
public:
    static void initializeClass(ExtensionManager* ext);
        
    JVRCScoreView();
    ~JVRCScoreView();

protected:
    virtual void keyPressEvent(QKeyEvent* event);

private:
    JVRCScoreViewImpl* impl;
};

}

#endif
