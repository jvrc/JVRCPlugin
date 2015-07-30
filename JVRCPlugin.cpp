/**
   @author Shin'ichiro Nakaoka
*/

#include "JVRCManagerItem.h"
#include "JVRCScoreView.h"
#include <cnoid/Plugin>

using namespace cnoid;

class JVRCPlugin : public Plugin
{
public:
    
    JVRCPlugin() : Plugin("JVRC") {
        require("Body");
    }
    
    virtual bool initialize() {
        JVRCManagerItem::initializeClass(this);
        JVRCScoreView::initializeClass(this);
        return true;
    }
};


CNOID_IMPLEMENT_PLUGIN_ENTRY(JVRCPlugin);
