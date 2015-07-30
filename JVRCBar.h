/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_JVRC_PLUGIN_JVRC_BAR_H
#define CNOID_JVRC_PLUGIN_JVRC_BAR_H

#include <cnoid/ToolBar>

namespace cnoid {

class JVRCBarImpl;
    
class JVRCBar : public ToolBar
{
public:
    JVRCBar();
    ~JVRCBar();

private:
    JVRCBarImpl* impl;
};

}

#endif
