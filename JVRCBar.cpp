/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "JVRCBar.h"
#include <cnoid/Button>
#include <cnoid/SpinBox>
#include <boost/bind.hpp>

using namespace boost;
using namespace cnoid;

namespace cnoid {

class JVRCBarImpl
{
public:
    SpinBox pointSpin;
    PushButton pointButton;
    JVRCBarImpl(JVRCBar* self);
};

}


namespace {

}
        

JVRCBar::JVRCBar()
    : ToolBar("JVRCBar")
{
    impl = new JVRCBarImpl(this);
}


JVRCBar::~JVRCBar()
{

}

    
JVRCBarImpl::JVRCBarImpl(JVRCBar* self)
{

}
