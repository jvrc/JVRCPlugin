/*!
  @author Shin'ichiro Nakaoka
*/

#include "../JVRCManagerItem.h"
#include <cnoid/PyUtil>

using namespace boost;
using namespace boost::python;
using namespace cnoid;

namespace {

JVRCManagerItemPtr JVRCManagerItem_instance()
{
    return JVRCManagerItem::instance();
}

}


BOOST_PYTHON_MODULE(JVRCPlugin)
{
    boost::python::import("cnoid.Base");

    class_<JVRCManagerItem, JVRCManagerItemPtr, bases<Item>, boost::noncopyable>("JVRCManagerItem", no_init)
        .def("instance", JVRCManagerItem_instance).staticmethod("instance")
        .def("startingPosition", &JVRCManagerItem::startingPosition)
        ;

    implicitly_convertible<JVRCManagerItemPtr, ItemPtr>();
}
