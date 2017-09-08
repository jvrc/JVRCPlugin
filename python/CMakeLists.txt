
set(target PyJVRCPlugin)

add_cnoid_python_module(${target}
  PyJVRCPlugin.cpp
)

target_link_libraries(${target} CnoidJVRCPlugin)
