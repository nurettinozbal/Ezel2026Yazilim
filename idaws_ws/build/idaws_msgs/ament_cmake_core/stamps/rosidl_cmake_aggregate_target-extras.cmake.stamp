# generated from rosidl_cmake/cmake/rosidl_cmake_aggregate_target-extras.cmake.in

# Create a convenience aggregate target idaws_msgs::idaws_msgs
# that links all generated interface targets, so downstream packages can use
# a single modern CMake target name instead of ${idaws_msgs_TARGETS}.
if(idaws_msgs_TARGETS AND NOT TARGET idaws_msgs::idaws_msgs)
  add_library(idaws_msgs::idaws_msgs INTERFACE IMPORTED)
  set_target_properties(idaws_msgs::idaws_msgs PROPERTIES
    INTERFACE_LINK_LIBRARIES "${idaws_msgs_TARGETS}")
endif()
