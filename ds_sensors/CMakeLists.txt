cmake_minimum_required(VERSION 2.8.3)
project(ds_sensors)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

list(INSERT CMAKE_MODULE_PATH 0 ${PROJECT_SOURCE_DIR}/cmake)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  ds_base
  ds_util
  ds_hotel_msgs
  ds_sensor_msgs
  ds_core_msgs
  ds_nmea_msgs
  ds_nmea_parsers
  cmake_modules
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS program_options)

## Eigen geometry library
find_package(Eigen3 REQUIRED)

find_package(ClangFormat)
if(CLANG_FORMAT_FOUND)
  add_custom_target(clang-format-ds-sensors
    COMMENT
    "Run clang-format on all project C++ sources"
    WORKING_DIRECTORY
    ${PROJECT_SOURCE_DIR}
    COMMAND
    find src
    find src/ds_sensors
    include/ds_sensors
    -iname '*.h' -o -iname '*.cpp'
    | xargs ${CLANG_FORMAT_EXECUTABLE} -i
    )

endif(CLANG_FORMAT_FOUND)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
#add_message_files(DIRECTORY msg
#  FILES
#  PhinsStatus.msg
#)

## Generate services in the 'srv' folder
# add_service_files(DIRECTORY srv
#   FILES
#   StringCommand.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )
#
## Generate added messages and services with any dependencies listed here
#generate_messages(
#    DEPENDENCIES
#    ds_core_msgs
#    ds_nmea_msgs
#    std_msgs
#    ds_sensor_msgs
#)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ds_sensors
   CATKIN_DEPENDS ds_base ds_hotel_msgs ds_sensor_msgs message_generation
   DEPENDS EIGEN3
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

## Declare a C++ library
add_library(${PROJECT_NAME}

  include/${PROJECT_NAME}/ds_sensors.h
  include/${PROJECT_NAME}/depthsensor.h
  src/${PROJECT_NAME}/depthsensor.cpp
  src/${PROJECT_NAME}/depthsensor_private.h

  include/${PROJECT_NAME}/sbe49.h
  src/${PROJECT_NAME}/sbe49.cpp
  src/${PROJECT_NAME}/sbe49_private.h

  include/${PROJECT_NAME}/aps1540.h
  src/${PROJECT_NAME}/aps1540.cpp
  src/${PROJECT_NAME}/aps1540_private.h

  include/${PROJECT_NAME}/parodigiquartz.h
  src/${PROJECT_NAME}/parodigiquartz.cpp
  src/${PROJECT_NAME}/parodigiquartz_private.h

  include/${PROJECT_NAME}/isd4000.h
  src/${PROJECT_NAME}/isd4000.cpp

  include/${PROJECT_NAME}/resonsvp70.h
  src/${PROJECT_NAME}/resonsvp70.cpp
  src/${PROJECT_NAME}/resonsvp70_private.h

  include/${PROJECT_NAME}/rdidvl.h
  include/${PROJECT_NAME}/rdidvl_header_match.h
  src/${PROJECT_NAME}/rdidvl.cpp
  src/${PROJECT_NAME}/rdidvl_private.h
  src/${PROJECT_NAME}/rdidvl_structs.h

  include/${PROJECT_NAME}/nortekdvl1000.h
  src/${PROJECT_NAME}/nortekdvl1000.cpp
  src/${PROJECT_NAME}/nortekdvl1000_structs.h

  include/${PROJECT_NAME}/anderaaoxyoptode.h
  src/${PROJECT_NAME}/anderaaoxyoptode.cpp
  src/${PROJECT_NAME}/anderaaoxyoptode_private.h

  include/${PROJECT_NAME}/remushtpgl.h
  src/${PROJECT_NAME}/remushtpgl.cpp
  src/${PROJECT_NAME}/remushtpgl_private.h

  include/${PROJECT_NAME}/phins.h
  src/${PROJECT_NAME}/phins.cpp
  src/${PROJECT_NAME}/phins_private.h

  include/${PROJECT_NAME}/xeosgps.h
  src/${PROJECT_NAME}/xeosgps.cpp
  src/${PROJECT_NAME}/xeosgps_private.h

  include/${PROJECT_NAME}/nortekvector.h
  src/${PROJECT_NAME}/nortekvector.cpp
  src/${PROJECT_NAME}/nortekvector_private.h
  src/${PROJECT_NAME}/nortekvector_structs.h

  include/${PROJECT_NAME}/sbe03.h
  src/${PROJECT_NAME}/sbe03.cpp
  src/${PROJECT_NAME}/sbe03_private.h

  include/${PROJECT_NAME}/phinsbin.h
  src/${PROJECT_NAME}/phinsbin.cpp
  src/${PROJECT_NAME}/phinsbin_private.h
  src/${PROJECT_NAME}/phinstdv3.h

  include/${PROJECT_NAME}/microstrainahrs.h
  src/${PROJECT_NAME}/microstrainahrs.cpp
  src/${PROJECT_NAME}/microstrainahrs_private.h
  src/${PROJECT_NAME}/microstrain_structs.h

  include/${PROJECT_NAME}/sonardyne_usbl.h
  src/${PROJECT_NAME}/sonardyne_usbl.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(sensor_node
  src/sensor_node.cpp
  src/sensor_node_utils.cpp
  src/sensor_node_utils.h
)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
set_target_properties(sensor_node PROPERTIES OUTPUT_NAME sensor PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(sensor_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(sensor_node
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)

# test data spitter executable
add_executable(test_data_node
    src/test_data_spitter.h
    src/test_data_spitter.cpp
    src/test_data_spitter_node.cpp
)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(test_data_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(test_data_node
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
if(CATKIN_ENABLE_TESTING)
    catkin_add_gtest(test_sbe49_parsing src/test/test_sbe49_parsing.cpp)
    target_link_libraries(test_sbe49_parsing ${PROJECT_NAME})

    catkin_add_gtest(test_aps1540_parsing src/test/test_aps1540_parsing.cpp)
    target_link_libraries(test_aps1540_parsing ${PROJECT_NAME})

    catkin_add_gtest(test_remushtpgl_parsing src/test/test_remushtpgl_parsing.cpp)
    target_link_libraries(test_remushtpgl_parsing ${PROJECT_NAME})

    catkin_add_gtest(test_parodigiquartz_parsing src/test/test_parodigiquartz_parsing.cpp)
    target_link_libraries(test_parodigiquartz_parsing ${PROJECT_NAME})

    catkin_add_gtest(test_isd4000_parsing src/test/test_isd4000_parsing.cpp)
    target_link_libraries(test_isd4000_parsing ${PROJECT_NAME})

    catkin_add_gtest(test_resonsvp70_parsing src/test/test_resonsvp70_parsing.cpp)
    target_link_libraries(test_resonsvp70_parsing ${PROJECT_NAME})

    catkin_add_gtest(test_rdidvlparse
            src/test/test_rdidvl_parsing.cpp)
    target_link_libraries(test_rdidvlparse ${PROJECT_NAME})
#    catkin_add_gtest(test_rdidvl_parsing src/test/test_rdidvl_parsing.cpp)
#    target_link_libraries(test_rdidvl_parsing ${PROJECT_NAME})

    catkin_add_gtest(test_nortekdvl_parsing src/test/test_nortekdvl1000_parsing.cpp)
    target_link_libraries(test_nortekdvl_parsing ${PROJECT_NAME})

    catkin_add_gtest(test_anderaaoxyoptode_parsing src/test/test_anderaaoxyoptode_parsing.cpp)
    target_link_libraries(test_anderaaoxyoptode_parsing ${PROJECT_NAME})

    catkin_add_gtest(test_nortekvector_parsing src/test/test_nortekvector_parsing.cpp)
    target_link_libraries(test_nortekvector_parsing ${PROJECT_NAME})

    catkin_add_gtest(test_sbe03_parsing src/test/test_sbe03_parsing.cpp)
    target_link_libraries(test_sbe03_parsing ${PROJECT_NAME})

    catkin_add_gtest(test_sensor_node_argument_parsing
            src/test/test_sensor_node_argument_parsing.cpp
            src/sensor_node_utils.h
            src/sensor_node_utils.cpp
            )
    target_link_libraries(test_sensor_node_argument_parsing
            ${PROJECT_NAME}
            ${Boost_LIBRARIES})
endif(CATKIN_ENABLE_TESTING)
## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
