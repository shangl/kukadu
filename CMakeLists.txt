cmake_minimum_required(VERSION 2.8.3)
project(kukadu)

set(CMAKE_BUILD_TYPE Debug)

include(CheckCXXCompilerFlag)

if(${ENABLE_VISUALIZATION})
    add_definitions(-DVISUALIZATION=1)
else()
    add_definitions(-DVISUALIZATION=0)
endif()

find_package(cmake_modules REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    actionlib_msgs
    control_msgs
    eigen_conversions
    geometry_msgs
    iis_robot_dep
    message_generation
    message_runtime
    moveit_core
    moveit_msgs
    moveit_ros_planning_interface
    pcl_conversions
    pcl_ros
    roscpp
    sensor_msgs
    std_msgs
    tf
    tf2_msgs
    tf_conversions
    trajectory_msgs
    cv_bridge
)

find_package(PCL REQUIRED)
find_package(Boost 1.54 REQUIRED)
find_package(Armadillo REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(OpenCV REQUIRED)

if($ENV{ROS_DISTRO} STREQUAL "kinetic")
    set(PYTH_INC /usr/include/python3.5m)
    set(PYTH_LIB python3.5m)
    set(PYTH_LIB_PATH /usr/lib/python3.5/config-3.5m-x86_64-linux-gnu)
    add_definitions(-DROSKINETIC)
else()
    set(PYTH_INC /usr/include/python3.4m)
    set(PYTH_LIB python3.4m)
    set(PYTH_LIB_PATH /usr/lib/python3.4/config-3.4m-x86_64-linux-gnu)
    add_definitions(-DUSEOLD)
endif()

find_package(Qt5Gui REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(PkgConfig REQUIRED)

add_definitions(-fPIC)
add_definitions(-shared)
add_definitions(-pthread)
add_definitions(-DROS_SUPPORT)
add_definitions(-DOSNAME_LINUX=1)

#set(KukaduGuiLib_src ${PROJECT_SOURCE_DIR}/src/gui/gui.cpp)
#set(KukaduGuiLib_hdr ${PROJECT_SOURCE_DIR}/include/kukadu/gui/gui.hpp)

#qt5_wrap_cpp(KukaduGuiLib_hdr_moc ${KukaduGuiLib_hdr})

pkg_check_modules(GTK2 REQUIRED gtk+-2.0)
pkg_check_modules(GTK REQUIRED gtkglext-1.0)

generate_dynamic_reconfigure_options(
  cfg/arlocalizer/ARParam.cfg
)

catkin_package(
    INCLUDE_DIRS include external/ReflexxesTypeII/include external/komo/share/src ${PYTH_INC}
    LIBRARIES ${PROJECT_NAME} kukaduvision reflexxeslib artoolkitplus
    CATKIN_DEPENDS cmake_modules actionlib_msgs control_msgs control_msgs eigen_conversions
        geometry_msgs iis_robot_dep message_generation moveit_core moveit_msgs
        moveit_ros_planning_interface pcl_conversions pcl_ros roscpp sensor_msgs std_msgs
        tf tf2_msgs tf_conversions trajectory_msgs
    DEPENDS gsl gslcblas ncurses
        stdc++ ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${ARMADILLO_LIBRARIES} ${PYTH_LIB}
        komo swift algo gui ors rt lapack blas ann glut GLU GL X11 ${GTK_LIBRARIES} ${GTK2_LIBRARIES} gthread-2.0 qhull m mysqlcppconn
)

include_directories(
    include
    ${PYTH_INC}
    ${PCL_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    external/komo/share/src
    external/ReflexxesTypeII/include
)

set(CURSES_USE_NCURSES TRUE)

SET(AR_TRACKER_PARAM "6, 6, 6, 1, 16" CACHE STRING "Tracker parameter maker size x, y, sample size, input format, max load, max pattern" )
add_definitions(-DAR_TRACKER_PARAM=${AR_TRACKER_PARAM})

link_directories(${PYTH_LIB_PATH})

set(ENABLE_VISUALIZATION 1)

#AUX_SOURCE_DIRECTORY(src/gui/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/robot/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/utils/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/robot/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/types/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/control/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/storage/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/learning/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/planning/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/statistics/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/learning/rl/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/manipulation/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/robot/kuka_driver/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/learning/regression/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/manipulation/haptic/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/learning/clustering/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/manipulation/playing/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/manipulation/pushing/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/utils/easyloggingpp/src/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/learning/metric_learning/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/robot/kinematics/constraints KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/learning/projective_simulation KUKADUSRC)

AUX_SOURCE_DIRECTORY(src/vision/ KUKADUVISIONSRC)
AUX_SOURCE_DIRECTORY(src/vision/arlocalizer ARTOOLKITPLUSSRC)

AUX_SOURCE_DIRECTORY(external/ReflexxesTypeII/src/TypeIIRML/ REFLEXXESSRC)

add_subdirectory(external/komo)

# vision must not use c++11 because PCL crashes in that case
SET_SOURCE_FILES_PROPERTIES(
    ${KUKADUVISIONSRC}
    PROPERTIES
    COMPILE_FLAGS "-DUSEBOOST"
)

SET_SOURCE_FILES_PROPERTIES(
    ${ARTOOLKITPLUSSRC}
    PROPERTIES
    COMPILE_FLAGS "-DUSEBOOST"
)

# compile the core kukadu with c++11 if possible (required because the vision part crashes if compiled with c++11 due to PCL)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    SET_SOURCE_FILES_PROPERTIES(
        ${KUKADUSRC}
        PROPERTIES
        COMPILE_FLAGS "-std=c++11"
    )
    add_definitions(-DCPP11SUPPORTED)
elseif(COMPILER_SUPPORTS_CXX0X)
    SET_SOURCE_FILES_PROPERTIES(
        ${KUKADUSRC}
        PROPERTIES
        COMPILE_FLAGS "-std=c++0x"
    )
    add_definitions(-DCPP11SUPPORTED)
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

add_library(reflexxeslib ${REFLEXXESSRC})
add_library(kukaduvision ${KUKADUVISIONSRC})
add_library(artoolkitplus ${ARTOOLKITPLUSSRC})
add_library(${PROJECT_NAME} ${KUKADUSRC} ${MyProjectLib_src} ${MyProjectLib_hdr_moc})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${ARMADILLO_LIBRARIES}
    reflexxeslib ncurses gsl gslcblas stdc++ pthread dl util m komo swift algo ann gui ors optim motion rt lapack blas glut GLU GL X11 ${GTK_LIBRARIES} ${GTK2_LIBRARIES}
    gthread-2.0 qhull ${PYTH_LIB} m mysqlcppconn Qt5::Widgets)

target_link_libraries(kukaduvision ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${ARMADILLO_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(artoolkitplus ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#add_executable(kukadu_demo_guided tutorials/guided.cpp)
#target_link_libraries(kukadu_demo_guided kukadu ${catkin_LIBRARIES} komo swift algo ann gui ors optim motion rt lapack blas glut GLU GL X11 ${GTK_LIBRARIES} ${GTK2_LIBRARIES} gthread-2.0 qhull m)

#add_executable(simple_planning tutorials/simple_planning.cpp)
#target_link_libraries(simple_planning kukadu ${catkin_LIBRARIES})

#add_executable(komo_planning tutorials/komo_planning.cpp)
#target_link_libraries(komo_planning kukadu ${catkin_LIBRARIES})

#add_executable(moveit_planning tutorials/moveit_planning.cpp)
#target_link_libraries(moveit_planning kukadu ${catkin_LIBRARIES})

#add_executable(cart_traj tutorials/cartesian_trajectories.cpp)
#target_link_libraries(cart_traj kukadu ${catkin_LIBRARIES})

add_executable(artracker tutorials/artracker.cpp)
target_link_libraries(artracker ${catkin_LIBRARIES} artoolkitplus kukadu)

add_executable(control_queue tutorials/control_queue.cpp)
if(COMPILER_SUPPORTS_CXX11)
    SET_TARGET_PROPERTIES(control_queue PROPERTIES COMPILE_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    SET_TARGET_PROPERTIES(control_queue PROPERTIES COMPILE_FLAGS "-std=c++0x")
endif()
target_link_libraries(control_queue kukadu ${catkin_LIBRARIES})

#add_executable(gaussian tutorials/gaussianprocess.cpp)
#target_link_libraries(gaussian kukadu ${catkin_LIBRARIES})

#add_executable(dmp tutorials/dmp.cpp)
#target_link_libraries(dmp kukadu ${catkin_LIBRARIES})

#add_executable(database tutorials/database.cpp)
#target_link_libraries(database kukadu ${catkin_LIBRARIES})

#add_executable(graphics tutorials/graphics.cpp)
#target_link_libraries(graphics kukadu ${catkin_LIBRARIES})

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${PYTH_LIB})
add_dependencies(kukaduvision ${catkin_EXPORTED_TARGETS})
#add_dependencies(artoolkitplus ${catkin_EXPORTED_TARGETS} artoolkitplus_gencfg kukadu kukaduvision)
#add_dependencies(reflexxeslib ${catkin_EXPORTED_TARGETS})
#add_dependencies(kukadu_demo_guided ${catkin_EXPORTED_TARGETS})
#add_dependencies(komo_planning ${catkin_EXPORTED_TARGETS})
#add_dependencies(moveit_planning ${catkin_EXPORTED_TARGETS})
#add_dependencies(simple_planning ${catkin_EXPORTED_TARGETS})
#add_dependencies(dmp ${catkin_EXPORTED_TARGETS})
#add_dependencies(gaussian ${catkin_EXPORTED_TARGETS})
#add_dependencies(database ${catkin_EXPORTED_TARGETS})
#add_dependencies(graphics ${catkin_EXPORTED_TARGETS})
add_dependencies(control_queue ${catkin_EXPORTED_TARGETS})
add_dependencies(artracker ${catkin_EXPORTED_TARGETS} artoolkitplus kukadu)

if(CATKIN_ENABLE_TESTING)
        catkin_add_gtest(kukadutest test/kukadutest.cpp)
endif()

#install(TARGETS graphics RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#install(TARGETS database RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#install(TARGETS gaussian RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#install(TARGETS kukadu_demo_guided RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS control_queue RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS artracker RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#install(TARGETS simple_planning RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#install(TARGETS komo_planning RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#install(TARGETS moveit_planning RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#install(TARGETS dmp RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS ${PROJECT_NAME} kukaduvision reflexxeslib ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
