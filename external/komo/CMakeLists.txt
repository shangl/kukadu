project(komo)
cmake_minimum_required(VERSION 2.6)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

add_definitions(-fPIC)
add_definitions(-DMT_LAPACK)
add_definitions(-DMT_Core)
add_definitions(-DMT_GTKGL)
add_definitions(-DMT_GL)
add_definitions(-DMT_GTK)
add_definitions(-DMT_QHULL)
add_definitions(-DMT_Algo)
add_definitions(-DMT_Gui)
add_definitions(-DMT_ANN)
add_definitions(-DMT_extern_SWIFT)
add_definitions(-DSWIFT_DECOMP)
add_definitions(-DNO_CYCLE_DETECTION)

find_package(Boost 1.54 REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(GTK2 REQUIRED gtk+-2.0)
pkg_check_modules(GTK REQUIRED gtkglext-1.0)

include_directories(
    share/src
    share/include
    share/src/extern/SWIFT_decomposer/RAPID
    ${Qt5Gui_INCLUDE_DIRS}
    ${QT5Core_INCLUDE_DIRS}
    ${QT5CoreWidgets_INCLUDE_DIRS}
    ${GTK2_INCLUDE_DIRS}
    ${GTK_INCLUDE_DIRS}
)

add_library(core share/src/Core/array_lapack.cpp share/src/Core/util.cpp share/src/Core/core.cpp
    share/src/Core/geo.cpp share/src/Core/array.cpp share/src/Core/thread.cpp share/src/Core/registry.cpp
    share/src/Core/module.cpp share/src/Core/keyValueGraph.cpp share/src/Core/module_FloatA_Recorder.cpp
)

add_library(algo share/src/Algo/rrt.cpp share/src/Algo/ann.cpp share/src/Algo/spline.cpp share/src/Algo/kalman.cpp
    share/src/Algo/gaussianProcess_plot.cpp share/src/Algo/gaussianProcess.cpp
    share/src/Algo/MLcourse.cpp share/src/Algo/algos.cpp
)

add_library(swift share/src/extern/SWIFT/mesh.cpp share/src/extern/SWIFT/mesh_utils.cpp share/src/extern/SWIFT/scene.cpp
    share/src/extern/SWIFT/pair.cpp share/src/extern/SWIFT/lut.cpp share/src/extern/SWIFT/fileio.cpp
    share/src/extern/SWIFT/pqueue.cpp share/src/extern/SWIFT/object.cpp
)

add_library(gui share/src/Gui/mesh.cpp share/src/Gui/ors_opengl.cpp share/src/Gui/plot.cpp
    share/src/Gui/geo_qhull.cpp share/src/Gui/gtk.cpp share/src/Gui/graphview.cpp share/src/Gui/opengl.cpp
    share/src/Gui/mesh_lewiner.cpp
)

add_library(ors share/src/Ors/ors_opengl.cpp share/src/Ors/ors_sceneGui.cpp share/src/Ors/ors_oldTaskVariables.cpp
    share/src/Ors/ors.cpp share/src/Ors/roboticsCourse.cpp  share/src/Ors/ors_physx.cpp share/src/Ors/ors_actionInterface.cpp
    share/src/Ors/ors_ode.cpp share/src/Ors/ors_swift.cpp share/src/Ors/ors_featherstone.cpp share/src/Ors/ors_blender.cpp
)

add_library(optim share/src/Optim/opt-constrained.cpp share/src/Optim/opt-newton.cpp share/src/Optim/optimization.cpp
    share/src/Optim/search_CMA.cpp share/src/Optim/benchmarks.cpp share/src/Optim/opt-rprop.cpp share/src/Optim/opt-convert.cpp
    share/src/Optim/cmaes.c
)

add_library(motion share/src/Motion/motion.cpp share/src/Motion/adaptiveMotionExecution.cpp share/src/Motion/pr2_heuristics
    share/src/Motion/dynamicMovementPrimitives.cpp share/src/Motion/taskMap_proxy.cpp
    share/src/Motion/feedbackControl.cpp share/src/Motion/gamepad2tasks.cpp
    share/src/Motion/taskMap_default.cpp share/src/Motion/rrt_planner.cpp share/src/Motion/mobject.cpp
    share/src/Motion/komo.cpp share/src/Motion/motionHeuristics.cpp share/src/Motion/komo.cpp share/src/Motion/taskMap_transition.cpp
    share/src/Motion/taskMap_constrained.cpp
)

add_library(swiftdeco share/src/extern/SWIFT_decomposer/viewer.cpp share/src/extern/SWIFT_decomposer/guidraw.cpp
    share/src/extern/SWIFT_decomposer/cvxutils.cpp share/src/extern/SWIFT_decomposer/RAPID/collide.cpp
    share/src/extern/SWIFT_decomposer/RAPID/overlap.cpp share/src/extern/SWIFT_decomposer/RAPID/build.cpp
    share/src/extern/SWIFT_decomposer/convex.cpp share/src/extern/SWIFT_decomposer/io.cpp
    share/src/extern/SWIFT_decomposer/gui.cpp
)

target_link_libraries(core swift)
target_link_libraries(algo core swift ann qhull)
target_link_libraries(gui core swift algo ann glut GLU GL X11 ${GTK_LIBRARIES} ${GTK2_LIBRARIES})
target_link_libraries(ors core algo ann gui swift rt glut GLU GL X11)
target_link_libraries(optim core rt)
target_link_libraries(motion core ors optim algo rt)

add_library(komo share/src/Komo/komo.cpp)
target_link_libraries(komo core algo swift gui ors optim motion rt lapack blas glut ann GLU GL X11 ${GTK_LIBRARIES} ${GTK2_LIBRARIES} gthread-2.0 qhull m)

