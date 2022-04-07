cmake_minimum_required(VERSION 2.8)
project(ORB_SLAM3)

#IF(NOT CMAKE_BUILD_TYPE)
#  SET(CMAKE_BUILD_TYPE Debug)
#ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall   -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wno-deprecated -O3 -march=native ")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-deprecated -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(Pangolin REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_search_module(EIGEN3 REQUIRED eigen3)
pkg_search_module(OPENCV REQUIRED opencv4)
pkg_search_module(G2O  REQUIRED g2o)
pkg_search_module(DBoW REQUIRED dbow2)

find_package(realsense2)

# Installation
set(ORB_SLAM3_CXX_FLAGS "")
set(ORB_SLAM3_LINKER_FLAGS "")
set(ORB_SLAM3_INCLUDE_DIRS "")

list(APPEND ORB_SLAM3_INCLUDE_DIRS   ${EIGEN3_INCLUDE_DIR})
list(APPEND ORB_SLAM3_LINKER_FLAGS   ${EIGEN3_LDFLAGS})

list(APPEND ORB_SLAM3_CXX_FLAGS      ${OPENCV_CXX_FLAGS})
list(APPEND ORB_SLAM3_LINKER_FLAGS   ${OPENCV_LDFLAGS})
list(APPEND ORB_SLAM3_INCLUDE_DIRS   ${OPENCV_INCLUDE_DIRS})

list(APPEND ORB_SLAM3_CXX_FLAGS      ${G2O_CXX_FLAGS})
list(APPEND ORB_SLAM3_LINKER_FLAGS   ${G2O_LDFLAGS})
list(APPEND ORB_SLAM3_INCLUDE_DIRS   ${G2O_INCLUDE_DIRS})

list(APPEND ORB_SLAM3_CXX_FLAGS      ${DBoW_CXX_FLAGS})
list(APPEND ORB_SLAM3_LINKER_FLAGS   ${DBoW_LDFLAGS})
list(APPEND ORB_SLAM3_INCLUDE_DIRS   ${DBoW_INCLUDE_DIRS})

list(APPEND ORB_SLAM3_LINKER_FLAGS -lboost_serialization -lcrypto)

message("ORB_SLAM3_CXX_FLAGS:       ${ORB_SLAM3_CXX_FLAGS}")
message("ORB_SLAM3_LINKER_FLAGS:    ${ORB_SLAM3_LINKER_FLAGS}")
message("ORB_SLAM3_INCLUDE_DIRS:    ${ORB_SLAM3_INCLUDE_DIRS}")


include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${PROJECT_SOURCE_DIR}/include/CameraModels
${ORB_SLAM3_INCLUDE_DIRS}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
src/System.cc
src/Tracking.cc
src/LocalMapping.cc
src/LoopClosing.cc
src/ORBextractor.cc
src/ORBmatcher.cc
src/FrameDrawer.cc
src/Converter.cc
src/MapPoint.cc
src/KeyFrame.cc
src/Atlas.cc
src/Map.cc
src/MapDrawer.cc
src/Optimizer.cc
src/Frame.cc
src/KeyFrameDatabase.cc
src/Sim3Solver.cc
src/Initializer.cc
src/Viewer.cc
src/ImuTypes.cc
src/G2oTypes.cc
src/CameraModels/Pinhole.cpp
src/CameraModels/KannalaBrandt8.cpp
src/OptimizableTypes.cpp
src/MLPnPsolver.cpp
src/TwoViewReconstruction.cc
)

target_link_libraries(${PROJECT_NAME}
  ${Pangolin_LIBRARIES}
  ${ORB_SLAM3_LINKER_FLAGS}
)


install( DIRECTORY ${CMAKE_SOURCE_DIR}/include     DESTINATION ${CMAKE_INSTALL_PREFIX}/include/ORB_SLAM3)
install( TARGETS ORB_SLAM3 ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)

configure_file("orbslam3.pc.in" "orbslam3.pc" @ONLY)
install( FILES ${CMAKE_CURRENT_BINARY_DIR}/orbslam3.pc DESTINATION /usr/local/lib/pkgconfig/)

### Build examples

# RGB-D examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D)

#add_executable(rgbd_tum
#Examples/RGB-D/rgbd_tum.cc)
#target_link_libraries(rgbd_tum ${PROJECT_NAME})


# Stereo examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo)

#add_executable(stereo_kitti
#Examples/Stereo/stereo_kitti.cc)
#target_link_libraries(stereo_kitti ${PROJECT_NAME})

#add_executable(stereo_euroc
#Examples/Stereo/stereo_euroc.cc)
#target_link_libraries(stereo_euroc ${PROJECT_NAME})

#add_executable(stereo_tum_vi
#Examples/Stereo/stereo_tum_vi.cc)
#target_link_libraries(stereo_tum_vi ${PROJECT_NAME})

# Monocular examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular)

#add_executable(mono_tum
#Examples/Monocular/mono_tum.cc)
#target_link_libraries(mono_tum ${PROJECT_NAME})

#add_executable(mono_kitti
#Examples/Monocular/mono_kitti.cc)
#target_link_libraries(mono_kitti ${PROJECT_NAME})

#add_executable(mono_euroc
#Examples/Monocular/mono_euroc.cc)
#target_link_libraries(mono_euroc ${PROJECT_NAME})

#add_executable(mono_tum_vi
#Examples/Monocular/mono_tum_vi.cc)
#target_link_libraries(mono_tum_vi ${PROJECT_NAME})

# Monocular-Inertial examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular-Inertial)

#add_executable(mono_inertial_euroc
#Examples/Monocular-Inertial/mono_inertial_euroc.cc)
#target_link_libraries(mono_inertial_euroc ${PROJECT_NAME})

add_executable(mono_inertial_test
Examples/Monocular-Inertial/mono_inertial_test.cc)
target_link_libraries(mono_inertial_test ${PROJECT_NAME})

#add_executable(mono_inertial_tum_vi
#Examples/Monocular-Inertial/mono_inertial_tum_vi.cc)
#target_link_libraries(mono_inertial_tum_vi ${PROJECT_NAME})

# Stereo-Inertial examples
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo-Inertial)

#add_executable(stereo_inertial_euroc
#Examples/Stereo-Inertial/stereo_inertial_euroc.cc)
#target_link_libraries(stereo_inertial_euroc ${PROJECT_NAME})

#add_executable(stereo_inertial_tum_vi
#Examples/Stereo-Inertial/stereo_inertial_tum_vi.cc)
#target_link_libraries(stereo_inertial_tum_vi ${PROJECT_NAME})

