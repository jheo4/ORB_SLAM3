cmake_minimum_required(VERSION 2.8)
project(DBoW2)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(HDRS_DBOW2
  DBoW2/BowVector.h
  DBoW2/FORB.h
  DBoW2/FClass.h
  DBoW2/FeatureVector.h
  DBoW2/ScoringObject.h
  DBoW2/TemplatedVocabulary.h)
set(SRCS_DBOW2
  DBoW2/BowVector.cpp
  DBoW2/FORB.cpp
  DBoW2/FeatureVector.cpp
  DBoW2/ScoringObject.cpp)

set(HDRS_DUTILS
  DUtils/Random.h
  DUtils/Timestamp.h)
set(SRCS_DUTILS
  DUtils/Random.cpp
  DUtils/Timestamp.cpp)

find_package(PkgConfig REQUIRED)
pkg_search_module(OPENCV opencv opencv4)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(${OPENCV_INCLUDE_DIRS})
add_library(DBoW2 SHARED ${SRCS_DBOW2} ${SRCS_DUTILS})
target_link_libraries(DBoW2 ${OPENCV_LDFLAGS})

string(REPLACE ";" " " OPENCV_LDFLAGS  "${OPENCV_LDFLAGS}")

install( DIRECTORY ${CMAKE_SOURCE_DIR}/DBoW2  DESTINATION ${CMAKE_INSTALL_PREFIX}/include/ORB_SLAM3/Thirdparty/DBoW2)
install( DIRECTORY ${CMAKE_SOURCE_DIR}/DUtils DESTINATION ${CMAKE_INSTALL_PREFIX}/include/ORB_SLAM3/Thirdparty/DBoW2)
install( TARGETS DBoW2 ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
configure_file("dbow2.pc.in" "dbow2.pc" @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/dbow2.pc DESTINATION /usr/local/lib/pkgconfig/)

