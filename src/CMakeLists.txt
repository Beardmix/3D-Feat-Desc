cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(3DFeatureDescriptors)

find_package(PCL 1.6 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

SET(SOURCE
  main.cpp
)

SET(HEADERS
	PCA/PCA.cpp
  common/common.cpp
  SplitSpace/SplitSpace.cpp
  Signature/Signature.cpp
  Match/Match.cpp
  Search/Search.cpp
  KinectSearch/KinectSearch.cpp
  Viewer/Viewer.cpp
)
set(CMAKE_CXX_FLAGS "-g -Wall -Wno-deprecated")

ADD_EXECUTABLE(exe ${SOURCE} ${HEADERS})
target_link_libraries (exe ${PCL_LIBRARIES})



