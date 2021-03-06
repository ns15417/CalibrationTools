cmake_minimum_required(VERSION 3.5)

project(CaptureVideo)
set(gcc_like_compilers GNU Clang Intel)

if(MSVC)
   # Force to always compile with W4
  if(CMAKE_CXX_FLAGS MATCHES "/W[0-4]" OR CMAKE_C_FLAGS MATCHES "/W[0-4]")
    string(REGEX REPLACE "/W[0-4]" "/W4" CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")
	string(REGEX REPLACE "/W[0-4]" "/W4" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
  else()
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /W4")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")
  endif()
elseif((";${gcc_like_compilers};" MATCHES ";${CMAKE_CXX_COMPILER_ID};"))
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Werror -msse3 -std=c++11 -pthread -g -march=native -Wno-deprecated-declarations")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -msse3 -std=c++11 -pthread -g -march=native -Wno-deprecated-declarations")
endif()
include_directories(${PROJECT_SOURCE_DIR}/src/)

set(OpenCV_DIR "/home/shinan/linux_lib/opencv-4.0.0/build/")
find_package(OpenCV 4.0 REQUIRED)
find_package(Eigen3)
message(hahah${OpenCV_INCLUDE_DIRS})
message(${EIGEN3_INCLUDE_DIR})
find_package(Boost REQUIRED COMPONENTS filesystem)
#set(CERES_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/3rdparty/ceres_solver/include)
#set(GLOG_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/3rdparty/glog/include)

include_directories(${Boost_INCLUDE_DIRS})
message(${Boost_INCLUDE_DIRS})
include_directories(
    ${OpenCV_INCLUDE_DIR}
	#${CERES_INCLUDE_DIR}
	#${GLOG_INCLUDE_DIR}
	${EIGEN3_INCLUDE_DIR})

add_subdirectory(src)
