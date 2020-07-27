#include_directories("D:/boost_1_64_0/boost_1_64_0/")
#link_directories("D:/boost_1_64_0/boost_1_64_0/vc141-x64/lib")


#link_directories(${PROJECT_SOURCE_DIR}/3rdparty/ceres_solver/lib)
#link_directories(${PROJECT_SOURCE_DIR}/3rdparty/glog/lib)

message(${OpenCV_LIBS})
add_executable(captureImage
               captureImage.cpp
              )
target_link_libraries(captureImage
                     ${OpenCV_LIBS})

add_executable(fisheyeCalib fisheye_calibrate.cpp)
target_link_libraries(fisheyeCalib ${OpenCV_LIBS}
        ${Boost_LIBRARIES})

add_executable(FisheyeStereo_calib FisheyeStereo_calib.cpp)
target_link_libraries(FisheyeStereo_calib ${OpenCV_LIBS}
        ${Boost_LIBRARIES})

//add_executable(stereo_calib stereo_calib.cpp)
//target_link_libraries(stereo_calib ${OpenCV_LIBS}
        //${Boost_LIBRARIES})	  
