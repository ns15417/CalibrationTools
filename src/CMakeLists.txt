#include_directories("D:/boost_1_64_0/boost_1_64_0/")
#link_directories("D:/boost_1_64_0/boost_1_64_0/vc141-x64/lib")


link_directories(${PROJECT_SOURCE_DIR}/3rdparty/ceres_solver/lib)
link_directories(${PROJECT_SOURCE_DIR}/3rdparty/glog/lib)

#add_executable(calibrationSingle 
                #calibrationSingle.cpp)
#target_link_libraries(calibrationSingle
                    # ${OpenCV_LIBS}
		      # boost_filesystem-vc140-mt-gd-1_64
		      # ceres
		      # glog)
					 
add_executable(captureImage
               captureImage.cpp
              )
target_link_libraries(captureImage
                     ${OpenCV_LIBS})
					 
add_executable(calibResultTest 
               calibResultTest.cpp)
target_link_libraries(calibResultTest
                       ${OpenCV_LIBS})
add_executable(StereoCalib
               stereo_calib.cpp)
target_link_libraries(StereoCalib
                      ${OpenCV_LIBS})
add_executable(FisheyeStereo_calib
              FisheyeStereo_calib.cpp)
target_link_libraries(FisheyeStereo_calib
                     ${OpenCV_LIBS})
					  
