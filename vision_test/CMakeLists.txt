cmake_minimum_required(VERSION 2.8)
project( DisplayImage )
find_package( OpenCV REQUIRED )
add_executable( SURF_detector SURF_detector.cpp )
target_link_libraries( SURF_detector ${OpenCV_LIBS} )

add_executable( SURF_Homography SURF_Homography.cpp )
target_link_libraries( SURF_Homography ${OpenCV_LIBS} )

add_executable( objectDetection objectDetection.cpp )
target_link_libraries( objectDetection ${OpenCV_LIBS} )

add_executable( continuous continuous.cpp )
target_link_libraries( continuous ${OpenCV_LIBS} )

add_executable( continuous_SIFT continuous_SIFT.cpp )
target_link_libraries( continuous_SIFT ${OpenCV_LIBS} )

add_executable( SIFT_2images SIFT_2images.cpp)
target_link_libraries( SIFT_2images ${OpenCV_LIBS})

#add_executable( test test.cpp )
#target_link_libraries( continuous ${OpenCV_LIBS} )

add_executable( corner corner.cpp )
target_link_libraries( corner ${OpenCV_LIBS} )
