project(HELLO_ICUB)
find_package(YARP REQUIRED)
find_package( OpenCV REQUIRED )
find_package(Eigen3 REQUIRED NO_MODULE)

INCLUDE_DIRECTORIES(~/aruco/include)
LINK_DIRECTORIES(~/aruco/lib)

add_executable(hello main.cpp ArucoDetector.cpp ../utils/utils.cpp)
# we now add the YARP libraries to our project.
 
target_link_libraries(hello ${YARP_LIBRARIES} ${OpenCV_LIBS} Eigen3::Eigen)