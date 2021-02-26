project(HELLO_ICUB)

INCLUDE_DIRECTORIES(/usr/local/include ~/aruco/include)
LINK_DIRECTORIES(/usr/local/lib ~/aruco/lib)

find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)
find_package( OpenCV REQUIRED )
find_package(Eigen3 REQUIRED NO_MODULE)

add_executable(main main.cpp ../iCub/Robot.cpp ../utils/utils.cpp ../iCubWorld/simobjloader.cpp ../iCubVision/ArucoDetector.cpp)
# we now add the YARP libraries to our project.
 
target_link_libraries(main ${YARP_LIBRARIES} Eigen3::Eigen iKin ${OpenCV_LIBS})
