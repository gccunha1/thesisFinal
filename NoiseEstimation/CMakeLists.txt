project(HELLO_ICUB)

INCLUDE_DIRECTORIES(/usr/local/include/bayesopt/ /usr/local/include ~/aruco/include)
LINK_DIRECTORIES(/usr/local/lib ~/aruco/lib)

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)
find_package( OpenCV REQUIRED )
add_executable(teste main.cpp MarkerNoiseEstimator.cpp ../utils/utils.cpp ../iCub/Robot.cpp ../utils/utils.cpp ../iCubWorld/simobjloader.cpp ../iCubVision/ArucoDetector.cpp)
# we now add the YARP libraries to our project.
 
target_link_libraries(teste bayesopt nlopt ${YARP_LIBRARIES} Eigen3::Eigen iKin ${OpenCV_LIBS})