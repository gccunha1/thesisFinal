project(HELLO_ICUB)
find_package(YARP REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(ICUB REQUIRED)

add_executable(hello main.cpp Robot.cpp ../utils/utils.cpp ../iCubWorld/simobjloader.cpp)
# we now add the YARP libraries to our project.
 
target_link_libraries(hello ${YARP_LIBRARIES} Eigen3::Eigen)