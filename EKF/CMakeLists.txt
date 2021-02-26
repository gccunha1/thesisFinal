project(HELLO_ICUB)
find_package(YARP REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
INCLUDE_DIRECTORIES(/usr/local/include)

LINK_DIRECTORIES(/usr/local/lib)

add_executable(hello main.cpp ../utils/utils.cpp ../iCub/Robot.cpp ActiveSelectionEKF.cpp)

 
target_link_libraries(hello ${YARP_LIBRARIES} Eigen3::Eigen nlopt)