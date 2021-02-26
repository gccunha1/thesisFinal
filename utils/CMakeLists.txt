project(HELLO_ICUB)
find_package(Eigen3 REQUIRED NO_MODULE)

add_executable(hello test.cpp utils.cpp)
# we now add the YARP libraries to our project.
 
target_link_libraries(hello Eigen3::Eigen)