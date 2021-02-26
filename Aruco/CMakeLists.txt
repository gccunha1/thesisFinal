project(ARUCO)
find_package( OpenCV REQUIRED )
INCLUDE_DIRECTORIES(~/aruco/include)
LINK_DIRECTORIES(~/aruco/lib)



add_executable(main main.cpp)
# we now add the YARP libraries to our project.
 
target_link_libraries(main ${OpenCV_LIBS})
