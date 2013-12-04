cmake_minimum_required(VERSION 2.8.3)
project(ps3_ardrone)

find_package(catkin REQUIRED COMPONENTS rosconsole roscpp ps3joy ardrone_autonomy)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
catkin_package(
    DEPENDS roscpp ps3joy ardrone_autonomy
)

include_directories(${Boost_INCLUDE_DIRS} ${ardrone_autonomy_INCLUDE_DIRS})


add_executable(ps3_ardrone src/ps3_ardrone.cpp)
target_link_libraries(ps3_ardrone ${Boost_LIBRARIES} ${catkin_LIBRARIES})


install(TARGETS ps3_ardrone  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

