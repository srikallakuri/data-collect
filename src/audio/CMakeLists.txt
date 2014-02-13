cmake_minimum_required(VERSION 2.8.3)

project(capture)

find_package(catkin REQUIRED COMPONENTS roscpp audio_common_msgs)

find_package(PkgConfig)
pkg_check_modules(GST gstreamer-0.10)

find_package(Boost REQUIRED COMPONENTS thread)

include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${GST_INCLUDE_DIRS})

catkin_package()

add_executable(capture src/capture.cpp)
target_link_libraries(capture ${catkin_LIBRARIES} ${GST_LIBRARIES} ${Boost_LIBRARIES} asound) 

add_executable(capture2 src/capture2.cpp)
target_link_libraries(capture2 ${catkin_LIBRARIES} ${GST_LIBRARIES} ${Boost_LIBRARIES} asound) 

add_executable(playback src/playback.cpp)
target_link_libraries(playback ${catkin_LIBRARIES} ${GST_LIBRARIES} ${Boost_LIBRARIES} asound) 

install(TARGETS capture
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
