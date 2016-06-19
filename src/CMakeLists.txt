#cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
#project(cntrl)

#add_executable(test4 main.cpp)

#install(TARGETS test4 RUNTIME DESTINATION bin)


#find_package(gazebo REQUIRED)

#include_directories(${GAZEBO_INCLUDE_DIRS})
#link_directories(${GAZEBO_LIBRARY_DIRS})
#list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

#add_executable(test1 test1.cc)
#target_link_libraries(test1 ${GAZEBO_LIBRARIES})




include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()

include(FindBoost)
find_package(Boost ${MIN_BOOST_VERSION} REQUIRED system filesystem regex)

find_package(Protobuf REQUIRED)


include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

#include_directories(${cntrl_SOURCE_DIR}/src)
#add_subdirectory(${cntrl_SOURCE_DIR}/src)

#include_directories("src")
#add_subdirectory(src)

#include_directories("build")
#add_subdirectory(build)

add_executable(cntrl cntrl.cpp terminal_cntrl.cpp gz_comms.cpp testCntrl.cpp control_orientation.cpp control_navigation.cpp mavlink_comms.cc RC_controller_comms.cpp BuddyBox.c BuddyBoxThread.c PortAudioStream.c)
#target_link_libraries(cntrl ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} pthread "-lncurses" "-lportaudio")
target_link_libraries(cntrl ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} pthread ncurses portaudio)
