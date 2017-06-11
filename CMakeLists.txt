cmake_minimum_required(VERSION 3.2)
include(ExternalProject)

project(rfmqtt CXX)

set(includes
	src/vendor/
	src/vendor/yaml-cpp/include/
	src/vendor/paho.mqtt.c/src/
)

set(sources
	src/rfmqtt.cpp
)

add_subdirectory(src/vendor/yaml-cpp EXCLUDE_FROM_ALL)
add_subdirectory(src/vendor/docopt EXCLUDE_FROM_ALL)
add_subdirectory(src/vendor/paho.mqtt.c EXCLUDE_FROM_ALL)


set(CMAKE_CXX_STANDARD 11) # C++11...
set(CMAKE_CXX_STANDARD_REQUIRED ON) #...is required...
set(CMAKE_CXX_EXTENSIONS OFF) #...without compiler extensions like gnu++11

if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-long-long -pedantic")
endif()

add_executable(rfmqtt ${sources})
target_link_libraries(rfmqtt wiringPi yaml-cpp docopt pthread paho-mqtt3c)
target_include_directories(rfmqtt PRIVATE ${includes})

# add_custom_command(TARGET rfmqtt POST_BUILD
# 	COMMAND ${CMAKE_COMMAND} -E copy_if_different
# 		"${PROJECT_SOURCE_DIR}/build/src/vendor/paho.mqtt.c/src/$<CONFIGURATION>/paho-mqtt3c.dll"
# 		$<TARGET_FILE_DIR:rfmqtt>/paho-mqtt3c.dll)