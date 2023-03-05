cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(JSON_BuildTests OFF CACHE INTERNAL "")
add_subdirectory(json)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS} "${CMAKE_SOURCE_DIR}/range-v3/include")
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

# target_link_libraries(foo PRIVATE nlohmann_json::nlohmann_json)

add_library(pr2_model_control SHARED pr2_model_control.cc json)
# target_include_directories(pr2_model_control PRIVATE ${CMAKE_WORKING_DIR}/include)
target_link_libraries(pr2_model_control nlohmann_json ${GAZEBO_LIBRARIES})

add_library(box_grasping SHARED box_grasping.cc)
target_link_libraries(box_grasping ${GAZEBO_LIBRARIES})

