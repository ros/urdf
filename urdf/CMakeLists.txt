cmake_minimum_required(VERSION 3.7)
project(urdf)

find_package(Boost REQUIRED thread)
find_package(urdfdom REQUIRED)
find_package(urdfdom_headers REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  urdf_parser_plugin pluginlib rosconsole_bridge roscpp cmake_modules)


find_package(TinyXML REQUIRED)
find_package(TinyXML2 REQUIRED)

# Find version components
if(NOT urdfdom_headers_VERSION)
  set(urdfdom_headers_VERSION "0.0.0")
endif()
string(REGEX REPLACE "^([0-9]+).*" "\\1" URDFDOM_HEADERS_MAJOR_VERSION "${urdfdom_headers_VERSION}")
string(REGEX REPLACE "^[0-9]+\\.([0-9]+).*" "\\1" URDFDOM_HEADERS_MINOR_VERSION "${urdfdom_headers_VERSION}")
string(REGEX REPLACE "^[0-9]+\\.[0-9]+\\.([0-9]+).*" "\\1" URDFDOM_HEADERS_REVISION_VERSION "${urdfdom_headers_VERSION}")
set(generated_compat_header "${CATKIN_DEVEL_PREFIX}/include/${PROJECT_NAME}/urdfdom_compatibility.h")
include_directories("${CATKIN_DEVEL_PREFIX}/include")
configure_file(urdfdom_compatibility.h.in "${generated_compat_header}" @ONLY)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  INCLUDE_DIRS include ${CATKIN_DEVEL_PREFIX}/include
  CATKIN_DEPENDS pluginlib rosconsole_bridge roscpp
  DEPENDS urdfdom_headers urdfdom Boost TinyXML TinyXML2
)
install(FILES ${generated_compat_header} DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${urdfdom_INCLUDE_DIRS}
  ${urdfdom_headers_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS}
  ${TinyXML2_INCLUDE_DIRS}
  )

link_directories(${Boost_LIBRARY_DIRS} ${catkin_LIBRARY_DIRS})

add_library(${PROJECT_NAME} src/model.cpp src/rosconsole_bridge.cpp)
target_link_libraries(${PROJECT_NAME} ${TinyXML_LIBRARIES} ${TinyXML2_LIBRARIES} ${catkin_LIBRARIES} ${urdfdom_LIBRARIES})

if(WIN32)
  target_compile_definitions(${PROJECT_NAME} PRIVATE "URDF_BUILDING_LIBRARY")
endif()

if(APPLE)
  set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS "-undefined dynamic_lookup")
endif(APPLE)

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS rostest)
  add_rostest_gtest(test_urdf_parser test/test_robot_model_parser.launch test/test_robot_model_parser.cpp)
  target_link_libraries(test_urdf_parser ${PROJECT_NAME})

  catkin_add_gtest(urdfdom_compatibility_test test/urdfdom_compatibility.cpp)
  target_link_libraries(urdfdom_compatibility_test ${PROJECT_NAME})

  if(NOT MSVC)
  set_source_files_properties(test/test_model_parser_initxml.cpp PROPERTIES COMPILE_FLAGS -Wno-deprecated-declarations)
  endif()
  catkin_add_gtest(test_model_parser_initxml test/test_model_parser_initxml.cpp)
  target_link_libraries(test_model_parser_initxml ${PROJECT_NAME})
endif()

# Install library
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
