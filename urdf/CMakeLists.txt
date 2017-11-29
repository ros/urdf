cmake_minimum_required(VERSION 3.5)
project(urdf)

find_package(ament_cmake_ros REQUIRED)
find_package(urdfdom REQUIRED)
find_package(urdfdom_headers REQUIRED)
find_package(tinyxml_vendor REQUIRED)
find_package(TinyXML REQUIRED)

# Find version components
if(NOT urdfdom_headers_VERSION)
set(urdfdom_headers_VERSION "0.0.0")
endif()
string(REGEX REPLACE "^([0-9]+).*" "\\1" URDFDOM_HEADERS_MAJOR_VERSION "${urdfdom_headers_VERSION}")
string(REGEX REPLACE "^[0-9]+\\.([0-9]+).*" "\\1" URDFDOM_HEADERS_MINOR_VERSION "${urdfdom_headers_VERSION}")
string(REGEX REPLACE "^[0-9]+\\.[0-9]+\\.([0-9]+).*" "\\1" URDFDOM_HEADERS_REVISION_VERSION "${urdfdom_headers_VERSION}")
set(generated_install_path "${CMAKE_CURRENT_BINARY_DIR}/include")
set(generated_compat_header "${generated_install_path}/${PROJECT_NAME}/urdfdom_compatibility.h")
include_directories(${generated_install_path})
configure_file(urdfdom_compatibility.h.in "${generated_compat_header}" @ONLY)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

install(FILES ${generated_compat_header} DESTINATION include/${PROJECT_NAME})

include_directories(
  include
  ${urdfdom_INCLUDE_DIRS}
  ${urdfdom_headers_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/model.cpp)
target_link_libraries(${PROJECT_NAME} ${TinyXML_LIBRARIES} ${urdfdom_LIBRARIES})

if(WIN32)
  target_compile_definitions(${PROJECT_NAME} PRIVATE "URDF_BUILDING_DLL")
endif()

if(APPLE)
  set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS "-undefined dynamic_lookup")
endif()

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  # This forces cppcheck to consider all files in this project to be C++,
  # including the headers which end with .h, which cppcheck would normally
  # consider to be C instead.
  ament_cppcheck(LANGUAGE "c++")
  ament_cpplint()
  ament_lint_cmake()
  ament_uncrustify()
endif()

ament_export_libraries(${PROJECT_NAME})
ament_export_include_directories(include)
ament_export_dependencies(tinyxml_vendor)
ament_export_dependencies(TinyXML)
ament_export_dependencies(urdfdom)
ament_export_dependencies(urdfdom_headers)
ament_package()
