cmake_minimum_required(VERSION 3.5)

project(nanomap_rviz2_plugins)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wnon-virtual-dtor -Woverloaded-virtual)
endif()

find_package(ament_cmake REQUIRED)

find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)

find_package(Qt5 REQUIRED COMPONENTS Widgets Test)

find_package(geometry_msgs REQUIRED)

find_package(image_transport REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(map_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nanomap_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(resource_retriever REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ignition-math6 REQUIRED)
set(Blosc_ROOT "$ENV{HOME}/github/third-party/lib/blosc")
set(OpenVDB_LIBRARYDIR "$ENV{HOME}/github/third-party/lib/openvdb/lib")
set(CMAKE_MODULE_PATH "$ENV{HOME}/github/third-party/lib/openvdb/lib/cmake/OpenVDB")
find_package(OpenVDB REQUIRED)

set(nanomap_rviz2_plugins_headers
  include/nanomap_rviz2_plugins/displays/openvdb_grid/openvdb_grid_common.hpp
  include/nanomap_rviz2_plugins/displays/openvdb_grid/openvdb_grid_structs.hpp
  include/nanomap_rviz2_plugins/displays/openvdb_grid/openvdb_grid_display.hpp
  include/nanomap_rviz2_plugins/displays/openvdb_grid/openvdb_grid_transformer_factory.hpp
  include/nanomap_rviz2_plugins/displays/openvdb_grid/openvdb_grid_transformer.hpp
  include/nanomap_rviz2_plugins/displays/openvdb_grid/transformers/flat_color_grid_transformer.hpp
  include/nanomap_rviz2_plugins/displays/openvdb_grid/transformers/xyz_grid_transformer.hpp
  include/nanomap_rviz2_plugins/rendering/objects/openvdb_grid_renderable.hpp
  include/nanomap_rviz2_plugins/rendering/objects/openvdb_grid.hpp
)

foreach(header "${nanomap_rviz2_plugins_headers}")
  qt5_wrap_cpp(nanomap_rviz2_plugins_moc_files "${header}")
endforeach()

set(nanomap_rviz2_plugins_source_files
src/nanomap_rviz2_plugins/displays/openvdb_grid/openvdb_grid_common.cpp
src/nanomap_rviz2_plugins/displays/openvdb_grid/openvdb_grid_structs.cpp
src/nanomap_rviz2_plugins/displays/openvdb_grid/openvdb_grid_display.cpp
src/nanomap_rviz2_plugins/displays/openvdb_grid/openvdb_grid_transformer_factory.cpp
src/nanomap_rviz2_plugins/displays/openvdb_grid/transformers/flat_color_grid_transformer.cpp
src/nanomap_rviz2_plugins/displays/openvdb_grid/transformers/xyz_grid_transformer.cpp
src/nanomap_rviz2_plugins/rendering/objects/openvdb_grid_renderable.cpp
src/nanomap_rviz2_plugins/rendering/objects/openvdb_grid.cpp
)

add_library(nanomap_rviz2_plugins SHARED
  ${nanomap_rviz2_plugins_moc_files}
  ${nanomap_rviz2_plugins_source_files}
)

target_include_directories(nanomap_rviz2_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${Qt5Widgets_INCLUDE_DIRS}
  ${OpenVDB_INCLUDE_DIR}
)

target_link_libraries(nanomap_rviz2_plugins PUBLIC
  rviz_ogre_vendor::OgreMain
  rviz_ogre_vendor::OgreOverlay
  ${OpenVDB_LIBRARIES}
)

target_link_libraries(nanomap_rviz2_plugins PRIVATE
  ignition-math6
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(nanomap_rviz2_plugins PRIVATE "RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY")

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

ament_target_dependencies(nanomap_rviz2_plugins
  PUBLIC
  geometry_msgs
  image_transport
  interactive_markers
  laser_geometry
  map_msgs
  nav_msgs
  rclcpp
  resource_retriever
  rviz_common
  rviz_rendering
  nanomap_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  urdf
  visualization_msgs
)

# Export old-style CMake variables
ament_export_include_directories("include/${PROJECT_NAME}")

# Export modern CMake targets
ament_export_targets(nanomap_rviz2_plugins HAS_LIBRARY_TARGET)

ament_export_dependencies(
  geometry_msgs
  image_transport
  interactive_markers
  laser_geometry
  map_msgs
  nav_msgs
  nanomap_msgs
  rclcpp
  resource_retriever
  rviz_common
  rviz_ogre_vendor
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  urdf
  visualization_msgs
)

install(
  TARGETS nanomap_rviz2_plugins
  EXPORT nanomap_rviz2_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_package()
