project( Polyhedron_Demo )

cmake_minimum_required(VERSION 3.1)
if(NOT POLICY CMP0070 AND POLICY CMP0053)
  # Only set CMP0053 to OLD with CMake<3.10, otherwise there is a warning.
  cmake_policy(SET CMP0053 OLD)
endif()
if(POLICY CMP0071)
  cmake_policy(SET CMP0071 NEW)
endif()
if(POLICY CMP0072)
  # About the use of OpenGL
  cmake_policy(SET CMP0072 NEW)
endif()

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()


# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)

list(FIND CMAKE_CXX_COMPILE_FEATURES cxx_generalized_initializers has_cpp11)
if(has_cpp11 LESS 0)
  message(STATUS "NOTICE: This demo requires a C++11 compiler and will not be compiled.")
  return()
endif()

# Use C++11 for this directory and its sub-directories.
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

#Defines flags to emulate windows behavior for linking error generation
if(CMAKE_CXX_COMPILER_ID EQUAL Clang OR CMAKE_COMPILER_IS_GNUCC  OR CMAKE_COMPILER_IS_GNUCXX)
  if(UNIX OR APPLE)
    SET( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -fvisibility=hidden" )
  endif()
  if(UNIX AND NOT APPLE)
    SET( CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} -z defs")
    SET( CMAKE_MODULE_LINKER_FLAGS  "${CMAKE_MODULE_LINKER_FLAGS} -z defs")
    SET( CMAKE_SHARED_LINKER_FLAGS  "${CMAKE_SHARED_LINKER_FLAGS} -z defs")
  endif()
endif()
# Let plugins be compiled in the same directory as the executable.
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")

# Include this package's headers first
include_directories( BEFORE ./ ./include ./CGAL_demo)
list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_SOURCE_DIR}")

# Find CGAL

option( POLYHEDRON_QTSCRIPT_DEBUGGER 
  "Activate the use of Qt Script Debugger in UrbanMeshAnnotator" OFF)

# Find Qt5 itself
find_package(CGAL COMPONENTS Qt5 ImageIO PATHS ../CGAL)
include(${CGAL_USE_FILE})

set(CMAKE_BUILD_TYPE Release)

SET(CGAL_INCLUDE_DIR ../CGAL/include)

find_package(Qt5
             QUIET
             COMPONENTS            OpenGL Script
             OPTIONAL_COMPONENTS   ScriptTools)

# Find OpenGL
find_package(OpenGL)

if(Qt5_FOUND)

  add_definitions(-DQT_NO_KEYWORDS)
  add_definitions(-DSCENE_IMAGE_GL_BUFFERS_AVAILABLE)
endif(Qt5_FOUND)

find_package(Eigen3 3.2.0) #(requires 3.2.0 or greater)
if (EIGEN3_FOUND)
  include( ${EIGEN3_USE_FILE} )
endif(EIGEN3_FOUND)

# Activate concurrency?
option(POLYHEDRON_DEMO_ACTIVATE_CONCURRENCY
  "Enable concurrency"
  ON)

# Activate concurrency ? (turned OFF by default)
option(CGAL_ACTIVATE_CONCURRENT_MESH_3
  "Activate parallelism in Mesh_3"
  OFF)


if(CGAL_Qt5_FOUND AND Qt5_FOUND AND OPENGL_FOUND )
  include(${CGAL_USE_FILE})

  set(Boost_USE_MULTITHREADED      ON)
  find_package(Boost COMPONENTS thread system filesystem)
  
  qt5_wrap_ui( MainWindowUI_files MainWindow.ui)
  qt5_wrap_ui( statisticsUI_FILES Statistics_on_item_dialog.ui)
  qt5_wrap_ui( FileLoaderDialogUI_files FileLoaderDialog.ui )
  qt5_wrap_ui( PreferencesUI_FILES   Preferences.ui Details.ui)
  qt5_wrap_ui( ViewerUI_FILES  LightingDialog.ui)
  qt5_generate_moc( "File_loader_dialog.h" "${CMAKE_CURRENT_BINARY_DIR}/File_loader_dialog_moc.cpp" )

  include( ${CMAKE_CURRENT_SOURCE_DIR}/polyhedron_demo_macros.cmake )


  qt5_add_resources ( CGAL_Qt5_RESOURCE_FILES Polyhedron_3.qrc )
  
  #set(CGAL_THREE_HEADERS_PATH ${CGAL_DIR}/include)
  #find_path(CGAL_THREE_HEADERS_PATH
  #  NAME CGAL/Three/Scene_item.h
  #  HINTS ${CGAL_INCLUDE_DIRS}
  #  NO_DEFAULT_PATH
  #  DOC "Path to CGAL/Three/Scene_item.h")
  #message("++++++++++++++++find " ${CGAL_THREE_HEADERS_PATH})
  
  #if(CGAL_THREE_HEADERS_PATH)
    qt5_generate_moc( "${CGAL_INCLUDE_DIR}/CGAL/Three/Viewer_interface.h"
      "${CMAKE_CURRENT_BINARY_DIR}/Viewer_interface_moc.cpp" )
    qt5_generate_moc( "${CGAL_INCLUDE_DIR}/CGAL/Three/Scene_item.h"
      "${CMAKE_CURRENT_BINARY_DIR}/Scene_item_moc.cpp" )
    qt5_generate_moc( "${CGAL_INCLUDE_DIR}/CGAL/Three/Scene_group_item.h"
      "${CMAKE_CURRENT_BINARY_DIR}/Scene_group_item_moc.cpp" )
    qt5_generate_moc( "${CGAL_INCLUDE_DIR}/CGAL/Three/Scene_item_rendering_helper.h"
         "${CMAKE_CURRENT_BINARY_DIR}/Scene_item_rendering_helper_moc.cpp" )
    qt5_generate_moc( "${CGAL_INCLUDE_DIR}/CGAL/Three/TextRenderer.h"
      "${CMAKE_CURRENT_BINARY_DIR}/TextRenderer_moc.cpp" )
  #else()
  #  message(FATAL_ERROR "Cannot find <CGAL/Three/Viewer_interface.h>")
  #endif()

  #unset(CGAL_THREE_HEADERS_PATH CACHE)

# AUXILIARY LIBRARIES

  # put s (which are shared libraries) at the same location as
  # executable files
  set(CGAL_POLYHEDRON_DEMO_PLUGINS_DIR "${RUNTIME_OUTPUT_PATH}")
  set(LIBRARY_OUTPUT_PATH "${CGAL_POLYHEDRON_DEMO_PLUGINS_DIR}")

  set(CMAKE_WARN_DEPRECATED OFF)
  add_definitions(-DQT_NO_DEPRECATED_WARNINGS)

  add_library(demo_framework SHARED
    Scene.cpp
    Viewer.cpp
    Three.cpp
    ${ViewerUI_FILES}
    ${CGAL_Qt5_RESOURCE_FILES} ${CGAL_Qt5_MOC_FILES}
    Viewer_interface_moc.cpp
    Scene_item_moc.cpp
    Scene_item.cpp
    Triangle_container.cpp
    Edge_container.cpp
    Point_container.cpp
    Scene_group_item.cpp
    Scene_group_item_moc.cpp
    TextRenderer.cpp
    TextRenderer_moc.cpp
    Scene_item_rendering_helper.cpp
    Scene_item_rendering_helper_moc.cpp
    Primitive_container.cpp
    Polyhedron_demo_plugin_helper.cpp)
  target_link_libraries(demo_framework
    PUBLIC Qt5::OpenGL Qt5::Widgets Qt5::Gui Qt5::Script
    )
  # Let's define `three_EXPORT` during the compilation of `demo_framework`,
  # in addition of `demo_framework_EXPORT` (defined automatically by
  # CMake). That is to deal with the visibility of symbols of
  # `Three.h`/`Three.cpp`.
  target_compile_definitions(demo_framework PRIVATE three_EXPORTS=1)
  if(CGAL_HEADER_ONLY)
    target_compile_definitions(demo_framework PRIVATE -DCGAL_USE_Qt5_RESOURCES)
  endif()

  add_library(scene_color_ramp SHARED Color_ramp.cpp)
  target_link_libraries(scene_color_ramp PRIVATE Qt5::Core)

  add_library(scene_callback_signaler SHARED Callback_signaler.cpp)
  target_link_libraries(scene_callback_signaler PRIVATE Qt5::Core)

  macro(add_item item_name)
    add_library(${item_name} SHARED ${ARGN})
    target_link_libraries(${item_name}
      PUBLIC demo_framework ${CGAL_LIBRARIES} ${Boost_LIBRARIES}
      Qt5::OpenGL Qt5::Gui Qt5::Script Qt5::Widgets)
    cgal_add_compilation_test(${item_name})
  endmacro(add_item)

  add_item(scene_edit_box_item Plugins/PCA/Scene_edit_box_item.cpp )
  add_item(scene_surface_mesh_item Scene_surface_mesh_item.cpp)

  add_item(scene_polylines_item Scene_polylines_item.cpp)


  add_item(scene_item_decorator Scene_polyhedron_item_decorator.cpp )
  target_link_libraries(scene_item_decorator PUBLIC scene_surface_mesh_item)
 
  add_item(scene_k_ring_selection Plugins/PMP/Scene_facegraph_item_k_ring_selection.cpp)
  target_link_libraries(scene_k_ring_selection PUBLIC scene_surface_mesh_item)

  add_item(scene_selection_item Scene_polyhedron_selection_item.cpp)
  target_link_libraries(scene_selection_item PUBLIC scene_item_decorator  scene_k_ring_selection)

  if(EIGEN3_FOUND )
    add_item(scene_textured_item Scene_textured_surface_mesh_item.cpp texture.cpp)
    target_link_libraries(scene_surface_mesh_item PUBLIC scene_textured_item)
  endif()

  add_item(scene_polygon_soup_item Scene_polygon_soup_item.cpp)
  target_link_libraries(scene_polygon_soup_item PUBLIC scene_surface_mesh_item)

  add_item(scene_points_with_normal_item Scene_points_with_normal_item.cpp)

  foreach( lib 
      demo_framework
      scene_color_ramp
      scene_polygon_soup_item)
    add_to_cached_list( CGAL_EXECUTABLE_TARGETS ${lib} )
  endforeach()

  add_definitions( -DUSE_FORWARD_DECL)
  add_definitions( -DCGAL_KERNEL_NO_ASSERTIONS)
  add_library(polyhedron_demo SHARED
    MainWindow.cpp
    UrbanMeshAnnotator.cpp
    File_loader_dialog_moc.cpp
    ${CGAL_Qt5_RESOURCE_FILES} ${CGAL_Qt5_MOC_FILES}
    ${FileLoaderDialogUI_files} ${MainWindowUI_files} ${PreferencesUI_FILES} ${statisticsUI_FILES})
  target_link_libraries(polyhedron_demo PUBLIC
    demo_framework Qt5::Gui Qt5::OpenGL Qt5::Widgets Qt5::Script)
  add_executable  ( UrbanMeshAnnotator main.cpp)
  target_link_libraries( UrbanMeshAnnotator PRIVATE polyhedron_demo )
  add_to_cached_list( CGAL_EXECUTABLE_TARGETS UrbanMeshAnnotator )
  cgal_add_compilation_test(polyhedron_demo)

  if( POLYHEDRON_QTSCRIPT_DEBUGGER )
    if(TARGET Qt5::ScriptTools)
      target_link_libraries(polyhedron_demo PUBLIC Qt5::ScriptTools)
    else()
      message(STATUS "POLYHEDRON_QTSCRIPT_DEBUGGER is set to TRUE but the Qt5 ScriptTools library was not found.")
    endif()
  endif()
  target_link_libraries( UrbanMeshAnnotator PRIVATE demo_framework )

  # Link with CGAL
  target_link_libraries( UrbanMeshAnnotator PUBLIC ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES} )

  add_to_cached_list( CGAL_EXECUTABLE_TARGETS Polyhedron_3 )


  ###########
  # PLUGINS #
  ###########


  file(GLOB DEMO_PLUGINS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/Plugins/" "${CMAKE_CURRENT_SOURCE_DIR}/Plugins/*")
  FOREACH(SUB_DIR ${DEMO_PLUGINS})
      add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/Plugins/${SUB_DIR}")
  ENDFOREACH()
  add_dependencies(UrbanMeshAnnotator classification_plugin selection_plugin ply_plugin point_set_selection_plugin)
#
# EXECUTABLES
#
#
# Exporting
#
  if(TARGET CGAL_Qt5)
    export(TARGETS CGAL CGAL_Qt5 CGAL_ImageIO FILE polyhedron_demo_targets.cmake NAMESPACE Polyhedron_)
  else()
    export(TARGETS FILE polyhedron_demo_targets.cmake NAMESPACE Polyhedron_)
  endif()
  if(TARGET CGAL_Qt5_moc_and_resources)
    export(
      TARGETS CGAL_Qt5_moc_and_resources
      NAMESPACE Polyhedron_
      APPEND FILE polyhedron_demo_targets.cmake)
  endif()
  export(
    TARGETS
    demo_framework
    scene_surface_mesh_item
    scene_points_with_normal_item
    scene_color_ramp
    scene_polylines_item
    scene_selection_item
    scene_item_decorator
    scene_k_ring_selection
    scene_textured_item
    NAMESPACE Polyhedron_
    APPEND FILE polyhedron_demo_targets.cmake)

  export(
    TARGETS
    NAMESPACE Polyhedron_
    APPEND FILE polyhedron_demo_targets.cmake)
  configure_file(CGAL_polyhedron_demoConfig.cmake.in  CGAL_polyhedron_demoConfig.cmake)
#TO DO script the activation of all the plugins.

else (CGAL_Qt5_FOUND AND Qt5_FOUND AND OPENGL_FOUND )

  set(POLYHEDRON_MISSING_DEPS "")

  if(NOT CGAL_Qt5_FOUND)
    set(POLYHEDRON_MISSING_DEPS "the CGAL Qt5 library, ${POLYHEDRON_MISSING_DEPS}")
  endif()

  if(NOT Qt5_FOUND)
    set(POLYHEDRON_MISSING_DEPS "Qt5, ${POLYHEDRON_MISSING_DEPS}")
  endif()

  if(NOT OPENGL_FOUND)
    set(POLYHEDRON_MISSING_DEPS "OpenGL, ${POLYHEDRON_MISSING_DEPS}")
  endif()

  message(STATUS "NOTICE: This demo requires ${POLYHEDRON_MISSING_DEPS}and will not be compiled.")


endif (CGAL_Qt5_FOUND AND Qt5_FOUND AND OPENGL_FOUND )
