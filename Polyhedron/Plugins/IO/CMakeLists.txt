include( polyhedron_demo_macros )

qt5_wrap_ui( funcUI_FILES Function_dialog.ui )

list(FIND CMAKE_CXX_COMPILE_FEATURES cxx_rvalue_references has_cxx_rvalues)
list(FIND CMAKE_CXX_COMPILE_FEATURES cxx_variadic_templates has_cxx_variadic)

if(has_cxx_rvalues LESS 0 OR has_cxx_variadic LESS 0)
  message(STATUS "NOTICE: LAS/PLY IO examples require a C++11 compiler and will not be compiled.")
else()
  set(needed_cxx_features cxx_rvalue_references cxx_variadic_templates)
  
  polyhedron_demo_plugin(ply_plugin PLY_io_plugin KEYWORDS IO PointSetProcessing Classification)
  target_link_libraries(ply_plugin PUBLIC scene_points_with_normal_item scene_polygon_soup_item scene_surface_mesh_item scene_polygon_soup_item)
  target_compile_features(ply_plugin PRIVATE ${needed_cxx_features})
endif()
