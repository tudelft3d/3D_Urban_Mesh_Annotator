include( polyhedron_demo_macros )

if(EIGEN3_FOUND)

  set(classification_linked_libraries)
  set(classification_compile_definitions)

  if( WIN32 )
# to avoid a warning with old cmake
      set(classification_linked_libraries ${classification_linked_libraries})
  endif()


  #message("p_klein_function_plugin.dll----------------------------")
	qt5_wrap_ui( classificationUI_FILES Classification_widget.ui)
	polyhedron_demo_plugin(classification_plugin Classification_plugin Surface_mesh_item_classification ${classificationUI_FILES})
    
	set(classification_linked_libraries ${classification_linked_libraries})
    
	target_link_libraries(classification_plugin PUBLIC scene_polylines_item scene_polygon_soup_item scene_surface_mesh_item scene_selection_item scene_color_ramp ${classification_linked_libraries})
	add_dependencies(classification_plugin selection_plugin)
	target_compile_definitions(classification_plugin PUBLIC ${classification_compile_definitions})


else(EIGEN3_FOUND)
  message(STATUS "NOTICE: Eigen 3.1 (or greater) was not found. Classification plugin will not be available.")
endif()


