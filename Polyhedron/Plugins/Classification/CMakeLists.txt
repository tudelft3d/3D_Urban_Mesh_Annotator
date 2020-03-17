include( polyhedron_demo_macros )

if(EIGEN3_FOUND)

  set(classification_linked_libraries)
  set(classification_compile_definitions)
  
  find_package(Boost OPTIONAL_COMPONENTS serialization iostreams)

  if( WIN32 )
# to avoid a warning with old cmake
    set(_Boost_BZIP2_HEADERS             "boost/iostreams/filter/bzip2.hpp")
    set(_Boost_ZLIB_HEADERS              "boost/iostreams/filter/zlib.hpp")
    find_package( Boost OPTIONAL_COMPONENTS bzip2 zlib)
    if( Boost_ZLIB_FOUND AND Boost_BZIP2_FOUND)
      set(classification_linked_libraries ${classification_linked_libraries}
        ${Boost_ZLIB_LIBRARY} ${Boost_BZIP2_LIBRARY})
    endif()
  endif()

  if (Boost_SERIALIZATION_FOUND AND Boost_IOSTREAMS_FOUND AND (NOT WIN32 OR Boost_ZLIB_FOUND))
  #message("p_klein_function_plugin.dll----------------------------")
    qt5_wrap_ui( classificationUI_FILES Classification_widget.ui Classification_advanced_widget.ui )
    polyhedron_demo_plugin(classification_plugin Classification_plugin Point_set_item_classification Cluster_classification Surface_mesh_item_classification ${classificationUI_FILES} KEYWORDS Classification)
    
    set(classification_linked_libraries ${classification_linked_libraries}
      ${Boost_SERIALIZATION_LIBRARY}
      ${Boost_IOSTREAMS_LIBRARY})
    
    target_link_libraries(classification_plugin PUBLIC scene_points_with_normal_item
      scene_polylines_item scene_polygon_soup_item scene_surface_mesh_item scene_selection_item scene_color_ramp ${classification_linked_libraries})
    add_dependencies(classification_plugin point_set_selection_plugin selection_plugin)
    target_compile_definitions(classification_plugin PUBLIC ${classification_compile_definitions})
  else()
    message(STATUS "NOTICE: Boost Serialization or IO Streams or ZLIB not found. Classification plugin won't be available.")
  endif()


else(EIGEN3_FOUND)
  message(STATUS "NOTICE: Eigen 3.1 (or greater) was not found. Classification plugin will not be available.")
endif()


