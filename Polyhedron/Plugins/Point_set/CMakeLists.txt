include( polyhedron_demo_macros )
if(EIGEN3_FOUND)
  qt5_wrap_ui( point_set_normal_estimationUI_FILES Point_set_normal_estimation_plugin.ui)
  polyhedron_demo_plugin(point_set_normal_estimation_plugin Point_set_normal_estimation_plugin ${point_set_normal_estimationUI_FILES} KEYWORDS PointSetProcessing Classification)
  target_link_libraries(point_set_normal_estimation_plugin PUBLIC scene_points_with_normal_item scene_callback_signaler)

  polyhedron_demo_plugin(point_set_average_spacing_plugin Point_set_average_spacing_plugin KEYWORDS PointSetProcessing Classification)
  target_link_libraries(point_set_average_spacing_plugin PUBLIC scene_points_with_normal_item scene_callback_signaler)

else(EIGEN3_FOUND)
  message(STATUS "NOTICE: Eigen 3.1 (or greater) was not found. Surface reconstruction plugin will not be available.")
  message(STATUS "NOTICE: Eigen 3.1 (or greater) was not found. Normal estimation plugins will not be available.")
  message(STATUS "NOTICE: Eigen 3.1 (or greater) was not found. Smoothing plugin will not be available.")
  message(STATUS "NOTICE: Eigen 3.1 (or greater) was not found. Average spacing plugin will not be available.")
  message(STATUS "NOTICE: Eigen 3.1 (or greater) was not found. Feature detection plugin will not be available.")

endif()

  qt5_wrap_ui( ps_outliers_removal_UI_FILES Point_set_outliers_removal_plugin.ui)
  polyhedron_demo_plugin(point_set_outliers_removal_plugin Point_set_outliers_removal_plugin ${ps_outliers_removal_UI_FILES} KEYWORDS PointSetProcessing)
  target_link_libraries(point_set_outliers_removal_plugin PUBLIC scene_points_with_normal_item scene_callback_signaler)

  qt5_wrap_ui( point_set_selectionUI_FILES Point_set_selection_widget.ui)
  polyhedron_demo_plugin(point_set_selection_plugin Point_set_selection_plugin ${point_set_selectionUI_FILES} KEYWORDS PointSetProcessing Classification)
  target_link_libraries(point_set_selection_plugin PUBLIC scene_points_with_normal_item scene_polylines_item scene_edit_box_item)

  polyhedron_demo_plugin(merge_point_sets_plugin Merge_point_sets_plugin KEYWORDS PointSetProcessing Classification)
  target_link_libraries(merge_point_sets_plugin PUBLIC scene_points_with_normal_item)