include( polyhedron_demo_macros )

qt5_wrap_ui( selectionUI_FILES Selection_widget.ui)
polyhedron_demo_plugin(selection_plugin Selection_plugin ${selectionUI_FILES} KEYWORDS PolygonMesh IO)
target_link_libraries(selection_plugin PUBLIC scene_selection_item scene_polylines_item)