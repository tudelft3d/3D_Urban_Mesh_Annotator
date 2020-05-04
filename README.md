# 3D_Annotator_for_public
	3D Annotator for mesh and point cloud  
  
# Main entry point  
	Polyhedron_3  
  
# Required Plugins  
	camera_positions_plugin  
	classification_plugin  
	selection_plugin  
	ply_plugin  
  
# Required libraries:  
	CGAL-4.13 ~ 4.14  
	Qt5.9  
	Eigen  
	Boost  
	zlib  
  
# Required sub-modules:  
QT:  
	Qt5Core  
	Qt5OpenGL  
	Qt5Svg    
	Qt5Widgets  
	Qt5Gui  
	Qt5Xml  
	Qt5Script  
	imageformats/qjpeg  
	platforms/qwindows  
  
boost:  
	boost_serialization  
	boost_iostreams  
	boost_zlib  
	boost_bzip2  
  
# Modified CGAL files:  
	.\CGAL-4.14\include\CGAL\IO\PLY_reader.h
	.\CGAL-4.14\include\CGAL\IO\PLY_writer.h
	.\CGAL-4.14\include\CGAL\IO\read_ply_points.h
	.\CGAL-4.14\include\CGAL\IO\write_ply_points.h
	.\CGAL-4.14\include\CGAL\Three\Three.h
	.\CGAL-4.14\include\CGAL\Three\Scene_item.h
	.\CGAL-4.14\include\CGAL\Three\Scene_interface.h
	.\CGAL-4.14\include\CGAL\Three\Scene_draw_interface.h
	.\CGAL-4.14\include\CGAL\Qt\camera_impl.h
