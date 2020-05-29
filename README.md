# 3D_Annotator_for_public
	3D Annotator for mesh
  
# Main entry point  
	MeshAnnotator  
  
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
  
# Compiling

## macOS

  1. install CGAL so that all its dependency are installed: `brew install cgal`
  1. `brew instal eigen`
  1. `brew install qt5`
  1. a modified version of CGAL is used, the one in the folder `./CGAL`, and you need to use that one: `export CGAL_DIR=/path/to/CGAL_FOLDER` in this folder
  1. `cd MeshAnnotator`
  1. `mkdir build && cd build`
  1. `cmake -DCMAKE_BUILD_TYPE=true ..`
  1. `make


# How to annotate data

Firstly, you need to load a mesh (.ply) and start the [3D Annotation] in [Operations] menu.

Then you need to click the 'selection' layer in 'Geometric Objects' to start annotation process. 

After you finished, do not forget to save your work.