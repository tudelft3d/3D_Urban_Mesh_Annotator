
# UrbanMeshAnnotator

	Mesh annotator for urban scenes
  
## Build from source
### Windows with CMake 

  1. Install Eigen and Qt(>5.6)
  1. a modified version of CGAL is provided (folder `./CGAL/`), use it in CMake. 
  1. `cd src`
  1. `mkdir build && cd build`
  1. `cmake -DCMAKE_BUILD_TYPE=Release ..` to compile in release mode and not debug
  1. `make`
  1. `./UrbanMeshAnnotator`

### macOS with CMake 

  1. `brew instal eigen`
  1. `brew install qt5`
  1. a modified version of CGAL is used (folder `./CGAL/`), and you need to use that one: `export CGAL_DIR=/path/to/CGAL_FOLDER` 
  1. `cd src`
  1. `mkdir build && cd build`
  1. `cmake -DCMAKE_BUILD_TYPE=Release ..` to compile in release mode and not debug
  1. `make`
  1. `./UrbanMeshAnnotator`


## How to annotate data

See the instruction in Tutorial.md


# Main entry point  
	UrbanMeshAnnotator main.cpp  
  
# Required Plugins  
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
  