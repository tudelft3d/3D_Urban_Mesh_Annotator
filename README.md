
# Urban Mesh Annotation Tool

Mesh annotation tool for labelling urban scenes. 
Before you import your model, please check your input mesh is in ascii *.ply format.
Note that non-manifold meshes are not supported in this tool. 
If you still want to use it, please use [MeshLab](https://www.meshlab.net/) to repair it first.
For more information, please visit our [project website](https://3d.bk.tudelft.nl/projects/meshannotation/).

## Citation

If you use it in a scientific work, we kindly ask you to cite it:

<div class="filteredelement"><strong> SUM: A Benchmark Dataset of Semantic Urban Meshes </strong>. Weixiao Gao, Liangliang Nan, Bas Boom and Hugo Ledoux<em> arXiv preprint arXiv:2103.00355</em>. 2021 <br/><a href="https://arxiv.org/abs/2103.00355"><i class="fas fa-external-link-alt"></i> PDF</a> <a href="#myref" data-toggle="collapse"><i class="fas fa-caret-square-down"></i> BibTeX</a> <div id="myref" class="collapse" tabindex="-1"><pre class="bibtex">@misc{sum2021,
author = {Weixiao Gao and Liangliang Nan and Bas Boom and Hugo Ledoux},
title = {SUM: A Benchmark Dataset of Semantic Urban Meshes},
year={2021},
eprint={2103.00355},
archivePrefix={arXiv},
}
</pre></div></div>

 
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
  
