#ifndef SMESH_TYPE_H
#define SMESH_TYPE_H

#include "properties.h"

#include <CGAL/Surface_mesh/Surface_mesh.h>
#include <CGAL/boost/graph/properties.h>
#include <CGAL/Cartesian.h>

#include <CGAL/Point_set_3.h>
#include <CGAL/Shape_detection/Region_growing/Region_growing.h>
#include <CGAL/Shape_detection/Region_growing/Region_growing_on_polygon_mesh.h>
#include <CGAL/Shape_detection/Region_growing/Region_growing_on_point_set.h>

typedef CGAL::Cartesian<double> Kernel;
typedef Kernel::Point_2 Point_2;
typedef Kernel::Point_3 Point_3;
typedef Kernel::Plane_3 Plane_3;
typedef Kernel::Vector_3 Vector_3;
typedef CGAL::Surface_mesh<Point_3> SMesh;
typedef boost::graph_traits<SMesh>::face_descriptor face_descriptor;
typedef boost::graph_traits<SMesh>::vertex_descriptor vertex_descriptor;
typedef boost::graph_traits<SMesh>::halfedge_descriptor halfedge_descriptor;

typedef boost::graph_traits<SMesh>::vertex_iterator vertex_iterator;
typedef boost::graph_traits<SMesh>::halfedge_iterator halfedge_iterator;
typedef boost::graph_traits<SMesh>::edge_iterator edge_iterator;
typedef boost::graph_traits<SMesh>::face_iterator face_iterator;
typedef std::pair<face_descriptor, std::vector<std::size_t>> face_vind;
typedef std::tuple<face_descriptor, std::vector<std::size_t>, std::vector<float>> face_vind_texcoord;

//For CGAL region growing: mesh
typedef SMesh::Face_range Face_range_cgal;
typedef CGAL::Shape_detection::Polygon_mesh::One_ring_neighbor_query<SMesh> Neighbor_query_mesh_cgal;
typedef CGAL::Shape_detection::Polygon_mesh::Least_squares_plane_fit_region<Kernel, SMesh> region_type_mesh_cgal;
typedef CGAL::Shape_detection::Polygon_mesh::Least_squares_plane_fit_sorting<Kernel, SMesh, Neighbor_query_mesh_cgal, Face_range_cgal> sorting_mesh_cgal;
typedef std::vector<std::size_t> region_for_cgal;
typedef std::vector<region_for_cgal> regions_for_cgal;
typedef region_type_mesh_cgal::Vertex_to_point_map Vertex_to_point_map_mesh_cgal;
typedef CGAL::Shape_detection::Region_growing<Face_range_cgal, Neighbor_query_mesh_cgal, region_type_mesh_cgal, typename sorting_mesh_cgal::Seed_map> Region_growing_mesh_cgal;

//For CGAL region growing: pcl
typedef CGAL::Point_set_3<Point_3> Point_range_cgal;
typedef Point_range_cgal::Point_map Point_3_cgal_map;
typedef Point_range_cgal::Vector_map Normal_3_cgal_map;
typedef CGAL::Shape_detection::Point_set::K_neighbor_query<Kernel, Point_range_cgal, Point_3_cgal_map> Neighbor_query_pcl_cgal;
typedef CGAL::Shape_detection::Point_set::Least_squares_plane_fit_region<Kernel, Point_range_cgal, Point_3_cgal_map, Normal_3_cgal_map> region_type_pcl_cgal;
typedef CGAL::Shape_detection::Region_growing<Point_range_cgal, Neighbor_query_pcl_cgal, region_type_pcl_cgal> Region_growing_pcl_cgal;

namespace boost {

template <typename P>
struct property_map<CGAL::Surface_mesh<P>, CGAL::vertex_selection_t>
{

  typedef typename boost::graph_traits<CGAL::Surface_mesh<P> >::vertex_descriptor vertex_descriptor;

  typedef typename CGAL::Surface_mesh<P>::template Property_map<vertex_descriptor, int> type;
  typedef type const_type;
};


template <typename P>
struct property_map<CGAL::Surface_mesh<P>, CGAL::face_selection_t>
{

  typedef typename boost::graph_traits<CGAL::Surface_mesh<P> >::face_descriptor face_descriptor;

  typedef typename CGAL::Surface_mesh<P>::template Property_map<face_descriptor, int> type;
  typedef type const_type;
};

} // namespace boost


namespace CGAL {

template <typename P, typename Property_tag>
struct Get_pmap_of_surface_mesh_ {
  typedef typename boost::property_map<Surface_mesh<P>, Property_tag >::type type;
};

#define CGAL_PROPERTY_SURFACE_MESH_RETURN_TYPE(Tag) \
  typename boost::lazy_disable_if<                      \
     boost::is_const<P>,                                \
     Get_pmap_of_surface_mesh_<P, Tag >                  \
   >::type

template <typename P>
CGAL_PROPERTY_SURFACE_MESH_RETURN_TYPE(CGAL::face_selection_t)
 inline get(CGAL::face_selection_t, Surface_mesh<P> & smesh)
{
 typedef typename boost::graph_traits<Surface_mesh<P> >::face_descriptor face_descriptor;
  return smesh. template add_property_map<face_descriptor,int>("f:selection").first;
}



template <typename P>
CGAL_PROPERTY_SURFACE_MESH_RETURN_TYPE(CGAL::vertex_selection_t)
inline get(CGAL::vertex_selection_t, Surface_mesh<P> & smesh)
{
  typedef typename boost::graph_traits<Surface_mesh<P> >::vertex_descriptor vertex_descriptor;
  return smesh. template add_property_map<vertex_descriptor,int>("v:selection").first;
}
#undef CGAL_PROPERTY_SURFACE_MESH_RETURN_TYPE
} // namespace CGAL

#endif // SMESH_TYPE_H
