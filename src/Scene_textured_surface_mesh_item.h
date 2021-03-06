#ifndef SCENE_TEXTURED_SURFACE_MESH_ITEM_H
#define SCENE_TEXTURED_SURFACE_MESH_ITEM_H
#include <CGAL/Three/Scene_item.h>
#include <CGAL/Three/Viewer_interface.h>
#include "SMesh_type.h"
#include <iostream>
#include <vector>
#include "texture.h"

#ifdef scene_textured_item_EXPORTS
#  define SCENE_TEXTURED_SURFACE_MESH_ITEM_EXPORT Q_DECL_EXPORT
#else
#  define SCENE_TEXTURED_SURFACE_MESH_ITEM_EXPORT Q_DECL_IMPORT
#endif

struct Scene_textured_surface_mesh_item_priv;
// This class represents a textured polyhedron in the OpenGL scene
class SCENE_TEXTURED_SURFACE_MESH_ITEM_EXPORT Scene_textured_surface_mesh_item
    : public CGAL::Three::Scene_item {
  Q_OBJECT
public:
  Scene_textured_surface_mesh_item();
  //   Scene_textured_surface_mesh_item(const Scene_textured_surface_mesh_item&);
  Scene_textured_surface_mesh_item(const SMesh& p);
  Scene_textured_surface_mesh_item(SMesh* const p);
  ~Scene_textured_surface_mesh_item();

  Scene_textured_surface_mesh_item* clone() const;

  //Get texture information
  SMesh *mesh_in = new SMesh;
  SMesh::Property_map<halfedge_descriptor, std::pair<float, float> > uv_map;
  std::map<int, Kernel::Point_3> vertices_coords_tex;
  std::map<face_descriptor, std::vector<int>> face_vertex_inds_tex;
  std::map<face_descriptor, std::vector<float>> face_texcoord;
  QImage texture_image_in;

  // IO
  bool load(std::istream& in);
  bool save(std::ostream& out) const;

  // Function for displaying meta-data of the item
  virtual QString toolTip() const;

  // Indicate if rendering mode is supported
  virtual bool supportsRenderingMode(RenderingMode m) const { return (m != PointsPlusNormals && m != Points && m != Gouraud && m != ShadedPoints); }
  // Points/Wireframe/Flat/Gouraud OpenGL drawing in a display list
  void draw() const {}
  virtual void draw(CGAL::Three::Viewer_interface*) const;
  virtual void drawEdges() const {}
  virtual void drawEdges(CGAL::Three::Viewer_interface* viewer) const;

  // Get wrapped textured_polyhedron
  SMesh*       textured_face_graph();
  const SMesh* textured_face_graph() const;

  // Get dimensions
  bool isFinite() const { return true; }
  bool isEmpty() const;
  void compute_bbox() const;
  virtual void invalidateOpenGLBuffers();
  virtual void selection_changed(bool);
  void add_border_edges(std::vector<float> border_edges, bool is_opengl_4_3);

Q_SIGNALS:
  void selectionChanged();

protected:
  friend struct Scene_textured_surface_mesh_item_priv;
  Scene_textured_surface_mesh_item_priv* d;

}; // end class Scene_textured_surface_mesh_item

#endif // SCENE_TEXTURED_SURFACE_MESH_ITEM_H
