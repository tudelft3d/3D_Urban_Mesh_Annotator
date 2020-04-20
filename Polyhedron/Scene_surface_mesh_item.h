#ifndef CGAL_SCENE_SURFACE_MESH_ITEM_H
#define CGAL_SCENE_SURFACE_MESH_ITEM_H
//Defines the precision of the positions (for performance/precision sake)
#define CGAL_GL_DATA GL_FLOAT
#define cgal_gl_data float
#define CGAL_IS_FLOAT 1

#include "Scene_surface_mesh_item_config.h"
#include <CGAL/Three/Scene_zoomable_item_interface.h>
#include <CGAL/Three/Scene_print_item_interface.h>
#include <CGAL/Three/Scene_item_with_properties.h>

//***********************Weixiao Update*******************************//
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
#include <CGAL/IO/PLY_reader.h>
#include "Scene_textured_surface_mesh_item.h"
//*******************************************************************//	

#ifndef Q_MOC_RUN
#include "SMesh_type.h"
#endif

#include <CGAL/Three/Scene_item.h>
#include <CGAL/Three/Scene_item_rendering_helper.h>
#include <CGAL/Three/Viewer_interface.h>

#ifndef Q_MOC_RUN
#include <vector>
#include <boost/scoped_ptr.hpp>
#include <boost/array.hpp>
#endif

#include <QColor>

#include "properties.h"


class QSlider;
struct Scene_surface_mesh_item_priv;
/******************Ziqian*********************/
typedef std::size_t seg_id;


class seg_boundary_edge_info;
class Segment {
public:
	std::set<face_descriptor> faces_included;
	std::set<seg_boundary_edge_info> boundary_edges;
	std::size_t id;
	Segment(std::vector<face_descriptor> faces) {
		BOOST_FOREACH(face_descriptor fd, faces) {
			faces_included.insert(fd);
		}
	}
	Segment() {}
};

class seg_boundary_edge_info {
public:
	Segment** adjecent_segs;
	void set_adjecent_segs(Segment* s1, Segment* s2);
	Segment* get_adjecent_segs(Segment* s);
};

/*********************************************/

class SCENE_SURFACE_MESH_ITEM_EXPORT Scene_surface_mesh_item
	: public CGAL::Three::Scene_item_rendering_helper,
	public CGAL::Three::Scene_item_with_properties,
	public CGAL::Three::Scene_zoomable_item_interface,
	public CGAL::Three::Scene_print_item_interface
{
	Q_INTERFACES(CGAL::Three::Scene_print_item_interface)
		Q_PLUGIN_METADATA(IID "com.geometryfactory.PolyhedronDemo.PrintInterface/1.0")
		Q_OBJECT
		Q_INTERFACES(CGAL::Three::Scene_zoomable_item_interface)
		Q_PLUGIN_METADATA(IID "com.geometryfactory.PolyhedronDemo.ZoomInterface/1.0")

public:
	typedef SMesh Face_graph;
	typedef SMesh::Property_map<vertex_descriptor, int> Vertex_selection_map;
	typedef SMesh::Property_map<face_descriptor, int> Face_selection_map;
	Scene_surface_mesh_item();
	// Takes ownership of the argument.
	Scene_surface_mesh_item(SMesh*);
	Scene_surface_mesh_item(SMesh);
	Scene_surface_mesh_item(const Scene_surface_mesh_item& other);

	~Scene_surface_mesh_item();


	Scene_surface_mesh_item* clone() const Q_DECL_OVERRIDE;
	void draw(CGAL::Three::Viewer_interface *) const Q_DECL_OVERRIDE;
	void drawEdges(CGAL::Three::Viewer_interface *) const Q_DECL_OVERRIDE;
	/******************************Ziqian*********************************/
	void Scene_surface_mesh_item::drawEdgesShowingSegBoundary(CGAL::Three::Viewer_interface* viewer) const;
	/*********************************************************************/
	void drawPoints(CGAL::Three::Viewer_interface *) const Q_DECL_OVERRIDE;

	bool supportsRenderingMode(RenderingMode m) const Q_DECL_OVERRIDE;
	bool isFinite() const Q_DECL_OVERRIDE { return true; }
	bool isEmpty() const Q_DECL_OVERRIDE;
	Bbox bbox() const Q_DECL_OVERRIDE;
	QString toolTip() const Q_DECL_OVERRIDE;
	void copyProperties(Scene_item *) Q_DECL_OVERRIDE;

	QMenu* contextMenu() Q_DECL_OVERRIDE;

	void setItemIsMulticolor(bool);
	//to be called before invalidate() to enable or disable the recomputation 
	//of the colors_ vector to scale on min_patch value. 
	// For example, the Mesh_segmentation_plugin computes the colors_
	// vector itself, so it must set recompute_colors to false to avoid 
	// having it ovewritten 
	// in the code of this item.
	void computeItemColorVectorAutomatically(bool);
	bool isItemMulticolor();
	bool hasPatchIds();

	Vertex_selection_map vertex_selection_map();
	Face_selection_map face_selection_map();

	std::vector<QColor>& color_vector();
	void show_feature_edges(bool);
	SMesh* polyhedron();
	const SMesh* polyhedron() const;

	Face_graph*       face_graph() { return polyhedron(); }
	const Face_graph* face_graph() const { return polyhedron(); }

	void invalidate_aabb_tree();
	void invalidateOpenGLBuffers()Q_DECL_OVERRIDE;
	void invalidate(Gl_data_names name);


	void compute_bbox()const Q_DECL_OVERRIDE;
	void standard_constructor(SMesh *sm);
	bool save(std::ostream& out) const;
	bool save_obj(std::ostream& out) const;
	bool load_obj(std::istream& in);

	//***********************Weixiao Update wirte ply attribute*******************************//
	bool write_ply_mesh(std::ostream& stream, bool binary) const;
	// Gets PLY comments (empty if point set not originated from PLY input)
	std::string& comments();
	const std::string& comments() const;


	std::map<vertex_descriptor, QColor> vertex_color;
	std::map<face_descriptor, int> face_label;
	std::map<face_descriptor, QColor> face_color;
	std::vector<std::string> face_label_comment;
	std::map<face_descriptor, std::vector<float>> face_texcoord;
	std::map<face_descriptor, int> face_textureid;
	std::vector<std::string> texture_name;
	std::map<face_descriptor, float> label_probabilities;
	std::map<face_descriptor, int> face_segment_id;

	std::string file_path;
	//*******************************************************************//	
	//***********************Ziqian*******************************//
	std::map<face_descriptor, bool> face_shown;
	std::map<face_descriptor, QColor> face_color_backup;

	typedef boost::graph_traits<SMesh>::edge_descriptor edge_descriptor;

	std::map<seg_id, Segment> segments;
	std::map<edge_descriptor, seg_boundary_edge_info> boundary_info;
	// record the informations about segments into the map "segments" based on the informations
	// in face_segment_id
	// this process is finished in PLY reading only, after the scene_surface_mesh is built.
	void computeSegments();
	bool segmentBoundryShow = true;
	RenderingMode m_RMode;
	//************************************************************//
	//statistics
	enum STATS
	{
		NB_VERTICES = 0,
		NB_CONNECTED_COMPOS,
		NB_BORDER_EDGES,
		IS_PURE_TRIANGLE,
		IS_PURE_QUAD,
		NB_DEGENERATED_FACES,
		HOLES,
		AREA,
		VOLUME,
		SELFINTER,
		NB_FACETS,
		MIN_AREA,
		MAX_AREA,
		MED_AREA,
		MEAN_AREA,
		MIN_ALTITUDE,
		MIN_ASPECT_RATIO,
		MAX_ASPECT_RATIO,
		MEAN_ASPECT_RATIO,
		GENUS,
		NB_EDGES,
		MIN_LENGTH,
		MAX_LENGTH,
		MID_LENGTH,
		MEAN_LENGTH,
		NB_NULL_LENGTH,
		MIN_ANGLE,
		MAX_ANGLE,
		MEAN_ANGLE
	};

	bool has_stats()const Q_DECL_OVERRIDE { return true; }
	QString computeStats(int type)Q_DECL_OVERRIDE;
	CGAL::Three::Scene_item::Header_data header() const Q_DECL_OVERRIDE;
	//zoomable interface
	void zoomToPosition(const QPoint &point, CGAL::Three::Viewer_interface *)const Q_DECL_OVERRIDE;
	//print_interface
	void printPrimitiveId(QPoint point, CGAL::Three::Viewer_interface*viewer)Q_DECL_OVERRIDE;
	bool printVertexIds(CGAL::Three::Viewer_interface*)const Q_DECL_OVERRIDE;
	bool printEdgeIds(CGAL::Three::Viewer_interface*)const Q_DECL_OVERRIDE;
	bool printFaceIds(CGAL::Three::Viewer_interface*)const Q_DECL_OVERRIDE;
	void printAllIds(CGAL::Three::Viewer_interface*) Q_DECL_OVERRIDE;
	bool shouldDisplayIds(CGAL::Three::Scene_item *current_item) const Q_DECL_OVERRIDE;
	bool testDisplayId(double x, double y, double z, CGAL::Three::Viewer_interface*)const Q_DECL_OVERRIDE;
	float alpha() const Q_DECL_OVERRIDE;
	void setAlpha(int alpha) Q_DECL_OVERRIDE;
	QSlider* alphaSlider();
	void computeElements() const Q_DECL_OVERRIDE;
	/************************Ziqian && Weixiao ************************/
	// Go through all the edges, and pick out all the boundary edge based on the infomations
	// in face_segment_id
	int updateSegmentId(std::map<face_descriptor, bool> &);

	void computeSegmentBoundary();

	void get_connected_faces(face_descriptor fd, std::vector<face_descriptor>& connected_faces);

	void emphasize_present_segment(seg_id seg);

	void unemphasize();

	void addChosenSegment(seg_id id);
	/******************************************************/

Q_SIGNALS:
	void item_is_about_to_be_changed();
	void selection_done();
	void selected_vertex(void*);
	void selected_facet(void*);
	void selected_edge(void*);
	void selected_halfedge(void*);
	/*******************Ziqian**********************/
	void selected_segment(void*);
	void changeColor();
	/***********************************************/

public Q_SLOTS:
	void itemAboutToBeDestroyed(Scene_item *) Q_DECL_OVERRIDE;
	virtual void selection_changed(bool) Q_DECL_OVERRIDE;

	void select(double orig_x,
		double orig_y,
		double orig_z,
		double dir_x,
		double dir_y,
		double dir_z) Q_DECL_OVERRIDE;

	bool intersect_face(double orig_x,
		double orig_y,
		double orig_z,
		double dir_x,
		double dir_y,
		double dir_z,
		const face_descriptor &f);
	void resetColors();
	void showVertices(bool);
	void showEdges(bool);
	void showFaces(bool);
	void showPrimitives(bool);
	void zoomToId();

protected:
	friend struct Scene_surface_mesh_item_priv;
	Scene_surface_mesh_item_priv* d;
};

#endif /* CGAL_SCENE_SURFACE_MESH_ITEM_H */
