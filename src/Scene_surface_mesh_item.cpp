#include "Scene_surface_mesh_item.h"

#include "texture.h"
#include "Scene_textured_surface_mesh_item.h"

#include "Color_map.h"

#ifndef Q_MOC_RUN
#include <boost/graph/properties.hpp>
#include <boost/graph/graph_traits.hpp>
#endif

#include <QOpenGLShaderProgram>
#include <QInputDialog>
#include <QOpenGLBuffer>
#include <QApplication>
#include <QVariant>
#include <QMessageBox>
#include <QMenu>
#include <QWidgetAction>
#include <QSlider>
#include <QOpenGLFramebufferObject>

#ifndef Q_MOC_RUN
#include <CGAL/Surface_mesh.h>
#include <CGAL/Surface_mesh/IO.h>
#include <CGAL/intersections.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>

#include <CGAL/Polygon_mesh_processing/connected_components.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/Polygon_mesh_processing/self_intersections.h>
#include <CGAL/Polygon_mesh_processing/shape_predicates.h>
#include <CGAL/Polygon_mesh_processing/orient_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
#include "triangulate_primitive.h"

#include <CGAL/IO/File_writer_wavefront.h>
#include <CGAL/IO/generic_copy_OFF.h>
#include <CGAL/IO/PLY_reader.h>
#include <CGAL/IO/PLY_writer.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/statistics_helpers.h>

#include <CGAL/Image_3.h>
#include <CGAL/ImageIO.h>
#include <QGraphicsScene>
#include <QGraphicsItem>
#include <QPen>

#include <CGAL/Three/Buffer_objects.h>
#include <CGAL/Three/Triangle_container.h>
#include <CGAL/Three/Edge_container.h>
#include <CGAL/Three/Point_container.h>
#include <QMenu>
#include "id_printing.h"
#endif


typedef CGAL::Three::Triangle_container Tri;
typedef CGAL::Three::Edge_container Ed;
typedef CGAL::Three::Point_container Pt;
typedef CGAL::Three::Viewer_interface VI;

using namespace CGAL::Three;

typedef Kernel::Point_3  Local_point;
typedef Kernel::Vector_3 Local_vector;
//modified from Buffer_for_vao.h
///adds `acolor` RGB components to `buffer`
static void add_color_in_buffer(const CGAL::Color& acolor, std::vector<float>& buffer)
{
	buffer.push_back((float)acolor.red() / (float)255);
	buffer.push_back((float)acolor.green() / (float)255);
	buffer.push_back((float)acolor.blue() / (float)255);
}

/// adds `kp` coordinates to `buffer`
template<typename KPoint>
static void add_point_in_buffer(const KPoint& kp, std::vector<float>& buffer)
{
	buffer.push_back(kp.x());
	buffer.push_back(kp.y());
	buffer.push_back(kp.z());
}

/// adds `kv` coordinates to `buffer`
template<typename KVector>
static void add_normal_in_buffer(const KVector& kv, std::vector<float>& buffer)
{
	buffer.push_back(kv.x());
	buffer.push_back(kv.y());
	buffer.push_back(kv.z());
}

static bool is_facet_convex(const std::vector<Local_point>& facet,
	const Local_vector& normal)
{
	Kernel::Orientation orientation, local_orientation;
	std::size_t id = 0;
	do
	{
		const Local_point& S = facet[id];
		const Local_point& T = facet[(id + 1 == facet.size()) ? 0 : id + 1];
		Local_vector V1 = Local_vector((T - S).x(), (T - S).y(), (T - S).z());
		const Local_point& U = facet[(id + 2 >= facet.size()) ? id + 2 - facet.size() : id + 2];
		Local_vector V2 = Local_vector((U - T).x(), (U - T).y(), (U - T).z());

		orientation = Kernel::Orientation_3()(V1, V2, normal);
		// Is it possible that orientation==COPLANAR ? Maybe if V1 or V2 is very small ?
	} while (++id != facet.size() &&
		(orientation == CGAL::COPLANAR || orientation == CGAL::ZERO));

	//Here, all orientations were COPLANAR. Not sure this case is possible,
	// but we stop here.
	if (orientation == CGAL::COPLANAR || orientation == CGAL::ZERO)
	{
		return false;
	}

	// Now we compute convexness
	for (id = 0; id < facet.size(); ++id)
	{
		const Local_point& S = facet[id];
		const Local_point& T = facet[(id + 1 == facet.size()) ? 0 : id + 1];
		Local_vector V1 = Local_vector((T - S).x(), (T - S).y(), (T - S).z());

		const Local_point& U = facet[(id + 2 >= facet.size()) ? id + 2 - facet.size() : id + 2];
		Local_vector V2 = Local_vector((U - T).x(), (U - T).y(), (U - T).z());

		local_orientation = Kernel::Orientation_3()(V1, V2, normal);

		if (local_orientation != CGAL::ZERO && local_orientation != orientation)
		{
			return false;
		}
	}
	return true;
}

//for get facet neighbors

//Used to triangulate the AABB_Tree
class Primitive
{
public:
	// types
	typedef face_descriptor Id; // Id type
	typedef Point_3 Point; // point type
	typedef Kernel::Triangle_3 Datum; // datum type

private:
	// member data
	Id m_it; // iterator
	Datum m_datum; // 3D triangle

	// constructor
public:
	Primitive() {}
	Primitive(Datum triangle, Id it)
		: m_it(it), m_datum(triangle)
	{
	}
public:
	Id& id() { return m_it; }
	const Id& id() const { return m_it; }
	Datum& datum() { return m_datum; }
	const Datum& datum() const { return m_datum; }

	/// Returns a point on the primitive
	Point reference_point() const { return m_datum.vertex(0); }
};


typedef CGAL::AABB_traits<Kernel, Primitive> AABB_traits;
typedef CGAL::AABB_tree<AABB_traits> Input_facets_AABB_tree;

struct Scene_surface_mesh_item_priv {

	typedef Kernel::Point_3 Point;
	typedef CGAL::Surface_mesh<Point> SMesh;
	typedef boost::graph_traits<SMesh>::face_descriptor face_descriptor;
	
	typedef boost::graph_traits<SMesh>::vertex_descriptor vertex_descriptor;


	typedef std::vector<QColor> Color_vector;

	Scene_surface_mesh_item_priv(const Scene_surface_mesh_item& other, Scene_surface_mesh_item* parent) :
		smesh_(new SMesh(*other.d->smesh_)),
		idx_data_(other.d->idx_data_),
		idx_edge_data_(other.d->idx_edge_data_),

		idx_boundary_edge_data_(other.d->idx_boundary_edge_data_)
	{
		item = parent;
		item->setTriangleContainer(1, new Triangle_container(VI::PROGRAM_WITH_LIGHT,
			false));
		item->setTriangleContainer(0, new Triangle_container(VI::PROGRAM_WITH_LIGHT,
			true));
		
		item->setTriangleContainer(2, new Triangle_container(VI::PROGRAM_WITH_LIGHT,
			false));

		item->setEdgeContainer(2, new Edge_container(VI::PROGRAM_SOLID_WIREFRAME,
			true));

		item->setEdgeContainer(1, new Edge_container(VI::PROGRAM_NO_SELECTION,
			true));
		item->setEdgeContainer(0, new Edge_container(VI::PROGRAM_WITHOUT_LIGHT,
			true));
		item->setPointContainer(0, new Point_container(VI::PROGRAM_NO_SELECTION,
			false));
		item->getEdgeContainer(0)->setFrameMatrix(QMatrix4x4());
		has_feature_edges = false;
		invalidate_stats();
		vertices_displayed = false;
		edges_displayed = false;
		faces_displayed = false;
		all_displayed = false;

		//alphaSlider = NULL;
		alphaSlider = new QSlider(::Qt::Horizontal);
		alphaSlider->setMinimum(0);
		alphaSlider->setMaximum(255);
		alphaSlider->setValue(120);

		has_vcolors = false;
		has_fcolors = false;
		item->setProperty("classname", QString("surface_mesh"));

		//pointSizeSlider = NULL;
		texture_item = new Scene_textured_surface_mesh_item();
		current_rendering_mode = other.renderingMode();
	}

	Scene_surface_mesh_item_priv(SMesh* sm, Scene_surface_mesh_item* parent) :
		smesh_(sm)
	{
		item = parent;
		item->setTriangleContainer(1, new Triangle_container(VI::PROGRAM_WITH_LIGHT,
			false));
		item->setTriangleContainer(0, new Triangle_container(VI::PROGRAM_WITH_LIGHT,
			true));
		
		item->setTriangleContainer(2, new Triangle_container(VI::PROGRAM_WITH_LIGHT,
			false));

		item->setEdgeContainer(2, new Edge_container(VI::PROGRAM_SOLID_WIREFRAME,
			true));

		item->setEdgeContainer(1, new Edge_container(VI::PROGRAM_NO_SELECTION,
			true));
		item->setEdgeContainer(0, new Edge_container(VI::PROGRAM_WITHOUT_LIGHT,
			true));
		item->setPointContainer(0, new Point_container(VI::PROGRAM_WITHOUT_LIGHT,
			false));

		has_feature_edges = false;
		invalidate_stats();
		vertices_displayed = false;
		edges_displayed = false;
		faces_displayed = false;
		all_displayed = false;
		//alphaSlider = NULL;
		alphaSlider = new QSlider(::Qt::Horizontal);
		alphaSlider->setMinimum(0);
		alphaSlider->setMaximum(255);
		alphaSlider->setValue(120);

		has_vcolors = false;
		has_fcolors = false;
		item->setProperty("classname", QString("surface_mesh"));

		//pointSizeSlider = NULL;
		texture_item = new Scene_textured_surface_mesh_item();
		current_rendering_mode = parent->renderingMode();
		QApplication::restoreOverrideCursor();
	}

	~Scene_surface_mesh_item_priv()
	{
		if (alphaSlider)
			delete alphaSlider;
		//if (pointSizeSlider)
		//	delete pointSizeSlider;
		delete[]texture_items;
		if (smesh_)
		{
			delete smesh_;
			smesh_ = NULL;
		}
	}

	void killIds();
	void fillTargetedIds(const face_descriptor& selected_fh,
		const Kernel::Point_3& point_under,
		CGAL::Three::Viewer_interface* viewer,
		const CGAL::qglviewer::Vec& offset);

	void initialize_colors() const;
	void invalidate_stats();
	void initializeBuffers(CGAL::Three::Viewer_interface*) const;
	void addFlatData(Point, Kernel::Vector_3, CGAL::Color*, Scene_item_rendering_helper::Gl_data_names name) const;
	void* get_aabb_tree();
	QList<Kernel::Triangle_3> triangulate_primitive(face_descriptor fit,
		Kernel::Vector_3 normal);

	bool check_neighbor_faces(const vertex_descriptor v_source, const vertex_descriptor v_target) const;
	void computeSegmentBoundary();
	void addSelectedFlatData(Point p, Kernel::Vector_3 n, CGAL::Color* c, Scene_item_rendering_helper::Gl_data_names name) const;

	//! \brief triangulate_facet Triangulates a facet.
	//! \param fd a face_descriptor of the facet that needs to be triangulated.
	//! \param fnormals a property_map containing the normals of the mesh.
	//! \param fcolors a property_map containing the colors of the mesh
	//! \param im a property_map containing the indices of the vertices of the mesh
	//! \param index if true, the function will fill the index vector. If false, the function will
	//! fill the flat data vectors.
	void
		triangulate_facet(face_descriptor fd,
			SMesh::Property_map<face_descriptor, Kernel::Vector_3>* fnormals,
			SMesh::Property_map<face_descriptor, CGAL::Color>* fcolors,
			boost::property_map< SMesh, boost::vertex_index_t >::type* im,
			Scene_item_rendering_helper::Gl_data_names name,
			bool index) const;
	void triangulate_convex_facet(face_descriptor fd,
		SMesh::Property_map<face_descriptor, Kernel::Vector_3>* fnormals,
		SMesh::Property_map<face_descriptor, CGAL::Color>* fcolors,
		boost::property_map< SMesh, boost::vertex_index_t >::type* im,
		Scene_item_rendering_helper::Gl_data_names name,
		bool index) const;
	void compute_elements(Scene_item_rendering_helper::Gl_data_names name) const;
	void compute_selected_elements(Scene_item_rendering_helper::Gl_data_names name) const;
	void checkFloat() const;
	TextListItem* textVItems;
	TextListItem* textEItems;
	TextListItem* textFItems;
	mutable bool vertices_displayed;
	mutable bool edges_displayed;
	mutable bool faces_displayed;
	mutable bool all_displayed;
	mutable QList<double> text_ids;
	mutable std::vector<TextItem*> targeted_id;

	mutable bool has_fpatch_id;
	mutable bool has_feature_edges;
	mutable bool floated;
	mutable bool has_vcolors;
	mutable bool has_fcolors;
	SMesh* smesh_;
	std::string m_comments;
	Scene_textured_surface_mesh_item* texture_item;
	Scene_textured_surface_mesh_item** texture_items;
	bool is_texture_initialized = false;
	RenderingMode current_rendering_mode;
	mutable bool is_filled;
	mutable bool isinit;
	mutable std::vector<unsigned int> idx_data_, idx_data_log;
	mutable std::size_t idx_data_size, idx_data_log_size;
	mutable std::map<unsigned int, unsigned int> current_indices; //map im values to ghosts-free values
	mutable std::vector<unsigned int> idx_edge_data_, idx_edge_data_log;
	mutable std::size_t idx_edge_data_size, idx_edge_data_log_size;
	mutable std::vector<unsigned int> idx_feature_edge_data_, idx_feature_edge_data_log;
	mutable std::size_t idx_feature_edge_data_size, idx_feature_edge_data_log_size;
	mutable std::vector<unsigned int> idx_boundary_edge_data_, idx_boundary_edge_data_log;
	mutable std::size_t idx_boundary_edge_data_size, idx_boundary_edge_data_log_size;

	mutable std::vector<cgal_gl_data> selected_flat_vertices, selected_flat_vertices_log;
	mutable std::size_t selected_flat_vertices_size, selected_flat_vertices_log_size;
	mutable std::vector<cgal_gl_data> selected_flat_normals, selected_flat_normals_log;
	mutable std::vector<cgal_gl_data> selected_f_colors, selected_f_colors_log;
	mutable std::vector<cgal_gl_data> smooth_vertices, smooth_vertices_log;
	mutable std::vector<cgal_gl_data> smooth_normals, smooth_normals_log;
	mutable std::vector<cgal_gl_data> flat_vertices, flat_vertices_log;
	mutable std::size_t flat_vertices_size, flat_vertices_log_size;
	mutable std::vector<cgal_gl_data> flat_normals, flat_normals_log;
	mutable std::vector<cgal_gl_data> f_colors, f_colors_log;
	mutable std::vector<cgal_gl_data> v_colors, v_colors_log;

	mutable std::map<face_descriptor, int> f_begin_map;
	mutable int choosed_segment_id = -1;
	mutable SMesh::Property_map<face_descriptor, Kernel::Vector_3 > fnormals_log;
	mutable bool is_merged_batch_ = false;
	mutable std::map<face_descriptor, std::vector<int>> face_non_duplicated_vertices_map_;
	mutable std::map<std::pair<int, int>, std::vector<face_descriptor> > st_faces_map_;
	mutable std::map<int, int> old_new_duplicate_verts_map_;
	mutable std::vector<QImage> texture_images_;
	mutable QOpenGLShaderProgram* program;
	Scene_surface_mesh_item* item;

	mutable SMesh::Property_map<face_descriptor, int> fpatch_id_map;
	mutable int min_patch_id;
	mutable SMesh::Property_map<vertex_descriptor, int> v_selection_map;
	mutable SMesh::Property_map<face_descriptor, int> f_selection_map;
	mutable SMesh::Property_map<boost::graph_traits<SMesh>::edge_descriptor, bool> e_is_feature_map;

	mutable Color_vector colors_;
	double volume, area;
	unsigned int number_of_null_length_edges;
	unsigned int number_of_degenerated_faces;
	int genus;
	bool self_intersect;
	mutable QSlider* alphaSlider;
	//mutable QSlider* pointSizeSlider;
	std::set<std::size_t> chosen_segments;
};

const char* aabb_property_name = "Scene_surface_mesh_item aabb tree";
Scene_surface_mesh_item::Scene_surface_mesh_item()
{
	d = new Scene_surface_mesh_item_priv(new SMesh(), this);
	d->floated = false;
	setRenderingMode(CGAL::Three::Three::defaultSurfaceMeshRenderingMode());
	d->checkFloat();
	d->textVItems = new TextListItem(this);
	d->textEItems = new TextListItem(this);
	d->textFItems = new TextListItem(this);
	are_buffers_filled = false;
	invalidate(ALL);
}

Scene_surface_mesh_item::Scene_surface_mesh_item(const Scene_surface_mesh_item& other)
{
	d = new Scene_surface_mesh_item_priv(other, this);
	setRenderingMode(CGAL::Three::Three::defaultSurfaceMeshRenderingMode());
	d->floated = false;
	d->checkFloat();
	d->textVItems = new TextListItem(this);
	d->textEItems = new TextListItem(this);
	d->textFItems = new TextListItem(this);
	
	are_buffers_filled = false;
	invalidate(ALL);
}

void Scene_surface_mesh_item::standard_constructor(SMesh* sm)
{
	d = new Scene_surface_mesh_item_priv(sm, this);
	d->floated = false;
	setRenderingMode(CGAL::Three::Three::defaultSurfaceMeshRenderingMode());
	d->checkFloat();
	d->textVItems = new TextListItem(this);
	d->textEItems = new TextListItem(this);
	d->textFItems = new TextListItem(this);
	are_buffers_filled = false;
	invalidate(ALL);
	is_first_construct = false;
	//get normal
	sm->add_property_map<face_descriptor, Kernel::Vector_3 >("f:normal").first;
	sm->property_map<face_descriptor, Kernel::Vector_3 >("f:normal").first = d->fnormals_log;
}

void Scene_surface_mesh_item::update_priv()
{
	d->is_merged_batch_ = this->is_merged_batch;
	d->face_non_duplicated_vertices_map_ = this->face_non_duplicated_vertices_map;
	d->st_faces_map_ = this->st_faces_map;
	d->old_new_duplicate_verts_map_ = this->old_new_duplicate_verts_map;
}

Scene_surface_mesh_item::Scene_surface_mesh_item(SMesh* sm)
{
	standard_constructor(sm);
}

Scene_surface_mesh_item::Scene_surface_mesh_item(SMesh sm)
{
	standard_constructor(new SMesh(sm));
}

Scene_surface_mesh_item* Scene_surface_mesh_item::clone() const
{
	return new Scene_surface_mesh_item(*this);
}

Scene_surface_mesh_item::Vertex_selection_map
Scene_surface_mesh_item::vertex_selection_map()
{
	if (!d->v_selection_map) {
		d->v_selection_map = d->smesh_->add_property_map<vertex_descriptor, int>("v:selection").first;
	}
	return d->v_selection_map;
}

Scene_surface_mesh_item::Face_selection_map
Scene_surface_mesh_item::face_selection_map()
{
	if (!d->f_selection_map) {
		d->f_selection_map = d->smesh_->add_property_map<face_descriptor, int>("f:selection").first;
	}
	return d->f_selection_map;
}

std::vector<QColor>&
Scene_surface_mesh_item::color_vector()
{
	return d->colors_;
}

void Scene_surface_mesh_item_priv::addFlatData(Point p, Kernel::Vector_3 n, CGAL::Color* c, Scene_item_rendering_helper::Gl_data_names name) const
{
	const CGAL::qglviewer::Vec offset = static_cast<CGAL::Three::Viewer_interface*>(CGAL::QGLViewer::QGLViewerPool().first())->offset();
	if (name.testFlag(Scene_item_rendering_helper::GEOMETRY))
	{
		flat_vertices.push_back((cgal_gl_data)(p.x() + offset[0]));
		flat_vertices.push_back((cgal_gl_data)(p.y() + offset[1]));
		flat_vertices.push_back((cgal_gl_data)(p.z() + offset[2]));
	}
	if (name.testFlag(Scene_item_rendering_helper::NORMALS))
	{
		flat_normals.push_back((cgal_gl_data)n.x());
		flat_normals.push_back((cgal_gl_data)n.y());
		flat_normals.push_back((cgal_gl_data)n.z());
	}
	if (c != NULL && name.testFlag(Scene_item_rendering_helper::COLORS))
	{
		f_colors.push_back((float)c->red() / 255);
		f_colors.push_back((float)c->green() / 255);
		f_colors.push_back((float)c->blue() / 255);
	}
}

//use for mesh with duplicate vertices, check if the interior mesh boundary is segment border
bool Scene_surface_mesh_item_priv::check_neighbor_faces(const vertex_descriptor v_source, const vertex_descriptor v_target) const
{
	int v_source_ind = v_source.idx();
	int v_target_ind = v_target.idx();

	auto it_source = item->old_new_duplicate_verts_map.find(v_source.idx());
	if (it_source != item->old_new_duplicate_verts_map.end())
		v_source_ind = item->old_new_duplicate_verts_map[v_source.idx()];

	auto it_target = item->old_new_duplicate_verts_map.find(v_target.idx());
	if (it_target != item->old_new_duplicate_verts_map.end())
		v_target_ind = item->old_new_duplicate_verts_map[v_target.idx()];

	if (item->st_faces_map[std::make_pair(v_source_ind, v_target_ind)].size() > 1)
	{
		//for normal edge with only two adjacent faces
		if (item->st_faces_map[std::make_pair(v_source_ind, v_target_ind)].size() == 2)
		{
			face_descriptor fd1 = item->st_faces_map[std::make_pair(v_source_ind, v_target_ind)][0];
			face_descriptor fd2 = item->st_faces_map[std::make_pair(v_source_ind, v_target_ind)][1];
			std::size_t seg_id_1 = item->face_segment_id[fd1];
			std::size_t seg_id_2 = item->face_segment_id[fd2];
			if (seg_id_1 != seg_id_2)
				return true;
			else
				return false;
		}
		else
		{
		//for edge with more than two adjacent faces
			std::set<int> seg_ids;
			for (auto fd_i : item->st_faces_map[std::make_pair(v_source_ind, v_target_ind)])
			{
				seg_ids.insert(item->face_segment_id[fd_i]);
			}
			if (seg_ids.size() > 1)
				return true;
			else
				return false;
		}
	}
	else
		return false;
}

void Scene_surface_mesh_item_priv::addSelectedFlatData(Point p, Kernel::Vector_3 n, CGAL::Color* c, Scene_item_rendering_helper::Gl_data_names name) const
{
	const CGAL::qglviewer::Vec offset = static_cast<CGAL::Three::Viewer_interface*>(CGAL::QGLViewer::QGLViewerPool().first())->offset();
	if (name.testFlag(Scene_item_rendering_helper::GEOMETRY))
	{
		selected_flat_vertices.push_back((cgal_gl_data)(p.x() + offset[0]));
		selected_flat_vertices.push_back((cgal_gl_data)(p.y() + offset[1]));
		selected_flat_vertices.push_back((cgal_gl_data)(p.z() + offset[2]));
	}
	if (name.testFlag(Scene_item_rendering_helper::NORMALS))
	{
		selected_flat_normals.push_back((cgal_gl_data)n.x());
		selected_flat_normals.push_back((cgal_gl_data)n.y());
		selected_flat_normals.push_back((cgal_gl_data)n.z());
	}
	if (c != NULL && name.testFlag(Scene_item_rendering_helper::COLORS))
	{
		selected_f_colors.push_back((float)c->red() / 255);
		selected_f_colors.push_back((float)c->green() / 255);
		selected_f_colors.push_back((float)c->blue() / 255);
	}
}

void Scene_surface_mesh_item_priv::compute_selected_elements(Scene_item_rendering_helper::Gl_data_names name)const
{
	QApplication::setOverrideCursor(Qt::WaitCursor);

	if (!item->selected_facets_for_annotation.empty() ||
		item->is_edited_inside_one_segment_init)
	{	//change on selection
		item->is_edited_inside_one_segment_init = false;

		smooth_vertices.clear();
		smooth_normals.clear();
		flat_vertices.clear(); 
		flat_normals.clear();
		f_colors.clear();
		v_colors.clear();
		idx_data_.clear();
		idx_edge_data_.clear();
		idx_boundary_edge_data_.clear();
		idx_feature_edge_data_.clear();
		selected_flat_vertices.clear();
		selected_flat_normals.clear();
		selected_f_colors.clear();
		selected_flat_vertices_log.clear();
		selected_flat_normals_log.clear();
		selected_f_colors_log.clear();

		smooth_vertices.shrink_to_fit();
		smooth_normals.shrink_to_fit();
		flat_vertices.shrink_to_fit();
		flat_normals.shrink_to_fit();
		f_colors.shrink_to_fit();
		v_colors.shrink_to_fit();
		idx_data_.shrink_to_fit();
		idx_edge_data_.shrink_to_fit();
		idx_boundary_edge_data_.shrink_to_fit();
		idx_feature_edge_data_.shrink_to_fit();
		selected_flat_vertices.shrink_to_fit();
		selected_flat_normals.shrink_to_fit();
		selected_f_colors.shrink_to_fit();
		selected_flat_vertices_log.shrink_to_fit();
		selected_flat_normals_log.shrink_to_fit();
		selected_f_colors_log.shrink_to_fit();

		SMesh::Property_map<face_descriptor, Kernel::Vector_3 > fnormals = fnormals_log;
		const CGAL::qglviewer::Vec o = static_cast<CGAL::Three::Viewer_interface*>(CGAL::QGLViewer::QGLViewerPool().first())->offset();
		Kernel::Vector_3 offset(o.x, o.y, o.z);

		SMesh::Property_map<vertex_descriptor, SMesh::Point> positions = smesh_->points();

		SMesh::Property_map<face_descriptor, CGAL::Color> fcolors =
			smesh_->property_map<face_descriptor, CGAL::Color >("f:color").first;

		typedef boost::graph_traits<SMesh>::face_descriptor face_descriptor;
		typedef boost::graph_traits<SMesh>::halfedge_descriptor halfedge_descriptor;
		typedef boost::graph_traits<SMesh>::edge_descriptor edge_descriptor;

		if (name.testFlag(Scene_item_rendering_helper::GEOMETRY))
		{
			idx_data_.insert(idx_data_.end(), idx_data_log.begin(), idx_data_log.end());
		}

		if (name.testFlag(Scene_item_rendering_helper::COLORS))
		{
			has_fpatch_id = smesh_->property_map<face_descriptor, int >("f:patch_id").second;
			has_fcolors = smesh_->property_map<face_descriptor, CGAL::Color >("f:color").second;
			has_vcolors = smesh_->property_map<vertex_descriptor, CGAL::Color >("v:color").second;
		}

		if (name.testFlag(Scene_item_rendering_helper::COLORS) && has_fpatch_id) 
			initialize_colors();

		//update selected facets
		if (name.testFlag(Scene_item_rendering_helper::GEOMETRY))
			flat_vertices.insert(flat_vertices.end(), flat_vertices_log.begin(), flat_vertices_log.end());
		if (name.testFlag(Scene_item_rendering_helper::NORMALS))
			flat_normals.insert(flat_normals.end(), flat_normals_log.begin(), flat_normals_log.end());
		if (name.testFlag(Scene_item_rendering_helper::COLORS))
			f_colors.insert(f_colors.end(), f_colors_log.begin(), f_colors_log.end());
		BOOST_FOREACH(face_descriptor f_cur, item->selected_facets_for_annotation)
		{
			int accum_v = f_begin_map[f_cur];
			BOOST_FOREACH(halfedge_descriptor hd, halfedges_around_face(halfedge(f_cur, *smesh_), *smesh_))
			{
				Point p = positions[source(hd, *smesh_)] + offset;
				Kernel::Vector_3 n = fnormals[f_cur];
				CGAL::Color acolor = fcolors[f_cur];

				if (name.testFlag(Scene_item_rendering_helper::GEOMETRY))
				{
					flat_vertices[accum_v] = p.x();
					flat_vertices_log[accum_v] = p.x();
				}
				if (name.testFlag(Scene_item_rendering_helper::NORMALS))
				{
					flat_normals[accum_v] = n.x();
					flat_normals_log[accum_v] = n.x();
				}
				if (name.testFlag(Scene_item_rendering_helper::COLORS))
				{
					f_colors[accum_v] = (float)acolor.red() / (float)255;
					f_colors_log[accum_v] = (float)acolor.red() / (float)255;
				}

				++accum_v;
				if (name.testFlag(Scene_item_rendering_helper::GEOMETRY))
				{
					flat_vertices[accum_v] = p.y();
					flat_vertices_log[accum_v] = p.y();
				}
				if (name.testFlag(Scene_item_rendering_helper::NORMALS))
				{
					flat_normals[accum_v] = n.y();
					flat_normals_log[accum_v] = n.y();
				}
				if (name.testFlag(Scene_item_rendering_helper::COLORS))
				{
					f_colors[accum_v] = (float)acolor.green() / (float)255;
					f_colors_log[accum_v] = (float)acolor.green() / (float)255;
				}

				++accum_v;
				if (name.testFlag(Scene_item_rendering_helper::GEOMETRY))
				{
					flat_vertices[accum_v] = p.z();
					flat_vertices_log[accum_v] = p.z();
				}
				if (name.testFlag(Scene_item_rendering_helper::NORMALS))
				{
					flat_normals[accum_v] = n.z();
					flat_normals_log[accum_v] = n.z();
				}
				if (name.testFlag(Scene_item_rendering_helper::COLORS))
				{
					f_colors[accum_v] = (float)acolor.blue() / (float)255;
					f_colors_log[accum_v] = (float)acolor.blue() / (float)255;
				}

				++accum_v;
			}
		}

		//update selected segment and boundary edges
		idx_edge_data_.insert(idx_edge_data_.end(), idx_edge_data_log.begin(), idx_edge_data_log.end());
		idx_boundary_edge_data_.insert(idx_boundary_edge_data_.end(), idx_boundary_edge_data_log.begin(), idx_boundary_edge_data_log.end());
		idx_feature_edge_data_.insert(idx_feature_edge_data_.end(), idx_feature_edge_data_log.begin(), idx_feature_edge_data_log.end());

		if (!chosen_segments.empty())
		{
			if (chosen_segments.size() == 1)
				choosed_segment_id = (*chosen_segments.begin());
			BOOST_FOREACH(std::size_t seg_id, chosen_segments)
			{
				BOOST_FOREACH(face_descriptor f_cur, item->segments[seg_id].faces_included)
				{
					BOOST_FOREACH(halfedge_descriptor hd, halfedges_around_face(halfedge(f_cur, *smesh_), *smesh_))
					{
						//update verties, normal and color
						Point p = positions[source(hd, *smesh_)] + offset;
						Kernel::Vector_3 n = fnormals[f_cur];
						CGAL::Color acolor = fcolors[f_cur];

						if (name.testFlag(Scene_item_rendering_helper::GEOMETRY))
						{
							add_point_in_buffer(p, selected_flat_vertices);
							add_point_in_buffer(p, selected_flat_vertices_log);
						}
						if (name.testFlag(Scene_item_rendering_helper::NORMALS))
						{
							add_normal_in_buffer(n, selected_flat_normals);
							add_normal_in_buffer(n, selected_flat_normals_log);
						}
						if (name.testFlag(Scene_item_rendering_helper::COLORS))
						{
							add_color_in_buffer(acolor, selected_f_colors);
							add_color_in_buffer(acolor, selected_f_colors_log);
						}

						//updated boundary edges
						if (chosen_segments.size() > 1 && seg_id != choosed_segment_id)
						{
							face_descriptor fd1, fd2;
							fd1 = face(hd, *smesh_);
							fd2 = face(opposite(hd, *smesh_), *smesh_);
							std::size_t seg_id_1 = item->face_segment_id[fd1];
							std::size_t seg_id_2 = item->face_segment_id[fd2];

							if (fd1.is_valid() && fd2.is_valid())
							{
								if (seg_id_1 != seg_id_2 && //make sure is interior ring, not overlap with original boundary
									(seg_id_1 == choosed_segment_id || seg_id_2 == choosed_segment_id))
								{
									idx_boundary_edge_data_.push_back(source(hd, *smesh_));
									idx_boundary_edge_data_.push_back(target(hd, *smesh_));

									idx_boundary_edge_data_log.push_back(source(hd, *smesh_));
									idx_boundary_edge_data_log.push_back(target(hd, *smesh_));
								}
							}
							else if (fd1.is_valid() || fd2.is_valid())
							{
								idx_feature_edge_data_.push_back(source(hd, *smesh_));
								idx_feature_edge_data_.push_back(target(hd, *smesh_));

								idx_feature_edge_data_log.push_back(source(hd, *smesh_));
								idx_feature_edge_data_log.push_back(target(hd, *smesh_));

								if (!this->is_merged_batch_)
								{
									idx_boundary_edge_data_.push_back(source(hd, *smesh_));
									idx_boundary_edge_data_.push_back(target(hd, *smesh_));

									idx_boundary_edge_data_log.push_back(source(hd, *smesh_));
									idx_boundary_edge_data_log.push_back(target(hd, *smesh_));
								}
								else
								{
									if (check_neighbor_faces(source(hd, *smesh_), target(hd, *smesh_)))
									{
										idx_boundary_edge_data_.push_back(source(hd, *smesh_));
										idx_boundary_edge_data_.push_back(target(hd, *smesh_));

										idx_boundary_edge_data_log.push_back(source(hd, *smesh_));
										idx_boundary_edge_data_log.push_back(target(hd, *smesh_));
									}
								}
							}
						}
					}
				}
			}
		}

		if (has_vcolors && name.testFlag(Scene_item_rendering_helper::COLORS))
			v_colors.insert(v_colors.end(), v_colors_log.begin(), v_colors_log.end());

		if (floated &&
			(name.testFlag(Scene_item_rendering_helper::GEOMETRY) || name.testFlag(Scene_item_rendering_helper::NORMALS)))
		{
			if (name.testFlag(Scene_item_rendering_helper::GEOMETRY))
				smooth_vertices.insert(smooth_vertices.end(), smooth_vertices_log.begin(), smooth_vertices_log.end());

			if (name.testFlag(Scene_item_rendering_helper::NORMALS))
				smooth_normals.insert(smooth_normals.end(), smooth_normals_log.begin(), smooth_normals_log.end());
		}

		if (name.testFlag(Scene_item_rendering_helper::GEOMETRY))
		{
			idx_data_size = idx_data_.size();
			flat_vertices_size = flat_vertices.size();
			selected_flat_vertices_size = selected_flat_vertices.size();
			idx_edge_data_size = idx_edge_data_.size();
			idx_feature_edge_data_size = idx_feature_edge_data_.size();

			idx_boundary_edge_data_size = idx_boundary_edge_data_.size();

			item->getPointContainer(0)->allocate(Pt::Vertices, smooth_vertices.data(),
				static_cast<int>(num_vertices(*smesh_) * 3 * sizeof(cgal_gl_data)));

			item->getEdgeContainer(0)->allocate(Ed::Indices, idx_edge_data_.data(),
				static_cast<int>(idx_edge_data_.size() * sizeof(unsigned int)));
			item->getEdgeContainer(0)->allocate(Ed::Vertices, smooth_vertices.data(),
				static_cast<int>(num_vertices(*smesh_) * 3 * sizeof(cgal_gl_data)));

			item->getEdgeContainer(1)->allocate(Ed::Indices, idx_feature_edge_data_.data(),
				static_cast<int>(idx_feature_edge_data_.size() * sizeof(unsigned int)));
			item->getEdgeContainer(1)->allocate(Ed::Vertices, smooth_vertices.data(),
				static_cast<int>(num_vertices(*smesh_) * 3 * sizeof(cgal_gl_data)));

			item->getEdgeContainer(2)->allocate(Ed::Indices, idx_boundary_edge_data_.data(),
				static_cast<int>(idx_boundary_edge_data_.size() * sizeof(unsigned int)));
			item->getEdgeContainer(2)->allocate(Ed::Vertices, smooth_vertices.data(),
				static_cast<int>(num_vertices(*smesh_) * 3 * sizeof(cgal_gl_data)));

			item->getTriangleContainer(0)->allocate(Tri::Vertex_indices, idx_data_.data(),
				static_cast<int>(idx_data_.size() * sizeof(unsigned int)));
			item->getTriangleContainer(0)->allocate(Tri::Smooth_vertices, smooth_vertices.data(),
				static_cast<int>(num_vertices(*smesh_) * 3 * sizeof(cgal_gl_data)));

			item->getTriangleContainer(1)->allocate(Tri::Flat_vertices, flat_vertices.data(),
				static_cast<int>(flat_vertices.size() * sizeof(cgal_gl_data)));

			item->getTriangleContainer(2)->allocate(Tri::Flat_vertices, selected_flat_vertices.data(),
				static_cast<int>(selected_flat_vertices.size() * sizeof(cgal_gl_data)));

		}

		if (name.testFlag(Scene_item_rendering_helper::NORMALS))
		{
			item->getTriangleContainer(1)->allocate(Tri::Flat_normals, flat_normals.data(),
				static_cast<int>(flat_normals.size() * sizeof(cgal_gl_data)));
			item->getTriangleContainer(2)->allocate(Tri::Flat_normals, selected_flat_normals.data(),
				static_cast<int>(selected_flat_normals.size() * sizeof(cgal_gl_data)));
			item->getTriangleContainer(0)->allocate(Tri::Smooth_normals, smooth_normals.data(),
				static_cast<int>(num_vertices(*smesh_) * 3 * sizeof(cgal_gl_data)));
		}

		if (name.testFlag(Scene_item_rendering_helper::COLORS))
		{
			if (!f_colors.empty())
			{
				item->getTriangleContainer(1)->allocate(Tri::FColors, f_colors.data(),
					static_cast<int>(f_colors.size() * sizeof(cgal_gl_data)));
				item->getTriangleContainer(2)->allocate(Tri::FColors, selected_f_colors.data(),
					static_cast<int>(selected_f_colors.size() * sizeof(cgal_gl_data)));
			}
			else {
				item->getTriangleContainer(1)->allocate(Tri::FColors, 0, 0);
				item->getTriangleContainer(2)->allocate(Tri::FColors, 0, 0);
			}
			if (!v_colors.empty())
			{
				item->getTriangleContainer(0)->allocate(Tri::VColors, v_colors.data(),
					static_cast<int>(v_colors.size() * sizeof(cgal_gl_data)));
			}
			else
				item->getTriangleContainer(0)->allocate(Tri::VColors, 0, 0);
		}
	}
	else 
	{
		if (name.testFlag(Scene_item_rendering_helper::GEOMETRY))
		{
			item->getPointContainer(0)->allocate(Pt::Vertices, smooth_vertices_log.data(),
				static_cast<int>(num_vertices(*smesh_) * 3 * sizeof(cgal_gl_data)));

			item->getEdgeContainer(0)->allocate(Ed::Indices, idx_edge_data_log.data(),
				static_cast<int>(idx_edge_data_log.size() * sizeof(unsigned int)));
			item->getEdgeContainer(0)->allocate(Ed::Vertices, smooth_vertices_log.data(),
				static_cast<int>(num_vertices(*smesh_) * 3 * sizeof(cgal_gl_data)));

			item->getEdgeContainer(1)->allocate(Ed::Indices, idx_feature_edge_data_log.data(),
				static_cast<int>(idx_feature_edge_data_log.size() * sizeof(unsigned int)));
			item->getEdgeContainer(1)->allocate(Ed::Vertices, smooth_vertices_log.data(),
				static_cast<int>(num_vertices(*smesh_) * 3 * sizeof(cgal_gl_data)));

			item->getEdgeContainer(2)->allocate(Ed::Indices, idx_boundary_edge_data_log.data(),
				static_cast<int>(idx_boundary_edge_data_log.size() * sizeof(unsigned int)));
			item->getEdgeContainer(2)->allocate(Ed::Vertices, smooth_vertices_log.data(),
				static_cast<int>(num_vertices(*smesh_) * 3 * sizeof(cgal_gl_data)));

			item->getTriangleContainer(0)->allocate(Tri::Vertex_indices, idx_data_log.data(),
				static_cast<int>(idx_data_log.size() * sizeof(unsigned int)));
			item->getTriangleContainer(0)->allocate(Tri::Smooth_vertices, smooth_vertices_log.data(),
				static_cast<int>(num_vertices(*smesh_) * 3 * sizeof(cgal_gl_data)));

			item->getTriangleContainer(1)->allocate(Tri::Flat_vertices, flat_vertices_log.data(),
				static_cast<int>(flat_vertices_log.size() * sizeof(cgal_gl_data)));

			item->getTriangleContainer(2)->allocate(Tri::Flat_vertices, selected_flat_vertices_log.data(),
				static_cast<int>(selected_flat_vertices_log.size() * sizeof(cgal_gl_data)));

		}

		if (name.testFlag(Scene_item_rendering_helper::NORMALS))
		{
			item->getTriangleContainer(1)->allocate(Tri::Flat_normals, flat_normals_log.data(),
				static_cast<int>(flat_normals_log.size() * sizeof(cgal_gl_data)));
			item->getTriangleContainer(2)->allocate(Tri::Flat_normals, selected_flat_normals_log.data(),
				static_cast<int>(selected_flat_normals_log.size() * sizeof(cgal_gl_data)));
			item->getTriangleContainer(0)->allocate(Tri::Smooth_normals, smooth_normals_log.data(),
				static_cast<int>(num_vertices(*smesh_) * 3 * sizeof(cgal_gl_data)));
		}

		if (name.testFlag(Scene_item_rendering_helper::COLORS))
		{
			if (!f_colors_log.empty())
			{
				item->getTriangleContainer(1)->allocate(Tri::FColors, f_colors_log.data(),
					static_cast<int>(f_colors_log.size() * sizeof(cgal_gl_data)));
				item->getTriangleContainer(2)->allocate(Tri::FColors, selected_f_colors_log.data(),
					static_cast<int>(selected_f_colors_log.size() * sizeof(cgal_gl_data)));
			}
			else 
			{
				item->getTriangleContainer(1)->allocate(Tri::FColors, 0, 0);
				item->getTriangleContainer(2)->allocate(Tri::FColors, 0, 0);
			}
			if (!v_colors_log.empty())
			{
				item->getTriangleContainer(0)->allocate(Tri::VColors, v_colors_log.data(),
					static_cast<int>(v_colors_log.size() * sizeof(cgal_gl_data)));
			}
			else
				item->getTriangleContainer(0)->allocate(Tri::VColors, 0, 0);
		}
	}

	QApplication::restoreOverrideCursor();
}

void Scene_surface_mesh_item_priv::compute_elements(Scene_item_rendering_helper::Gl_data_names name)const
{
	if (!item->is_first_construct) // avoid several construction and initialize. All init do once with classification activate.
	{
		if (item->is_in_annotation)
		{
			compute_selected_elements(name);
		}
		else
		{
			QApplication::setOverrideCursor(Qt::WaitCursor);
			smooth_vertices.clear();
			smooth_normals.clear();
			flat_vertices.clear();
			flat_normals.clear();
			f_colors.clear();
			v_colors.clear();
			idx_data_.clear();
			idx_data_.shrink_to_fit();

			SMesh::Property_map<vertex_descriptor, Kernel::Vector_3 > vnormals =
				smesh_->add_property_map<vertex_descriptor, Kernel::Vector_3 >("v:normal").first;

			SMesh::Property_map<face_descriptor, Kernel::Vector_3 > fnormals =
				smesh_->add_property_map<face_descriptor, Kernel::Vector_3 >("f:normal").first;
			CGAL::Polygon_mesh_processing::compute_face_normals(*smesh_, fnormals);
			fnormals_log = fnormals;

			typedef boost::graph_traits<SMesh>::face_descriptor face_descriptor;
			CGAL::Polygon_mesh_processing::compute_vertex_normals(*smesh_, vnormals);

			const CGAL::qglviewer::Vec o = static_cast<CGAL::Three::Viewer_interface*>(CGAL::QGLViewer::QGLViewerPool().first())->offset();
			Kernel::Vector_3 offset(o.x, o.y, o.z);

			SMesh::Property_map<vertex_descriptor, SMesh::Point> positions = smesh_->points();

			SMesh::Property_map<vertex_descriptor, CGAL::Color> vcolors =
				smesh_->property_map<vertex_descriptor, CGAL::Color >("v:color").first;

			SMesh::Property_map<face_descriptor, CGAL::Color> fcolors =
				smesh_->property_map<face_descriptor, CGAL::Color >("f:color").first;

			///Only for test face color
			///item->face_color_replace_for_visualization(fcolors);

			boost::property_map< SMesh, boost::vertex_index_t >::type
				im = get(boost::vertex_index, *smesh_);

			idx_data_.reserve(num_faces(*smesh_) * 3);

			typedef boost::graph_traits<SMesh>::face_descriptor face_descriptor;
			typedef boost::graph_traits<SMesh>::halfedge_descriptor halfedge_descriptor;
			typedef boost::graph_traits<SMesh>::edge_descriptor edge_descriptor;
			auto itn = item->face_center_point_set.normal_map().begin();
			if (name.testFlag(Scene_item_rendering_helper::GEOMETRY))
			{
				BOOST_FOREACH(face_descriptor fd, faces(*smesh_))
				{
					//update mesh normal
					item->face_normals[fd] = fnormals[fd];
					*(itn++) = fnormals[fd];

					if (is_triangle(halfedge(fd, *smesh_), *smesh_))
					{
						BOOST_FOREACH(halfedge_descriptor hd, halfedges_around_face(halfedge(fd, *smesh_), *smesh_))
						{
							idx_data_.push_back(source(hd, *smesh_));
						}
					}
					else
					{
						std::vector<Point> facet_points;
						BOOST_FOREACH(halfedge_descriptor hd, halfedges_around_face(halfedge(fd, *smesh_), *smesh_))
						{
							facet_points.push_back(positions[target(hd, *smesh_)]);
						}
						bool is_convex = is_facet_convex(facet_points, fnormals[fd]);

						if (is_convex && is_quad(halfedge(fd, *smesh_), *smesh_))
						{
							halfedge_descriptor hd = halfedge(fd, *smesh_);
							//1st half
							idx_data_.push_back(source(hd, *smesh_));
							idx_data_.push_back(source(next(hd, *smesh_), *smesh_));
							idx_data_.push_back(source(next(next(hd, *smesh_), *smesh_), *smesh_));

							//2nd half
							idx_data_.push_back(source(hd, *smesh_));
							idx_data_.push_back(source(next(next(hd, *smesh_), *smesh_), *smesh_));
							idx_data_.push_back(source(prev(hd, *smesh_), *smesh_));
						}
						else if (is_convex)
						{
							triangulate_convex_facet(fd, &fnormals, 0, &im, name, true);
						}
						else
						{
							triangulate_facet(fd, &fnormals, 0, &im, name, true);
						}
					}
				}
			}

			if (name.testFlag(Scene_item_rendering_helper::COLORS))
			{

				has_fpatch_id = smesh_->property_map<face_descriptor, int >("f:patch_id").second;
				has_fcolors = smesh_->property_map<face_descriptor, CGAL::Color >("f:color").second;
				has_vcolors = smesh_->property_map<vertex_descriptor, CGAL::Color >("v:color").second;
			}

			if (name.testFlag(Scene_item_rendering_helper::GEOMETRY))
			{
				idx_edge_data_.clear();
				idx_edge_data_.shrink_to_fit();
				idx_edge_data_.reserve(num_edges(*smesh_) * 2);

				idx_boundary_edge_data_.clear();
				idx_boundary_edge_data_.shrink_to_fit();
				idx_boundary_edge_data_.reserve(num_edges(*smesh_) * 2);

				BOOST_FOREACH(edge_descriptor ed, edges(*smesh_))
				{
					if (!item->face_segment_id.empty())
					{
						face_descriptor fd1, fd2;
						fd1 = face(halfedge(ed, *smesh_), *smesh_);
						fd2 = face(opposite(halfedge(ed, *smesh_), *smesh_), *smesh_);

						if (fd1.is_valid() && fd2.is_valid())
						{
							if (item->face_segment_id[fd1] != item->face_segment_id[fd2])
							{
								idx_boundary_edge_data_.push_back(source(ed, *smesh_));
								idx_boundary_edge_data_.push_back(target(ed, *smesh_));
							}
						}
						else if (fd1.is_valid() || fd2.is_valid())
						{
							//idx_boundary_edge_data_.push_back(source(ed, *smesh_));
							//idx_boundary_edge_data_.push_back(target(ed, *smesh_));
							idx_feature_edge_data_.push_back(source(ed, *smesh_));
							idx_feature_edge_data_.push_back(target(ed, *smesh_));

							if (!this->is_merged_batch_)
							{
								idx_boundary_edge_data_.push_back(source(ed, *smesh_));
								idx_boundary_edge_data_.push_back(target(ed, *smesh_));
							}
							else
							{
								if (check_neighbor_faces(source(ed, *smesh_), target(ed, *smesh_)))
								{
									idx_boundary_edge_data_.push_back(source(ed, *smesh_));
									idx_boundary_edge_data_.push_back(target(ed, *smesh_));
								}
							}
						}
					}
					//if (has_feature_edges && get(e_is_feature_map, ed))
					//{
					//	idx_feature_edge_data_.push_back(source(ed, *smesh_));
					//	idx_feature_edge_data_.push_back(target(ed, *smesh_));
					//}
					idx_edge_data_.push_back(source(ed, *smesh_));
					idx_edge_data_.push_back(target(ed, *smesh_));
				}
				idx_edge_data_.shrink_to_fit();
				idx_boundary_edge_data_.shrink_to_fit();
				idx_feature_edge_data_.shrink_to_fit();
			}

			if (name.testFlag(Scene_item_rendering_helper::COLORS) &&
				has_fpatch_id) {
				initialize_colors();
			}

			//compute the Flat data
			flat_vertices.clear();
			flat_normals.clear();
			f_colors.clear();
			selected_flat_vertices.clear();
			selected_flat_normals.clear();
			selected_f_colors.clear();


			BOOST_FOREACH(face_descriptor fd, faces(*smesh_))
			{
				if (is_triangle(halfedge(fd, *smesh_), *smesh_) && fd.is_valid())
				{
					f_begin_map[fd] = f_colors.size();
					BOOST_FOREACH(halfedge_descriptor hd, halfedges_around_face(halfedge(fd, *smesh_), *smesh_))
					{
						if (name.testFlag(Scene_item_rendering_helper::GEOMETRY))
						{
							Point p = positions[source(hd, *smesh_)] + offset;
							if (!chosen_segments.empty() && chosen_segments.find(item->face_segment_id[fd]) != chosen_segments.end()) {
								add_point_in_buffer(p, selected_flat_vertices);
							}
							add_point_in_buffer(p, flat_vertices);
						}
						if (name.testFlag(Scene_item_rendering_helper::NORMALS))
						{
							Kernel::Vector_3 n = fnormals[fd];
							if (!chosen_segments.empty() && chosen_segments.find(item->face_segment_id[fd]) != chosen_segments.end()) {
								add_normal_in_buffer(n, selected_flat_normals);
							}
							add_normal_in_buffer(n, flat_normals);
						}
						if (name.testFlag(Scene_item_rendering_helper::COLORS))
						{
							if (has_fpatch_id)
							{
								//The sharp features detection produces patch ids >=1, this
								//is meant to insure the wanted id is in the range [min,max]
								QColor c = item->color_vector()[fpatch_id_map[fd] - min_patch_id];
								CGAL::Color color(c.red(), c.green(), c.blue());

								if (!chosen_segments.empty() && chosen_segments.find(item->face_segment_id[fd]) != chosen_segments.end()) {
									add_color_in_buffer(color, selected_f_colors);
								}
								add_color_in_buffer(color, f_colors);

							}
							else if (has_fcolors)
							{
								CGAL::Color c = fcolors[fd];
								if (!chosen_segments.empty() && chosen_segments.find(item->face_segment_id[fd]) != chosen_segments.end())
								{
									add_color_in_buffer(c, selected_f_colors);
								}
								add_color_in_buffer(c, f_colors);
							}
						}
					}
				}
				else
				{
					std::vector<Point> facet_points;
					BOOST_FOREACH(halfedge_descriptor hd, halfedges_around_face(halfedge(fd, *smesh_), *smesh_))
					{
						facet_points.push_back(positions[target(hd, *smesh_)]);
					}
					bool is_convex = is_facet_convex(facet_points, fnormals[fd]);
					if (is_convex && is_quad(halfedge(fd, *smesh_), *smesh_))
					{
						//1st half
						halfedge_descriptor hd = halfedge(fd, *smesh_);
						Point p = positions[source(hd, *smesh_)];
						Kernel::Vector_3 n = fnormals[fd];
						CGAL::Color* c;
						if (has_fpatch_id)
						{
							QColor color = item->color_vector()[fpatch_id_map[fd] - min_patch_id];
							c = new CGAL::Color(color.red(), color.green(), color.blue());
						}
						else if (has_fcolors)
							c = &fcolors[fd];
						else
							c = 0;

						addFlatData(p, n, c, name);
						if (!chosen_segments.empty() && chosen_segments.find(item->face_segment_id[fd]) != chosen_segments.end())
							addSelectedFlatData(p, n, c, name);


						hd = next(halfedge(fd, *smesh_), *smesh_);
						if (!chosen_segments.empty() && chosen_segments.find(item->face_segment_id[fd]) != chosen_segments.end())
							addSelectedFlatData(positions[source(hd, *smesh_)], fnormals[fd], c, name);
						addFlatData(positions[source(hd, *smesh_)], fnormals[fd], c, name);


						hd = next(next(halfedge(fd, *smesh_), *smesh_), *smesh_);
						if (!chosen_segments.empty() && chosen_segments.find(item->face_segment_id[fd]) != chosen_segments.end())
							addSelectedFlatData(positions[source(hd, *smesh_)]
								, fnormals[fd]
								, c
								, name);
						addFlatData(positions[source(hd, *smesh_)]
							, fnormals[fd]
							, c
							, name);


						//2nd half
						hd = halfedge(fd, *smesh_);
						if (!chosen_segments.empty() && chosen_segments.find(item->face_segment_id[fd]) != chosen_segments.end())
							addSelectedFlatData(positions[source(hd, *smesh_)]
								, fnormals[fd]
								, c
								, name);
						addFlatData(positions[source(hd, *smesh_)]
							, fnormals[fd]
							, c
							, name);


						hd = next(next(halfedge(fd, *smesh_), *smesh_), *smesh_);
						if (!chosen_segments.empty() && chosen_segments.find(item->face_segment_id[fd]) != chosen_segments.end())
							addSelectedFlatData(positions[source(hd, *smesh_)]
								, fnormals[fd]
								, c
								, name);
						addFlatData(positions[source(hd, *smesh_)]
							, fnormals[fd]
							, c
							, name);


						hd = prev(halfedge(fd, *smesh_), *smesh_);
						if (!chosen_segments.empty() && chosen_segments.find(item->face_segment_id[fd]) != chosen_segments.end())
							addSelectedFlatData(positions[source(hd, *smesh_)]
								, fnormals[fd]
								, c
								, name);
						addFlatData(positions[source(hd, *smesh_)]
							, fnormals[fd]
							, c
							, name);


						if (has_fpatch_id)
							delete c;
					}
					else if (is_convex)
					{
						triangulate_convex_facet(fd, &fnormals, &fcolors, 0, name, false);
					}
					else
					{
						triangulate_facet(fd, &fnormals, &fcolors, 0, name, false);
					}
				}
			}

			if (has_vcolors && name.testFlag(Scene_item_rendering_helper::COLORS))
			{
				BOOST_FOREACH(vertex_descriptor vd, vertices(*smesh_))
				{
					CGAL::Color c = vcolors[vd];
					v_colors.push_back((float)c.red() / 255);
					v_colors.push_back((float)c.green() / 255);
					v_colors.push_back((float)c.blue() / 255);
				}
			}

			if (floated &&
				(name.testFlag(Scene_item_rendering_helper::GEOMETRY) || name.testFlag(Scene_item_rendering_helper::NORMALS)))
			{
				BOOST_FOREACH(vertex_descriptor vd, vertices(*smesh_))
				{
					if (name.testFlag(Scene_item_rendering_helper::GEOMETRY))
					{
						Point p = positions[vd] + offset;
						add_point_in_buffer(p, smooth_vertices);
					}
					if (name.testFlag(Scene_item_rendering_helper::NORMALS))
					{
						Kernel::Vector_3 n = vnormals[vd];
						add_normal_in_buffer(n, smooth_normals);
					}
				}
			}


			if (name.testFlag(Scene_item_rendering_helper::GEOMETRY))
			{

				idx_data_size = idx_data_.size();
				flat_vertices_size = flat_vertices.size();
				selected_flat_vertices_size = selected_flat_vertices.size();
				idx_edge_data_size = idx_edge_data_.size();
				idx_feature_edge_data_size = idx_feature_edge_data_.size();
				idx_boundary_edge_data_size = idx_boundary_edge_data_.size();

				item->getPointContainer(0)->allocate(Pt::Vertices, smooth_vertices.data(),
					static_cast<int>(num_vertices(*smesh_) * 3 * sizeof(cgal_gl_data)));

				item->getEdgeContainer(0)->allocate(Ed::Indices, idx_edge_data_.data(),
					static_cast<int>(idx_edge_data_.size() * sizeof(unsigned int)));
				item->getEdgeContainer(0)->allocate(Ed::Vertices, smooth_vertices.data(),
					static_cast<int>(num_vertices(*smesh_) * 3 * sizeof(cgal_gl_data)));

				item->getEdgeContainer(1)->allocate(Ed::Indices, idx_feature_edge_data_.data(),
					static_cast<int>(idx_feature_edge_data_.size() * sizeof(unsigned int)));
				item->getEdgeContainer(1)->allocate(Ed::Vertices, smooth_vertices.data(),
					static_cast<int>(num_vertices(*smesh_) * 3 * sizeof(cgal_gl_data)));

				item->getEdgeContainer(2)->allocate(Ed::Indices, idx_boundary_edge_data_.data(),
					static_cast<int>(idx_boundary_edge_data_.size() * sizeof(unsigned int)));
				item->getEdgeContainer(2)->allocate(Ed::Vertices, smooth_vertices.data(),
					static_cast<int>(num_vertices(*smesh_) * 3 * sizeof(cgal_gl_data)));

				item->getTriangleContainer(0)->allocate(Tri::Vertex_indices, idx_data_.data(),
					static_cast<int>(idx_data_.size() * sizeof(unsigned int)));
				item->getTriangleContainer(0)->allocate(Tri::Smooth_vertices, smooth_vertices.data(),
					static_cast<int>(num_vertices(*smesh_) * 3 * sizeof(cgal_gl_data)));

				item->getTriangleContainer(1)->allocate(Tri::Flat_vertices, flat_vertices.data(),
					static_cast<int>(flat_vertices.size() * sizeof(cgal_gl_data)));

				item->getTriangleContainer(2)->allocate(Tri::Flat_vertices, selected_flat_vertices.data(),
					static_cast<int>(selected_flat_vertices.size() * sizeof(cgal_gl_data)));

			}

			if (name.testFlag(Scene_item_rendering_helper::NORMALS))
			{

				item->getTriangleContainer(1)->allocate(Tri::Flat_normals, flat_normals.data(),
					static_cast<int>(flat_normals.size() * sizeof(cgal_gl_data)));
				item->getTriangleContainer(2)->allocate(Tri::Flat_normals, selected_flat_normals.data(),
					static_cast<int>(selected_flat_normals.size() * sizeof(cgal_gl_data)));
				item->getTriangleContainer(0)->allocate(Tri::Smooth_normals, smooth_normals.data(),
					static_cast<int>(num_vertices(*smesh_) * 3 * sizeof(cgal_gl_data)));
			}

			if (name.testFlag(Scene_item_rendering_helper::COLORS))
			{
				if (!f_colors.empty())
				{
					item->getTriangleContainer(1)->allocate(Tri::FColors, f_colors.data(),
						static_cast<int>(f_colors.size() * sizeof(cgal_gl_data)));
					item->getTriangleContainer(2)->allocate(Tri::FColors, selected_f_colors.data(),
						static_cast<int>(selected_f_colors.size() * sizeof(cgal_gl_data)));
				}
				else {
					item->getTriangleContainer(1)->allocate(Tri::FColors, 0, 0);
					item->getTriangleContainer(2)->allocate(Tri::FColors, 0, 0);
				}

				if (!v_colors.empty())
				{
					item->getTriangleContainer(0)->allocate(Tri::VColors, v_colors.data(),
						static_cast<int>(v_colors.size() * sizeof(cgal_gl_data)));
				}
				else
					item->getTriangleContainer(0)->allocate(Tri::VColors, 0, 0);
			}

			//data backup
			if (!smooth_vertices.empty())
			{
				smooth_vertices_log.clear();
				smooth_vertices_log.assign(smooth_vertices.begin(), smooth_vertices.end());
			}
			if (!smooth_normals.empty())
			{
				smooth_normals_log.clear();
				smooth_normals_log.assign(smooth_normals.begin(), smooth_normals.end());
			}

			if (!idx_data_.empty())
			{
				idx_data_log.clear();
				idx_data_log.assign(idx_data_.begin(), idx_data_.end());
			}
			if (!idx_edge_data_.empty())
			{
				idx_edge_data_log.clear();
				idx_edge_data_log.assign(idx_edge_data_.begin(), idx_edge_data_.end());
			}
			if (!idx_feature_edge_data_.empty())
			{
				idx_feature_edge_data_log.clear();
				idx_feature_edge_data_log.assign(idx_feature_edge_data_.begin(), idx_feature_edge_data_.end());
			}

			if (!flat_vertices.empty())
			{
				flat_vertices_log.clear();
				flat_vertices_log.assign(flat_vertices.begin(), flat_vertices.end());
			}
			if (!flat_normals.empty())
			{
				flat_normals_log.clear();
				flat_normals_log.assign(flat_normals.begin(), flat_normals.end());
			}

			if (!selected_f_colors.empty())
			{
				selected_f_colors_log.clear();
				selected_f_colors_log.assign(selected_f_colors.begin(), selected_f_colors.end());
			}
			if (!selected_flat_vertices.empty())
			{
				selected_flat_vertices_log.clear();
				selected_flat_vertices_log.assign(selected_flat_vertices.begin(), selected_flat_vertices.end());
			}
			if (!selected_flat_normals.empty())
			{
				selected_flat_normals_log.clear();
				selected_flat_normals_log.assign(selected_flat_normals.begin(), selected_flat_normals.end());
			}

			if (!v_colors.empty())
			{
				v_colors_log.clear();
				v_colors_log.assign(v_colors.begin(), v_colors.end());
			}
			if (!f_colors.empty())
			{
				f_colors_log.clear();
				f_colors_log.assign(f_colors.begin(), f_colors.end());
			}
			QApplication::restoreOverrideCursor();
		}
	}
}

void Scene_surface_mesh_item_priv::computeSegmentBoundary() 
{
	//only used in the init process
	typedef boost::graph_traits<SMesh>::halfedge_descriptor halfedge_descriptor;
	typedef boost::graph_traits<SMesh>::edge_descriptor edge_descriptor;

	idx_boundary_edge_data_.clear();
	idx_boundary_edge_data_.shrink_to_fit();
	idx_boundary_edge_data_.reserve(num_edges(*smesh_) * 2);

	idx_feature_edge_data_.clear();
	idx_feature_edge_data_.shrink_to_fit();
	idx_feature_edge_data_.reserve(num_edges(*smesh_) * 2);

	//idx_edge_data_.clear();
	//idx_edge_data_.shrink_to_fit();
	//idx_edge_data_.reserve(num_edges(*smesh_) * 2);

	BOOST_FOREACH(edge_descriptor ed, edges(*smesh_))
	{
		face_descriptor fd1, fd2;
		fd1 = face(halfedge(ed, *smesh_), *smesh_);
		fd2 = face(opposite(halfedge(ed, *smesh_), *smesh_), *smesh_);

		if (fd1.is_valid() && fd2.is_valid())
		{
			if (item->face_segment_id[fd1] != item->face_segment_id[fd2])
			{
				idx_boundary_edge_data_.push_back(source(ed, *smesh_));
				idx_boundary_edge_data_.push_back(target(ed, *smesh_));
				//seg_id s1 = item->face_segment_id[fd1];
				//seg_id s2 = item->face_segment_id[fd2];

				//item->boundary_info[ed].set_adjecent_segs(&(item->segments[s1]),
				//	&(item->segments[s2]));
			}
		}
		else if (fd1.is_valid() || fd2.is_valid())
		{
			//idx_boundary_edge_data_.push_back(source(ed, *smesh_));
			//idx_boundary_edge_data_.push_back(target(ed, *smesh_));
			idx_feature_edge_data_.push_back(source(ed, *smesh_));
			idx_feature_edge_data_.push_back(target(ed, *smesh_));

			if (!this->is_merged_batch_)
			{
				idx_boundary_edge_data_.push_back(source(ed, *smesh_));
				idx_boundary_edge_data_.push_back(target(ed, *smesh_));
			}
			else
			{
				if (check_neighbor_faces(source(ed, *smesh_), target(ed, *smesh_)))
				{
					idx_boundary_edge_data_.push_back(source(ed, *smesh_));
					idx_boundary_edge_data_.push_back(target(ed, *smesh_));
				}
			}
		}

		//idx_edge_data_.push_back(source(ed, *smesh_));
		//idx_edge_data_.push_back(target(ed, *smesh_));
	}

	//idx_edge_data_.shrink_to_fit();
	//idx_edge_data_size = idx_edge_data_.size();

	//item->getEdgeContainer(0)->setIdxSize(idx_edge_data_size);
	//item->getEdgeContainer(0)->allocate(Ed::Indices, idx_edge_data_.data(),
	//	static_cast<int>(idx_edge_data_.size() * sizeof(unsigned int)));
	//item->getEdgeContainer(0)->allocate(Ed::Vertices, smooth_vertices.data(),
	//	static_cast<int>(num_vertices(*smesh_) * 3 * sizeof(cgal_gl_data)));

	idx_boundary_edge_data_.shrink_to_fit();
	idx_boundary_edge_data_size = idx_boundary_edge_data_.size();

	item->getEdgeContainer(2)->setIdxSize(idx_boundary_edge_data_size);
	item->getEdgeContainer(2)->allocate(Ed::Indices, idx_boundary_edge_data_.data(),
		static_cast<int>(idx_boundary_edge_data_.size() * sizeof(unsigned int)));
	item->getEdgeContainer(2)->allocate(Ed::Vertices, smooth_vertices.data(),
		static_cast<int>(num_vertices(*smesh_) * 3 * sizeof(cgal_gl_data)));

	if (!idx_boundary_edge_data_.empty())
	{
		idx_boundary_edge_data_log.clear();
		idx_boundary_edge_data_log.assign(idx_boundary_edge_data_.begin(), idx_boundary_edge_data_.end());
	}


	idx_feature_edge_data_.shrink_to_fit();
	idx_feature_edge_data_size = idx_feature_edge_data_.size();

	//item->getEdgeContainer(1)->setIdxSize(idx_feature_edge_data_size);
	item->getEdgeContainer(1)->allocate(Ed::Indices, idx_feature_edge_data_.data(),
		static_cast<int>(idx_feature_edge_data_.size() * sizeof(unsigned int)));
	item->getEdgeContainer(1)->allocate(Ed::Vertices, smooth_vertices.data(),
		static_cast<int>(num_vertices(*smesh_) * 3 * sizeof(cgal_gl_data)));

	if (!idx_feature_edge_data_.empty())
	{
		idx_feature_edge_data_log.clear();
		idx_feature_edge_data_log.assign(idx_feature_edge_data_.begin(), idx_feature_edge_data_.end());
	}
}

void Scene_surface_mesh_item::computeSegmentBoundary()
{
	computeSegments();
	d->computeSegmentBoundary();
}

void Scene_surface_mesh_item::computeSegments()
{
	for (std::map<face_descriptor, int>::iterator p = face_segment_id.begin(); p != face_segment_id.end(); p++)
	{
		if (p->first.is_valid())
		{
			segments[p->second].faces_included.insert(p->first);
			segments[p->second].segment_area += CGAL::Polygon_mesh_processing::face_area(p->first, *(this->polyhedron()));
		}
	}

	std::pair<int, int> minmax_faces_segment = std::make_pair<int, int>(2147483647, 0);
	for (std::map<seg_id, Segment>::iterator p = segments.begin(); p != segments.end(); p++)
	{
		p->second.id = p->first;
		if (minmax_faces_segment.first > p->second.faces_included.size())
		{
			minmax_faces_segment.first = p->second.faces_included.size();
			minmax_faces_segment_id.first = p->second.id;
		}

		if (minmax_faces_segment.second < p->second.faces_included.size())
		{
			minmax_faces_segment.second = p->second.faces_included.size();
			minmax_faces_segment_id.second = p->second.id;
		}
	}
}

void Scene_surface_mesh_item::get_connected_faces(face_descriptor fd, std::vector<face_descriptor>& connected_faces)
{
	connected_faces.clear();
	if (!d->is_merged_batch_)
	{
		halfedge_descriptor head = halfedge(fd, *d->smesh_);
		for (halfedge_descriptor hd = head; ; )
		{
			halfedge_descriptor op = opposite(hd, *d->smesh_);
			connected_faces.push_back(face(op, *d->smesh_));
			hd = next(hd, *d->smesh_);
			if (hd == head)
				break;
		}
	}
	else
	{
		int v0 = d->face_non_duplicated_vertices_map_[fd][0],
			v1 = d->face_non_duplicated_vertices_map_[fd][1],
			v2 = d->face_non_duplicated_vertices_map_[fd][2];

		std::vector<face_descriptor> neighbor_faces;
		neighbor_faces.insert(neighbor_faces.end(), d->st_faces_map_[std::make_pair(v0, v1)].begin(), d->st_faces_map_[std::make_pair(v0, v1)].end());
		neighbor_faces.insert(neighbor_faces.end(), d->st_faces_map_[std::make_pair(v1, v0)].begin(), d->st_faces_map_[std::make_pair(v1, v0)].end());
		neighbor_faces.insert(neighbor_faces.end(), d->st_faces_map_[std::make_pair(v0, v2)].begin(), d->st_faces_map_[std::make_pair(v0, v2)].end());
		neighbor_faces.insert(neighbor_faces.end(), d->st_faces_map_[std::make_pair(v2, v0)].begin(), d->st_faces_map_[std::make_pair(v2, v0)].end());
		neighbor_faces.insert(neighbor_faces.end(), d->st_faces_map_[std::make_pair(v1, v2)].begin(), d->st_faces_map_[std::make_pair(v1, v2)].end());
		neighbor_faces.insert(neighbor_faces.end(), d->st_faces_map_[std::make_pair(v2, v1)].begin(), d->st_faces_map_[std::make_pair(v2, v1)].end());

		sort(neighbor_faces.begin(), neighbor_faces.end());
		neighbor_faces.erase(unique(neighbor_faces.begin(), neighbor_faces.end()),
			neighbor_faces.end());

		for (auto fd_i : neighbor_faces)
		{
			if (fd_i != fd)
				connected_faces.push_back(fd_i);
		}
	}
}

void Scene_surface_mesh_item::emphasize_present_segment(seg_id seg) {
	m_RMode = renderingMode();
	d->chosen_segments.clear();
	d->chosen_segments.insert(seg);
	//d->compute_elements(ALL);
	setRenderingMode(Emphasizing);
	d->item->is_in_annotation = true;
	d->item->is_edited_inside_one_segment_init = true;

	//tmp_default_renderingmode = CGAL::Three::Three::defaultSurfaceMeshRenderingMode();
	CGAL::Three::Three::SetdefaultSurfaceMeshRenderingMode(Emphasizing);

	Q_EMIT itemChanged();
	invalidateOpenGLBuffers();
	Q_EMIT redraw();
}

void Scene_surface_mesh_item::unemphasize()
{
	//setRenderingMode(m_RMode);
	//if (tmp_default_renderingmode >= 0 && tmp_default_renderingmode < NumberOfRenderingMode)
	//	CGAL::Three::Three::SetdefaultSurfaceMeshRenderingMode(tmp_default_renderingmode);
	//setRenderingMode(CGAL::Three::Three::defaultSurfaceMeshRenderingMode());
	setRenderingMode(default_renderingmode);
	d->chosen_segments.clear();
	//d->compute_elements(ALL);
	d->item->is_in_annotation = true;

	Q_EMIT itemChanged();
	//invalidateOpenGLBuffers();
	Q_EMIT redraw();
}

void Scene_surface_mesh_item::addChosenSegment(seg_id id)
{
	d->chosen_segments.insert(id);
}

void Scene_surface_mesh_item::eraseChosenSegment(seg_id id)
{
	d->chosen_segments.erase(id);
}

int Scene_surface_mesh_item::updateSegmentId(std::map<face_descriptor, bool> &face_visited_check)
{
	std::vector<std::vector<face_descriptor>> all_regions;
	std::map<face_descriptor, int> face_segment_id_clone = face_segment_id;
	std::map<face_descriptor, bool> face_visited_check_clone = face_visited_check;
	BOOST_FOREACH(face_descriptor fd, faces(*(polyhedron())))
	{
		if (face_visited_check_clone[fd] || !fd.is_valid())
			continue;

		std::vector<face_descriptor> current_region, current_seeds;
		face_visited_check_clone[fd] = true;
		current_seeds.emplace_back(fd);
		current_region.emplace_back(fd);
		int seed_id = face_segment_id_clone[fd];
		int new_id = 0;
		for (int i = 0; i < current_seeds.size(); ++i)
		{
			face_descriptor f_cu = current_seeds[i];

			// loop over all incident faces around the face
			halfedge_descriptor head = halfedge(f_cu, *d->smesh_);
			for (halfedge_descriptor hd = head; ; )
			{
				halfedge_descriptor op = opposite(hd, *d->smesh_);
				face_descriptor f_neg = face(op, *d->smesh_);

				if (f_neg.is_valid())
					face_neighbors[f_cu].push_back(f_neg);

				if (seed_id == face_segment_id_clone[f_neg])
				{
					if (!face_visited_check_clone[f_neg]
						&& f_neg.is_valid())
					{
						face_visited_check_clone[f_neg] = true;
						current_region.emplace_back(f_neg);
						current_seeds.emplace_back(f_neg);
					}
				}

				hd = next(hd, *d->smesh_);
				if (hd == head)
					break;
			}
		}

		if (!current_region.empty())
			all_regions.emplace_back(current_region);
	}

	for (int ri = 0; ri < all_regions.size(); ++ri)
	{
		float prob_accum = 0;
		for (int fi = 0; fi < all_regions[ri].size(); ++fi)
		{
			face_segment_id[all_regions[ri][fi]] = ri;
			prob_accum += label_probabilities[all_regions[ri][fi]];
		}
		//compute error probability
		prob_accum /= float(all_regions[ri].size());
		total_error_facets += (1.0f - prob_accum) *  float(all_regions[ri].size());
	}
	return all_regions.size();
}

void Scene_surface_mesh_item::findDuplicateVertices
(
	Scene_surface_mesh_item* sm_item,
	std::vector<Kernel::Point_3> &points,
	std::vector<std::vector<std::size_t> > &faces_vind
)
{
	int v_ind = 0;
	std::map<Kernel::Point_3, int> duplicate_map;
	BOOST_FOREACH(Kernel::Point_3 pd, points)
	{
		auto it = duplicate_map.find(pd);
		if (it == duplicate_map.end())
		{
			duplicate_map[pd] = v_ind;
		}
		else
		{
			old_new_duplicate_verts_map[v_ind] = duplicate_map[pd];
		}
		++v_ind;
	}

	int f_ind = 0;
	BOOST_FOREACH(face_descriptor fd, faces(*(polyhedron())))
	{
		for(auto vi : faces_vind[f_ind])
		{
			auto it = old_new_duplicate_verts_map.find(vi);
			if (it != old_new_duplicate_verts_map.end())
			{
				sm_item->face_non_duplicated_vertices_map[fd].push_back((*it).second);
			}
			else
			{
				sm_item->face_non_duplicated_vertices_map[fd].push_back(vi);
			}
		}

		int v0 = sm_item->face_non_duplicated_vertices_map[fd][0],
			v1 = sm_item->face_non_duplicated_vertices_map[fd][1],
			v2 = sm_item->face_non_duplicated_vertices_map[fd][2];

		auto it_01 = sm_item->st_faces_map.find(std::make_pair(v0, v1));
		auto it_10 = sm_item->st_faces_map.find(std::make_pair(v1, v0));
		auto it_02 = sm_item->st_faces_map.find(std::make_pair(v0, v2));
		auto it_20 = sm_item->st_faces_map.find(std::make_pair(v2, v0));
		auto it_12 = sm_item->st_faces_map.find(std::make_pair(v1, v2));
		auto it_21 = sm_item->st_faces_map.find(std::make_pair(v2, v1));

		if (it_01 == sm_item->st_faces_map.end())
			sm_item->st_faces_map[std::make_pair(v0, v1)] = std::vector<face_descriptor>();
		if (it_10 == sm_item->st_faces_map.end())
			sm_item->st_faces_map[std::make_pair(v1, v0)] = std::vector<face_descriptor>();
		if (it_02 == sm_item->st_faces_map.end())
			sm_item->st_faces_map[std::make_pair(v0, v2)] = std::vector<face_descriptor>();
		if (it_20 == sm_item->st_faces_map.end())
			sm_item->st_faces_map[std::make_pair(v2, v0)] = std::vector<face_descriptor>();
		if (it_12 == sm_item->st_faces_map.end())
			sm_item->st_faces_map[std::make_pair(v1, v2)] = std::vector<face_descriptor>();
		if (it_21 == sm_item->st_faces_map.end())
			sm_item->st_faces_map[std::make_pair(v2, v1)] = std::vector<face_descriptor>();

		sm_item->st_faces_map[std::make_pair(v0, v1)].push_back(fd);
		sm_item->st_faces_map[std::make_pair(v1, v0)].push_back(fd);
		sm_item->st_faces_map[std::make_pair(v0, v2)].push_back(fd);
		sm_item->st_faces_map[std::make_pair(v2, v0)].push_back(fd);
		sm_item->st_faces_map[std::make_pair(v1, v2)].push_back(fd);
		sm_item->st_faces_map[std::make_pair(v2, v1)].push_back(fd);

		++f_ind;
	}

	sm_item->update_priv();
}

void Scene_surface_mesh_item::grouping_facets_for_multi_texture_rendering
(
	Scene_surface_mesh_item* sm_item, 
	std::vector<Kernel::Point_3> &points,
	std::map<int, std::set<int>> &textureID_vertices_grouping,
	std::map<int, std::vector<face_vind_texcoord>> &textureID_facets_grouping
)
{
	d->texture_images_ = this->texture_images;
	d->texture_items = new Scene_textured_surface_mesh_item*[this->texture_images.size()];

	int tex_i = 0;
	auto it_tex = textureID_facets_grouping.begin();
	auto it_tex_end = textureID_facets_grouping.end();
	for (; it_tex != it_tex_end; ++it_tex)
	{
		//define pointer
		d->texture_items[tex_i] = new Scene_textured_surface_mesh_item;
		//parsing texture image
		d->texture_items[tex_i]->texture_image_in = sm_item->texture_images[tex_i];

		//add new vertices
		std::map<int, vertex_descriptor> old_new_vert_map;
		auto it_vert = textureID_vertices_grouping[(*it_tex).first].begin();
		auto it_vert_end = textureID_vertices_grouping[(*it_tex).first].end();
		for (; it_vert != it_vert_end; ++it_vert)
		{
			d->texture_items[tex_i]->mesh_in->add_vertex(points[(*it_vert)]);
			old_new_vert_map[(*it_vert)] = *(--d->texture_items[tex_i]->mesh_in->vertices_end());
		}

		//add new faces and texture coordinates
		for (auto fd_tuple : (*it_tex).second)
		{
			auto new_vert_0 = old_new_vert_map[get<1>(fd_tuple)[0]];
			auto new_vert_1 = old_new_vert_map[get<1>(fd_tuple)[1]];
			auto new_vert_2 = old_new_vert_map[get<1>(fd_tuple)[2]];
			d->texture_items[tex_i]->mesh_in->add_face(new_vert_0, new_vert_1, new_vert_2);
			d->texture_items[tex_i]->face_texcoord[*(--d->texture_items[tex_i]->mesh_in->faces_end())] = get<2>(fd_tuple);
		}

		++tex_i;
	}
}

void Scene_surface_mesh_item::reset_into_one_segment(SMesh* smesh)
{
	BOOST_FOREACH(face_descriptor fd, faces(*smesh))
	{
		face_segment_id[fd] = 0;
	}
}

void Scene_surface_mesh_item::region_growing_on_mesh
(
	SMesh* selected_segment_mesh,
	double& max_distance_to_plane_,
	double& max_accepted_angle_,
	int& min_region_size_,
	std::map<int, face_descriptor> &segment_fid_face_map,
	std::vector<int> &selected_main_faces
)
{
	const Face_range_cgal face_range = faces(*selected_segment_mesh);

	// Default parameter values for the data file polygon_mesh.off.
	const Kernel::FT          max_distance_to_plane = Kernel::FT(max_distance_to_plane_);
	const Kernel::FT          max_accepted_angle = Kernel::FT(max_accepted_angle_);
	const std::size_t min_region_size = min_region_size_;

	// Create instances of the classes Neighbor_query and Region_type.
	Neighbor_query_mesh_cgal neighbor_query(*selected_segment_mesh);
	const Vertex_to_point_map_mesh_cgal vertex_to_point_map(get(CGAL::vertex_point, *selected_segment_mesh));

	region_type_mesh_cgal region_type(
		*selected_segment_mesh,
		max_distance_to_plane, max_accepted_angle, min_region_size,
		vertex_to_point_map);

	// Sort face indices.
	sorting_mesh_cgal sorting(
		*selected_segment_mesh, neighbor_query,
		vertex_to_point_map);
	sorting.sort();

	// Create an instance of the region growing class.
	Region_growing_mesh_cgal region_growing(
		face_range, neighbor_query, region_type,
		sorting.seed_map());

	// Run the algorithm.
	regions_for_cgal regions;
	region_growing.detect(std::back_inserter(regions));

	//paring region to mesh
	int rg_ind = 0;
	std::pair<int, float> rid_maxarea(-1, 0.0f);
	std::vector<std::vector<int>> planar_regions;
	for (auto rg : regions)
	{
		if (!rg.empty())
		{
			std::vector<int> rg_tmp;
			float rg_area = 0.0f;
			for (auto fi : rg)
			{
				rg_tmp.push_back(fi);
				if (selected_main_faces.size() == 1)//means mesh clustering
					face_segment_id[id_face[fi]] = rg_ind;
				else //means segment clustering
					rg_area += CGAL::Polygon_mesh_processing::face_area(segment_fid_face_map[fi], *selected_segment_mesh);
			}
			if (rid_maxarea.second < rg_area)
			{
				rid_maxarea.first = rg_ind;
				rid_maxarea.second = rg_area;
			}
			planar_regions.push_back(rg_tmp);
			++rg_ind;
		}
	}

	if (selected_main_faces.size() == 1)
		CGAL::Three::Three::information("New regions generated: " + QString::number(planar_regions.size()) + " segments.");
	if (!regions.empty() && selected_main_faces.size() != 1)
		selected_main_faces.insert(selected_main_faces.end(), planar_regions[rid_maxarea.first].begin(), planar_regions[rid_maxarea.first].end());
}

void Scene_surface_mesh_item::region_growing_on_pcl
(
	Point_range_cgal& pcl_in,
	SMesh* selected_segment_mesh,
	double& max_distance_to_plane_,
	double& max_accepted_angle_,
	int& min_region_size_,
	int& k_neighbors,
	std::map<int, face_descriptor> &segment_fid_face_map,
	std::vector<int> &selected_main_faces
)
{
	// Default parameter values for the data file polygon_mesh.off.
	const std::size_t k = k_neighbors;
	const Kernel::FT          max_distance_to_plane = Kernel::FT(max_distance_to_plane_);
	const Kernel::FT          max_accepted_angle = Kernel::FT(max_accepted_angle_);
	const std::size_t min_region_size = min_region_size_;

	// Create instances of the classes Neighbor_query and Region_type.
	Neighbor_query_pcl_cgal neighbor_query(pcl_in, k, pcl_in.point_map());

	region_type_pcl_cgal region_type(
		pcl_in,
		max_distance_to_plane, max_accepted_angle, min_region_size,
		pcl_in.point_map(), pcl_in.normal_map());

	// Create an instance of the region growing class.
	Region_growing_pcl_cgal region_growing(
		pcl_in, neighbor_query, region_type);

	// Run the algorithm.
	regions_for_cgal regions;
	region_growing.detect(std::back_inserter(regions));

	//paring region to mesh
	int rg_ind = 0;
	std::pair<int, float> rid_maxarea(-1, 0.0f);
	std::vector<std::vector<int>> planar_regions;
	for (auto rg : regions)
	{
		if (!rg.empty())
		{
			std::vector<int> rg_tmp;
			float rg_area = 0.0f;
			for (auto pi : rg)
			{
				rg_tmp.push_back(pi);
				if (selected_main_faces.size() == 1)//means mesh clustering
					face_segment_id[id_face[pi]] = rg_ind;
				else //means segment clustering
					rg_area += CGAL::Polygon_mesh_processing::face_area(segment_fid_face_map[pi], *selected_segment_mesh);
			}
			if (rid_maxarea.second < rg_area)
			{
				rid_maxarea.first = rg_ind;
				rid_maxarea.second = rg_area;
			}
			planar_regions.push_back(rg_tmp);
			++rg_ind;
		}
	}
	if (!regions.empty() && selected_main_faces.size() != 1)
		selected_main_faces.insert(selected_main_faces.end(), planar_regions[rid_maxarea.first].begin(), planar_regions[rid_maxarea.first].end());
}

void Scene_surface_mesh_item_priv::initialize_colors() const
{
	// Fill indices map and get max subdomain value
	int max = 0;
	min_patch_id = (std::numeric_limits<int>::max)();
	BOOST_FOREACH(face_descriptor fd, faces(*smesh_)) {
		max = (std::max)(max, fpatch_id_map[fd]);
		min_patch_id = (std::min)(min_patch_id, fpatch_id_map[fd]);
	}
	if (item->property("recompute_colors").toBool())
	{
		colors_.clear();
		compute_color_map(item->color(), (std::max)(1, max + 1 - min_patch_id),
			std::back_inserter(colors_));
	}
}

void Scene_surface_mesh_item_priv::initializeBuffers(CGAL::Three::Viewer_interface* viewer)const
{
	item->getTriangleContainer(2)->initializeBuffers(viewer);
	item->getTriangleContainer(1)->initializeBuffers(viewer);
	item->getTriangleContainer(0)->initializeBuffers(viewer);
	item->getEdgeContainer(1)->initializeBuffers(viewer);
	item->getEdgeContainer(0)->initializeBuffers(viewer);
	item->getEdgeContainer(2)->initializeBuffers(viewer);

	item->getPointContainer(0)->initializeBuffers(viewer);


	//Clean-up
	item->getPointContainer(0)->setFlatDataSize(vertices(*smesh_).size() * 3);
	item->getTriangleContainer(2)->setFlatDataSize(selected_flat_vertices_size);
	item->getTriangleContainer(1)->setFlatDataSize(flat_vertices_size);
	item->getTriangleContainer(0)->setIdxSize(idx_data_size);
	item->getEdgeContainer(1)->setIdxSize(idx_feature_edge_data_size);
	item->getEdgeContainer(0)->setIdxSize(idx_edge_data_size);
	item->getEdgeContainer(2)->setIdxSize(idx_boundary_edge_data_size);

	smooth_vertices.resize(0);
	smooth_normals.resize(0);
	flat_vertices.resize(0);
	flat_normals.resize(0);
	f_colors.resize(0);
	v_colors.resize(0);
	idx_data_.resize(0);
	idx_edge_data_.resize(0);
	idx_feature_edge_data_.resize(0);
	idx_boundary_edge_data_.resize(0);

	smooth_vertices.shrink_to_fit();
	smooth_normals.shrink_to_fit();
	flat_vertices.shrink_to_fit();
	flat_normals.shrink_to_fit();
	f_colors.shrink_to_fit();
	v_colors.shrink_to_fit();
	idx_data_.shrink_to_fit();
	idx_edge_data_.shrink_to_fit();
	idx_feature_edge_data_.shrink_to_fit();
	idx_boundary_edge_data_.shrink_to_fit();
}

void Scene_surface_mesh_item::computeUV() const	
{
	if (d->is_texture_initialized == false)
	{
		for (int tex_item_i = 0; tex_item_i < this->texture_images.size();++tex_item_i)
		{
			QPointF min(FLT_MAX, FLT_MAX), max(-FLT_MAX, -FLT_MAX);
			SMesh::Property_map<halfedge_descriptor, std::pair<float, float> > uv_tmp;
			uv_tmp = (*(d->texture_items[tex_item_i]->mesh_in)).add_property_map<halfedge_descriptor, std::pair<float, float> >("h:uv", std::make_pair(0.0f, 0.0f)).first;

			BOOST_FOREACH(face_descriptor fd, faces(*(d->texture_items[tex_item_i]->mesh_in)))
			{
				if (d->texture_items[tex_item_i]->face_texcoord[fd].empty() == true)
					break;

				SMesh::Halfedge_around_face_circulator he(halfedge(fd, *(d->texture_items[tex_item_i]->mesh_in)), *(d->texture_items[tex_item_i]->mesh_in));
				SMesh::Halfedge_around_face_circulator end = he;
				int ind = 0;

				CGAL_For_all(he, end)
				{
					//halfedge_descriptor hd(*he);

					float u = d->texture_items[tex_item_i]->face_texcoord[fd][ind];
					float v = d->texture_items[tex_item_i]->face_texcoord[fd][++ind];
					put(uv_tmp, *he, std::make_pair(u, v));

					if (u < min.x())
						min.setX(u);
					if (u > max.x())
						max.setX(u);
					if (v < min.y())
						min.setY(v);
					if (v > max.y())
						max.setY(v);

					++ind;
				}
			}

			//Rendering
			d->texture_items[tex_item_i]->uv_map = uv_tmp;
			d->texture_items[tex_item_i]->invalidateOpenGLBuffers();
		}
		d->is_texture_initialized = true;
	}
}

void Scene_surface_mesh_item::draw(CGAL::Three::Viewer_interface* viewer) const
{
	if (!isInit() && viewer->context()->isValid())
		initGL();
	if (getBuffersFilled())
		if (!getBuffersInit(viewer))
		{
			d->initializeBuffers(viewer);
			setBuffersInit(viewer, true);
		}

	if (renderingMode() == Gouraud)
	{
		//getTriangleContainer(0)->setColor(color());
		getTriangleContainer(0)->setColor(QColor(128, 128, 128));
		getTriangleContainer(0)->setSelected(is_selected);
		getTriangleContainer(0)->setAlpha(alpha());
		getTriangleContainer(0)->draw(viewer, !d->has_vcolors);

		//this->drawEdges(viewer);

	}
	else if (renderingMode() == TextureMode)
	{
		computeUV();

		for (int tex_item_i = 0; tex_item_i < this->texture_images.size(); ++tex_item_i)
			d->texture_items[tex_item_i]->draw(viewer);
	}
	else if (renderingMode() == TextureModePlusFlatEdges)
	{
		getTriangleContainer(1)->setColor(color());
		getTriangleContainer(1)->setSelected(is_selected);
		getTriangleContainer(1)->setAlpha(alpha());
		getTriangleContainer(1)->draw(viewer, !d->has_fcolors);

		computeUV();

		for (int tex_item_i = 0; tex_item_i < this->texture_images.size(); ++tex_item_i)
			d->texture_items[tex_item_i]->draw(viewer);

		this->drawEdges(viewer);

	}
	else if (renderingMode() == Emphasizing)
	{

		getTriangleContainer(2)->setColor(color());
		getTriangleContainer(2)->setSelected(is_selected);
		getTriangleContainer(2)->setAlpha(alpha());
		getTriangleContainer(2)->draw(viewer, !d->has_fcolors);

		computeUV();

		for (int tex_item_i = 0; tex_item_i < this->texture_images.size(); ++tex_item_i)
			d->texture_items[tex_item_i]->draw(viewer);

		this->drawEdges(viewer);
	}
	else
	{
		getTriangleContainer(1)->setColor(color());
		getTriangleContainer(1)->setSelected(is_selected);
		getTriangleContainer(1)->setAlpha(alpha());
		getTriangleContainer(1)->draw(viewer, !d->has_fcolors);
	}

		//depends on the checkbox
	this->drawPoints(viewer);
	this->drawEdges(viewer);
}

void Scene_surface_mesh_item::drawEdges(CGAL::Three::Viewer_interface* viewer) const
{
	if (!isInit())
		initGL();
	if (getBuffersFilled() &&
		!getBuffersInit(viewer))
	{
		d->initializeBuffers(viewer);
		setBuffersInit(viewer, true);
	}

	if (edgesShow)
	{
		getEdgeContainer(0)->setSelected(is_selected);
		getEdgeContainer(0)->setColor(color().lighter(50));
		getEdgeContainer(0)->draw(viewer, true);
	}

	if (segmentBoundryShow)
	{
		getEdgeContainer(2)->setSelected(false);
		getEdgeContainer(2)->setColor(QColor(Qt::blue));
		getEdgeContainer(2)->draw(viewer, true);
	}
	else
	{
		//getEdgeContainer(2)->setSelected(false);
		//getEdgeContainer(2)->setColor(color().lighter(50));
		//getEdgeContainer(2)->draw(viewer, true);
		//if (edgesShow)
		//{
		//	getEdgeContainer(1)->setSelected(false);
		//	getEdgeContainer(1)->setColor(QColor(Qt::black));
		//	getEdgeContainer(1)->draw(viewer, true);
		//}
	}

	if (meshBoundaryShow)
	{
		getEdgeContainer(1)->setSelected(false);
		getEdgeContainer(1)->setColor(QColor(Qt::red));
		getEdgeContainer(1)->draw(viewer, true);
	}

	//if (d->has_feature_edges)
	//{
	//	getEdgeContainer(1)->setSelected(false);
	//	getEdgeContainer(1)->setColor(QColor(Qt::red));
	//	getEdgeContainer(1)->draw(viewer, true);
	//}
}

void Scene_surface_mesh_item::drawPoints(CGAL::Three::Viewer_interface* viewer) const
{
	if (pointShow)
	{
		if (!isInit())
			initGL();
		if (getBuffersFilled() &&
			!getBuffersInit(viewer))
		{
			d->initializeBuffers(viewer);
			setBuffersInit(viewer, true);
		}
		getPointContainer(0)->setSelected(is_selected);
		getPointContainer(0)->setColor(color());

		//CGAL::Three::Three::setDefaultPointSize(PointSize());
		//getPointContainer(0)->setColor(color().lighter(PointSize()));

		getPointContainer(0)->draw(viewer, true);
	}
}

void Scene_surface_mesh_item::selection_changed(bool p_is_selected)
{
	if (p_is_selected != is_selected)
	{
		is_selected = p_is_selected;
	}
}

bool Scene_surface_mesh_item::supportsRenderingMode(RenderingMode m) const
{
	return (m == FlatPlusEdges || m == Wireframe || m == Flat || m == Gouraud || m == Points
		|| m == TextureMode || m == TextureModePlusFlatEdges || m == Emphasizing);
}


CGAL::Three::Scene_item::Bbox Scene_surface_mesh_item::bbox() const
{
	if (!is_bbox_computed)
		compute_bbox();
	return _bbox;
}

bool
Scene_surface_mesh_item::isEmpty() const
{

	return num_vertices(*d->smesh_) == 0;
}

QString Scene_surface_mesh_item::toolTip() const
{
	QString str = QObject::tr("<p>Surface_mesh <b>%1</b> (mode: %5, color: %6)</p>"
		"<p>Number of vertices: %2<br />"
		"Number of edges: %3<br />"
		"Number of faces: %4</p>")
		.arg(this->name())
		.arg(num_vertices(*d->smesh_))
		.arg(num_edges(*d->smesh_))
		.arg(num_faces(*d->smesh_))
		.arg(this->renderingModeName())
		.arg(this->color().name());
	str += QString("<br />Number of isolated vertices: %1<br />").arg(getNbIsolatedvertices());
	return str;
}

void Scene_surface_mesh_item_priv::checkFloat()const
{
#if CGAL_IS_FLOAT == 1
	floated = true;
#endif
}

void Scene_surface_mesh_item_priv::triangulate_convex_facet(face_descriptor fd,
	SMesh::Property_map<face_descriptor, Kernel::Vector_3>* fnormals,
	SMesh::Property_map<face_descriptor, CGAL::Color>* fcolors,
	boost::property_map< SMesh, boost::vertex_index_t >::type* im,
	Scene_item_rendering_helper::Gl_data_names name,
	bool index) const
{
	Point p0, p1, p2;
	SMesh::Halfedge_around_face_circulator he(halfedge(fd, *smesh_), *smesh_);
	SMesh::Halfedge_around_face_circulator he_end = he;

	while (next(*he, *smesh_) != prev(*he_end, *smesh_))
	{
		++he;
		vertex_descriptor v0(target(*he_end, *smesh_)),
			v1(target(*he, *smesh_)),
			v2(target(next(*he, *smesh_), *smesh_));
		p0 = smesh_->point(v0);
		p1 = smesh_->point(v1);
		p2 = smesh_->point(v2);
		if (!index)
		{
			CGAL::Color* color;
			if (has_fpatch_id)
			{
				QColor c = item->color_vector()[fpatch_id_map[fd] - min_patch_id];
				color = new CGAL::Color(c.red(), c.green(), c.blue());
			}
			else if (has_fcolors)
				color = &(*fcolors)[fd];
			else
				color = 0;
			addFlatData(p0,
				(*fnormals)[fd],
				color,
				name);
			addFlatData(p1,
				(*fnormals)[fd],
				color,
				name);

			addFlatData(p2,
				(*fnormals)[fd],
				color,
				name);
			if (has_fpatch_id)
				delete color;
		}
		else if (name.testFlag(Scene_item_rendering_helper::GEOMETRY))
		{
			idx_data_.push_back((*im)[v0]);
			idx_data_.push_back((*im)[v1]);
			idx_data_.push_back((*im)[v2]);
		}
	}
}

void
Scene_surface_mesh_item_priv::triangulate_facet(face_descriptor fd,
	SMesh::Property_map<face_descriptor, Kernel::Vector_3>* fnormals,
	SMesh::Property_map<face_descriptor, CGAL::Color>* fcolors,
	boost::property_map< SMesh, boost::vertex_index_t >::type* im,
	Scene_item_rendering_helper::Gl_data_names name,
	bool index) const
{

	//Computes the normal of the facet
	Kernel::Vector_3 normal = get(*fnormals, fd);
	if (normal == CGAL::NULL_VECTOR)
	{
		boost::graph_traits<SMesh>::halfedge_descriptor start = prev(halfedge(fd, *smesh_), *smesh_);
		boost::graph_traits<SMesh>::halfedge_descriptor hd = halfedge(fd, *smesh_);
		boost::graph_traits<SMesh>::halfedge_descriptor next_ = next(hd, *smesh_);
		do
		{
			const Point_3& pa = smesh_->point(target(hd, *smesh_));
			const Point_3& pb = smesh_->point(target(next_, *smesh_));
			const Point_3& pc = smesh_->point(target(prev(hd, *smesh_), *smesh_));
			if (!CGAL::collinear(pa, pb, pc))
			{
				normal = CGAL::cross_product(pb - pa, pc - pa);
				break;
			}
			next_ = next(next_, *smesh_);
		} while (next_ != start);

		if (normal == CGAL::NULL_VECTOR) // No normal could be computed, return
		{
			qDebug() << "Warning : normal is not valid. Facet not displayed";
			return;
		}
	}
	//check if normal contains NaN values
	if (normal.x() != normal.x() || normal.y() != normal.y() || normal.z() != normal.z())
	{
		qDebug() << "Warning : normal is not valid. Facet not displayed";
		return;
	}

	typedef FacetTriangulator<SMesh, Kernel, boost::graph_traits<SMesh>::vertex_descriptor> FT;
	const CGAL::qglviewer::Vec off = static_cast<CGAL::Three::Viewer_interface*>(CGAL::QGLViewer::QGLViewerPool().first())->offset();
	Kernel::Vector_3 offset(off.x, off.y, off.z);
	FT triangulation(fd, normal, smesh_, offset);
	//iterates on the internal faces
	for (FT::CDT::Finite_faces_iterator
		ffit = triangulation.cdt->finite_faces_begin(),
		end = triangulation.cdt->finite_faces_end();
		ffit != end; ++ffit)
	{
		if (ffit->info().is_external)
			continue;
		//add the vertices to the positions
		//adds the vertices, normals and colors to the appropriate vectors
		if (!index)
		{
			CGAL::Color* color;
			if (has_fpatch_id)
			{
				QColor c = item->color_vector()[fpatch_id_map[fd] - min_patch_id];
				color = new CGAL::Color(c.red(), c.green(), c.blue());
			}
			else if (has_fcolors)
				color = &(*fcolors)[fd];
			else
				color = 0;

			addFlatData(ffit->vertex(0)->point() - offset,
				(*fnormals)[fd],
				color,
				name);
			addFlatData(ffit->vertex(1)->point() - offset,
				(*fnormals)[fd],
				color,
				name);

			addFlatData(ffit->vertex(2)->point() - offset,
				(*fnormals)[fd],
				color,
				name);
			if (has_fpatch_id)
				delete color;
		}
		//adds the indices to the appropriate vector
		else
		{
			if (name.testFlag(Scene_item_rendering_helper::GEOMETRY))
			{
				idx_data_.push_back((*im)[triangulation.v2v[ffit->vertex(0)]]);
				idx_data_.push_back((*im)[triangulation.v2v[ffit->vertex(1)]]);
				idx_data_.push_back((*im)[triangulation.v2v[ffit->vertex(2)]]);
			}
		}

	}
}
void delete_aabb_tree(Scene_surface_mesh_item* item)
{
	QVariant aabb_tree_property = item->property(aabb_property_name);
	if (aabb_tree_property.isValid()) {
		void* ptr = aabb_tree_property.value<void*>();
		Input_facets_AABB_tree* tree = static_cast<Input_facets_AABB_tree*>(ptr);
		if (tree) {
			delete tree;
			tree = 0;
		}
		item->setProperty(aabb_property_name, QVariant());
	}
}

Scene_surface_mesh_item::~Scene_surface_mesh_item()
{
	delete_aabb_tree(this);
	CGAL::QGLViewer* viewer = *CGAL::QGLViewer::QGLViewerPool().begin();
	if (viewer)
	{
		CGAL::Three::Viewer_interface* v = qobject_cast<CGAL::Three::Viewer_interface*>(viewer);

		//Clears the targeted Id
		if (d)
		{
			BOOST_FOREACH(TextItem * item, d->targeted_id)
				v->textRenderer()->removeText(item);
		}
		//Remove vertices textitems
		if (d->textVItems)
		{
			v->textRenderer()->removeTextList(d->textVItems);
			delete d->textVItems;
			d->textVItems = NULL;
		}
		//Remove edges textitems
		if (d->textEItems)
		{
			v->textRenderer()->removeTextList(d->textEItems);
			delete d->textEItems;
			d->textEItems = NULL;
		}
		//Remove faces textitems
		if (d->textFItems)
		{
			v->textRenderer()->removeTextList(d->textFItems);
			delete d->textFItems;
			d->textFItems = NULL;
		}
	}
	delete d;
}
SMesh* Scene_surface_mesh_item::polyhedron() { return d->smesh_; }
const SMesh* Scene_surface_mesh_item::polyhedron() const { return d->smesh_; }

void Scene_surface_mesh_item::compute_bbox()const
{
	SMesh::Property_map<vertex_descriptor, Point_3> pprop = d->smesh_->points();
	CGAL::Bbox_3 bbox;

	BOOST_FOREACH(vertex_descriptor vd, vertices(*d->smesh_))
	{
		bbox = bbox + pprop[vd].bbox();
	}
	_bbox = Bbox(bbox.xmin(), bbox.ymin(), bbox.zmin(),
		bbox.xmax(), bbox.ymax(), bbox.zmax());
	is_bbox_computed = true;
}

void Scene_surface_mesh_item::itemAboutToBeDestroyed(Scene_item* item)
{
	Scene_item::itemAboutToBeDestroyed(item);
	if (d && d->smesh_ && item == this)
	{
		delete d->smesh_;
		d->smesh_ = NULL;
	}
}

void* Scene_surface_mesh_item_priv::get_aabb_tree()
{
	QVariant aabb_tree_property = item->property(aabb_property_name);
	if (aabb_tree_property.isValid()) {
		void* ptr = aabb_tree_property.value<void*>();
		return static_cast<Input_facets_AABB_tree*>(ptr);
	}
	else {
		QApplication::setOverrideCursor(Qt::WaitCursor);
		SMesh* sm = item->polyhedron();
		if (sm) {
			sm->collect_garbage();
			Input_facets_AABB_tree* tree =
				new Input_facets_AABB_tree();
			int index = 0;
			BOOST_FOREACH(face_descriptor f, faces(*sm))
			{
				//if face is degenerate, skip it
				if (CGAL::is_triangle(halfedge(f, *sm), *sm)
					&& CGAL::Polygon_mesh_processing::is_degenerate_triangle_face(f, *sm))
					continue;
				//if face not triangle, triangulate corresponding primitive before adding it to the tree
				if (!CGAL::is_triangle(halfedge(f, *sm), *sm))
				{
					Kernel::Vector_3 normal = CGAL::Polygon_mesh_processing::compute_face_normal(f, *sm);
					index += 3;
					Q_FOREACH(Kernel::Triangle_3 triangle, triangulate_primitive(f, normal))
					{
						Primitive primitive(triangle, f);
						tree->insert(primitive);
					}
				}
				else
				{
					Kernel::Triangle_3 triangle(
						sm->point(target(halfedge(f, *sm), *sm)),
						sm->point(target(next(halfedge(f, *sm), *sm), *sm)),
						sm->point(target(next(next(halfedge(f, *sm), *sm), *sm), *sm))
					);
					Primitive primitive(triangle, f);
					tree->insert(primitive);
				}
			}
			item->setProperty(aabb_property_name,
				QVariant::fromValue<void*>(tree));
			QApplication::restoreOverrideCursor();
			return tree;
		}
		else return 0;
	}
}

void
Scene_surface_mesh_item::select(double orig_x,
	double orig_y,
	double orig_z,
	double dir_x,
	double dir_y,
	double dir_z)
{
	SMesh* sm = d->smesh_;
	std::size_t vertex_to_emit = 0;
	typedef Input_facets_AABB_tree Tree;
	typedef Tree::Intersection_and_primitive_id<Kernel::Ray_3>::Type Object_and_primitive_id;
	int selected_facet_num = 0, selected_vertex_num = 0, selected_edge_num = 0;

	Tree* aabb_tree = static_cast<Tree*>(d->get_aabb_tree());
	if (aabb_tree)
	{
		const Kernel::Point_3 ray_origin(orig_x, orig_y, orig_z);
		const Kernel::Vector_3 ray_dir(dir_x, dir_y, dir_z);
		const Kernel::Ray_3 ray(ray_origin, ray_dir);
		typedef std::list<Object_and_primitive_id> Intersections;
		Intersections intersections;
		aabb_tree->all_intersections(ray, std::back_inserter(intersections));
		Intersections::iterator closest = intersections.begin();
		if (closest != intersections.end())
		{

			const Kernel::Point_3* closest_point =
				boost::get<Kernel::Point_3>(&(closest->first));
			for (Intersections::iterator
				it = boost::next(intersections.begin()),
				end = intersections.end();
				it != end; ++it)
			{
				if (!closest_point) {
					closest = it;
				}
				else {
					const Kernel::Point_3* it_point =
						boost::get<Kernel::Point_3>(&it->first);
					if (it_point &&
						(ray_dir * (*it_point - *closest_point)) < 0)
					{
						closest = it;
						closest_point = it_point;
					}
				}
			}
			if (closest_point) {
				face_descriptor selected_face = closest->second;

				// The computation of the nearest vertex may be costly.  Only
				// do it if some objects are connected to the signal
				// 'selected_vertex'.
				if (QObject::receivers(SIGNAL(selected_vertex(void*))) > 0)
				{

					SMesh::Halfedge_around_face_circulator he_it(sm->halfedge(selected_face), *sm), around_end(he_it);

					vertex_descriptor v = sm->target(*he_it), nearest_v = v;

					Kernel::FT sq_dist = CGAL::squared_distance(*closest_point,
						sm->point(v));
					while (++he_it != around_end) {
						v = sm->target(*he_it);
						Kernel::FT new_sq_dist = CGAL::squared_distance(*closest_point,
							sm->point(v));
						if (new_sq_dist < sq_dist) {
							sq_dist = new_sq_dist;
							nearest_v = v;
						}
					}
					//bottleneck
					vertex_to_emit = static_cast<std::size_t>(nearest_v);
				}

				if (QObject::receivers(SIGNAL(selected_edge(void*))) > 0
					|| QObject::receivers(SIGNAL(selected_halfedge(void*))) > 0)
				{
					SMesh::Halfedge_around_face_circulator he_it(sm->halfedge(selected_face), *sm), around_end(he_it);

					halfedge_descriptor nearest_h = *he_it;
					Kernel::FT sq_dist =
						CGAL::squared_distance(*closest_point,
							Kernel::Segment_3(sm->point(sm->target(*he_it)),
								sm->point(
									sm->target(
										sm->opposite(*he_it)))));

					while (++he_it != around_end)
					{
						Kernel::FT new_sq_dist =
							CGAL::squared_distance(*closest_point,
								Kernel::Segment_3(sm->point(sm->target(*he_it)),
									sm->point(
										sm->target(
											sm->opposite(*he_it)))));
						if (new_sq_dist < sq_dist) {
							sq_dist = new_sq_dist;
							nearest_h = *he_it;
						}
					}
					std::size_t s_nearest_h = static_cast<std::size_t>(nearest_h);
					std::size_t s_nearest_e = static_cast<std::size_t>(nearest_h) / 2;
					Q_EMIT selected_halfedge(reinterpret_cast<void*>(s_nearest_h));
					Q_EMIT selected_edge(reinterpret_cast<void*>(s_nearest_e));
				}

				Q_EMIT selected_vertex(reinterpret_cast<void*>(vertex_to_emit));
				std::size_t s_selected_f = static_cast<std::size_t>(selected_face);
				Q_EMIT selected_facet(reinterpret_cast<void*>(s_selected_f));
			}
		}
	}
	Scene_item::select(orig_x, orig_y, orig_z, dir_x, dir_y, dir_z);
	Q_EMIT selection_done();
}

//swap the facet properties according to the 'Surface_mesh.h' erase procedure
void  Scene_surface_mesh_item::map_garbage_properties
(
	std::map<face_descriptor, face_descriptor> &removed_unremoved
)
{
	if (!removed_unremoved.empty())
	{
		std::map<face_descriptor, int> face_label_update;
		std::map<face_descriptor, QColor> face_color_update;
		std::map<face_descriptor, std::vector<float>> face_texcoord_update;
		std::map<face_descriptor, int> face_textureid_update;
		std::map<face_descriptor, float> face_prob_update;
		std::map<face_descriptor, int> face_segment_id_update;
		BOOST_FOREACH(face_descriptor fd, faces(*polyhedron()))
		{
			std::map<face_descriptor, face_descriptor>::iterator it = removed_unremoved.find(fd);
			if (it != removed_unremoved.end())
			{
				face_descriptor swaped_fd = removed_unremoved[fd];
				face_label_update[fd] = face_label[swaped_fd];
				face_color_update[fd] = face_color[swaped_fd];
				face_texcoord_update[fd] = face_texcoord[swaped_fd];
				face_textureid_update[fd] = face_textureid[swaped_fd];
				face_prob_update[fd] = label_probabilities[swaped_fd];
				face_segment_id_update[fd] = face_segment_id[swaped_fd];
			}
			else
			{
				face_label_update[fd] = face_label[fd];
				face_color_update[fd] = face_color[fd];
				face_texcoord_update[fd] = face_texcoord[fd];
				face_textureid_update[fd] = face_textureid[fd];
				face_prob_update[fd] = label_probabilities[fd];
				face_segment_id_update[fd] = face_segment_id[fd];
			}
		}
		face_label.clear(); face_label = face_label_update;
		face_color.clear(); face_color = face_color_update;
		face_texcoord.clear(); face_texcoord = face_texcoord_update;
		face_textureid.clear(); face_textureid = face_textureid_update;
		label_probabilities.clear();  label_probabilities = face_prob_update;
		face_segment_id.clear();  face_segment_id = face_segment_id_update;

		segments.clear();
		computeSegmentBoundary();
	}
}

void Scene_surface_mesh_item::invalidateOpenGLBuffers() {
	invalidate(ALL);
}

void Scene_surface_mesh_item::invalidate(Gl_data_names name)
{
	Q_EMIT item_is_about_to_be_changed();

	if (name.testFlag(GEOMETRY) && is_facet_deleted)
	{
		is_bbox_computed = false;
		delete_aabb_tree(this);

		//d->smesh_->collect_garbage();
		std::map<face_descriptor, face_descriptor> removed_unremoved;
		d->smesh_->collect_garbage(removed_unremoved);
		map_garbage_properties(removed_unremoved);

		d->invalidate_stats();
		is_facet_deleted = false;
	}

	setBuffersFilled(false);
	Q_FOREACH(CGAL::QGLViewer * v, CGAL::QGLViewer::QGLViewerPool())
	{
		CGAL::Three::Viewer_interface* viewer = static_cast<CGAL::Three::Viewer_interface*>(v);
		if (viewer == NULL)
			continue;
		setBuffersInit(viewer, false);
		//viewer->update();
	}

	getTriangleContainer(1)->reset_vbos(name);
	getTriangleContainer(0)->reset_vbos(name);
	getEdgeContainer(1)->reset_vbos(name);
	getEdgeContainer(0)->reset_vbos(name);

	getTriangleContainer(2)->reset_vbos(name);
	getEdgeContainer(2)->reset_vbos(name);

	getPointContainer(0)->reset_vbos(name);

	if (isInit())
		processData(name);
	else
		initGL();

	if (!d->all_displayed)
		d->killIds();
	else
	{
		Q_FOREACH(CGAL::QGLViewer * v, CGAL::QGLViewer::QGLViewerPool())
		{
			CGAL::Three::Viewer_interface* viewer = static_cast<CGAL::Three::Viewer_interface*>(v);
			if (viewer == NULL)
				continue;
			d->killIds();
			if (d->vertices_displayed)
			{
				printVertexIds(viewer);
			}
			if (d->edges_displayed)
			{
				printEdgeIds(viewer);
			}
			if (d->faces_displayed)
			{
				printFaceIds(viewer);
			}
		}
	}
}

QList<Kernel::Triangle_3> Scene_surface_mesh_item_priv::triangulate_primitive(face_descriptor fit,
	Kernel::Vector_3 normal)
{
	typedef FacetTriangulator<SMesh, Kernel, boost::graph_traits<SMesh>::vertex_descriptor> FT;
	//The output list
	QList<Kernel::Triangle_3> res;
	//check if normal contains NaN values
	if (normal.x() != normal.x() || normal.y() != normal.y() || normal.z() != normal.z())
	{
		qDebug() << "Warning in triangulation of the selection item: normal contains NaN values and is not valid.";
		return QList<Kernel::Triangle_3>();
	}
	FT triangulation(fit, normal, smesh_);
	//iterates on the internal faces to add the vertices to the positions
	//and the normals to the appropriate vectors
	for (FT::CDT::Finite_faces_iterator
		ffit = triangulation.cdt->finite_faces_begin(),
		end = triangulation.cdt->finite_faces_end();
		ffit != end; ++ffit)
	{
		if (ffit->info().is_external)
			continue;


		res << Kernel::Triangle_3(ffit->vertex(0)->point(),
			ffit->vertex(1)->point(),
			ffit->vertex(2)->point());

	}
	return res;
}

void Scene_surface_mesh_item::invalidate_aabb_tree()
{
	delete_aabb_tree(this);
}

bool Scene_surface_mesh_item::intersect_face(double orig_x,
	double orig_y,
	double orig_z,
	double dir_x,
	double dir_y,
	double dir_z,
	const face_descriptor& f)
{
	typedef Input_facets_AABB_tree Tree;
	typedef Tree::Object_and_primitive_id Object_and_primitive_id;

	Tree* aabb_tree = static_cast<Tree*>(d->get_aabb_tree());
	if (aabb_tree)
	{
		const Kernel::Point_3 ray_origin(orig_x, orig_y, orig_z);
		const Kernel::Vector_3 ray_dir(dir_x, dir_y, dir_z);
		const Kernel::Ray_3 ray(ray_origin, ray_dir);
		typedef std::list<Object_and_primitive_id> Intersections;
		Intersections intersections;
		aabb_tree->all_intersections(ray, std::back_inserter(intersections));
		Intersections::iterator closest = intersections.begin();
		if (closest != intersections.end())
		{
			const Kernel::Point_3* closest_point =
				CGAL::object_cast<Kernel::Point_3>(&closest->first);
			for (Intersections::iterator
				it = boost::next(intersections.begin()),
				end = intersections.end();
				it != end; ++it)
			{
				if (!closest_point) {
					closest = it;
				}
				else {
					const Kernel::Point_3* it_point =
						CGAL::object_cast<Kernel::Point_3>(&it->first);
					if (it_point &&
						(ray_dir * (*it_point - *closest_point)) < 0)
					{
						closest = it;
						closest_point = it_point;
					}
				}
			}
			if (closest_point)
			{
				face_descriptor intersected_face = closest->second;
				return intersected_face == f;
			}
		}
	}
	return false;

}

void Scene_surface_mesh_item::setItemIsMulticolor(bool b)
{
	if (b)
	{
		d->fpatch_id_map = d->smesh_->add_property_map<face_descriptor, int>("f:patch_id", 1).first;
		d->has_fcolors = true;
	}
	else
	{
		if (d->smesh_->property_map<face_descriptor, int>("f:patch_id").second)
		{
			d->fpatch_id_map = d->smesh_->property_map<face_descriptor, int>("f:patch_id").first;
			d->smesh_->remove_property_map(d->fpatch_id_map);
			d->has_fcolors = false;
		}
		if (d->smesh_->property_map<face_descriptor, CGAL::Color >("f:color").second)
		{
			SMesh::Property_map<face_descriptor, CGAL::Color> pmap =
				d->smesh_->property_map<face_descriptor, CGAL::Color >("f:color").first;
			d->smesh_->remove_property_map(pmap);
			d->has_fcolors = false;
		}
		if (d->smesh_->property_map<vertex_descriptor, CGAL::Color >("v:color").second)
		{
			SMesh::Property_map<vertex_descriptor, CGAL::Color> pmap =
				d->smesh_->property_map<vertex_descriptor, CGAL::Color >("v:color").first;
			d->smesh_->remove_property_map(pmap);
			d->has_vcolors = false;
		}
		this->setProperty("NbPatchIds", 0); //for the joinandsplit_plugin
	}
}

void Scene_surface_mesh_item::show_feature_edges(bool b)
{
	d->has_feature_edges = b;
	if (b)
	{
		d->e_is_feature_map = d->smesh_->add_property_map<boost::graph_traits<SMesh>::edge_descriptor, bool>("e:is_feature").first;
		invalidate(COLORS);
		itemChanged();
	}
}

bool Scene_surface_mesh_item::isItemMulticolor()
{
	return d->has_fcolors || d->has_vcolors;
}

bool Scene_surface_mesh_item::hasPatchIds()
{
	return d->has_fpatch_id;
}

bool
Scene_surface_mesh_item::save(std::ostream& out) const
{
	QApplication::setOverrideCursor(Qt::WaitCursor);
	out.precision(17);
	out << *(d->smesh_);
	QApplication::restoreOverrideCursor();
	return (bool)out;
}

// Write mesh to .PLY file
bool Scene_surface_mesh_item::write_ply_mesh(std::ostream& stream, bool binary) const
{
	Q_ASSERT(d->smesh_ != NULL);

	if (!stream)
		return false;

	if (binary)
		CGAL::set_binary_mode(stream);

	//if the user do not start 3D Annotation, the comments keep same as input
	bool used_old_comments = false;
	if (d->m_comments.empty())
	{
		d->m_comments = this->input_comments;
		used_old_comments = true;
	}

	CGAL::write_PLY(stream, *(d->smesh_), &(d->m_comments), this->vertex_color,
		this->face_label, this->face_color, this->face_texcoord, this->face_textureid,
		this->label_probabilities, this->face_segment_id, this->texture_name, used_old_comments, this->total_labeled_faces);
	CGAL::Three::Three::information("Save successfully!");
	return true;
}

// Gets wrapped face set
std::string& Scene_surface_mesh_item::comments()
{
	return d->m_comments;
}
const std::string& Scene_surface_mesh_item::comments() const
{
	return d->m_comments;
}

void Scene_surface_mesh_item::set_comments(std::string comment_in)
{
	d->m_comments = comment_in;
}

//*******************************************************************//

void
Scene_surface_mesh_item_priv::
invalidate_stats()
{
	number_of_degenerated_faces = (unsigned int)(-1);
	number_of_null_length_edges = (unsigned int)(-1);
	volume = -std::numeric_limits<double>::infinity();
	area = -std::numeric_limits<double>::infinity();
	self_intersect = false;
	genus = -1;
}

QString Scene_surface_mesh_item::computeStats(int type)
{
	double minl, maxl, meanl, midl;
	switch (type)
	{
	case MIN_LENGTH:
	case MAX_LENGTH:
	case MID_LENGTH:
	case MEAN_LENGTH:
	case NB_NULL_LENGTH:
		edges_length(d->smesh_, minl, maxl, meanl, midl, d->number_of_null_length_edges);
	}

	double mini(0), maxi(0), ave(0);
	switch (type)
	{
	case MIN_ANGLE:
	case MAX_ANGLE:
	case MEAN_ANGLE:
		angles(d->smesh_, mini, maxi, ave);
	}
	double min_area, max_area, med_area, mean_area;
	switch (type)
	{
	case MIN_AREA:
	case MAX_AREA:
	case MEAN_AREA:
	case MED_AREA:
		if (!is_triangle_mesh(*d->smesh_))
		{
			return QString("n/a");
		}
		faces_area(d->smesh_, min_area, max_area, mean_area, med_area);
	}
	double min_altitude, min_ar, max_ar, mean_ar;
	switch (type)
	{
	case MIN_ALTITUDE:
	case MIN_ASPECT_RATIO:
	case MAX_ASPECT_RATIO:
	case MEAN_ASPECT_RATIO:
		if (!is_triangle_mesh(*d->smesh_))
		{
			return QString("n/a");
		}
		faces_aspect_ratio(d->smesh_, min_altitude, min_ar, max_ar, mean_ar);
	}

	switch (type)
	{
	case NB_VERTICES:
		return QString::number(num_vertices(*d->smesh_));

	case NB_FACETS:
		return QString::number(num_faces(*d->smesh_));

	case NB_CONNECTED_COMPOS:
	{
		boost::vector_property_map<int,
			boost::property_map<SMesh, boost::face_index_t>::type>
			fccmap(get(boost::face_index, *(d->smesh_)));
		return QString::number(CGAL::Polygon_mesh_processing::connected_components(*(d->smesh_), fccmap));
	}
	case NB_BORDER_EDGES:
	{
		int i = 0;
		BOOST_FOREACH(halfedge_descriptor hd, halfedges(*d->smesh_))
		{
			if (is_border(hd, *d->smesh_))
				++i;
		}
		return QString::number(i);
	}

	case NB_EDGES:
		return QString::number(num_halfedges(*d->smesh_) / 2);

	case NB_DEGENERATED_FACES:
	{
		if (is_triangle_mesh(*d->smesh_))
		{
			if (d->number_of_degenerated_faces == (unsigned int)(-1))
				d->number_of_degenerated_faces = nb_degenerate_faces(d->smesh_);
			return QString::number(d->number_of_degenerated_faces);
		}
		else
			return QString("n/a");
	}
	case AREA:
	{
		if (is_triangle_mesh(*d->smesh_))
		{
			if (d->area == -std::numeric_limits<double>::infinity())
				d->area = CGAL::Polygon_mesh_processing::area(*(d->smesh_));
			return QString::number(d->area);
		}
		else
			return QString("n/a");
	}
	case VOLUME:
	{
		if (is_triangle_mesh(*d->smesh_) && is_closed(*d->smesh_))
		{
			if (d->volume == -std::numeric_limits<double>::infinity())
				d->volume = CGAL::Polygon_mesh_processing::volume(*(d->smesh_));
			return QString::number(d->volume);
		}
		else
			return QString("n/a");
	}
	case SELFINTER:
	{
		//todo : add a test about cache validity
		if (is_triangle_mesh(*d->smesh_))
			d->self_intersect = CGAL::Polygon_mesh_processing::does_self_intersect(*(d->smesh_));
		if (d->self_intersect)
			return QString("Yes");
		else if (is_triangle_mesh(*d->smesh_))
			return QString("No");
		else
			return QString("n/a");
	}
	case GENUS:
	{
		if (!is_closed(*d->smesh_))
		{
			return QString("n/a");
		}
		else if (d->genus == -1)
		{
			std::ptrdiff_t s(num_vertices(*d->smesh_)),
				a(num_halfedges(*d->smesh_) / 2),
				f(num_faces(*d->smesh_));
			d->genus = 1.0 - double(s - a + f) / 2.0;
		}
		if (d->genus < 0)
		{
			return QString("n/a");
		}
		else
		{
			return QString::number(d->genus);
		}

	}
	case MIN_LENGTH:
		return QString::number(minl);
	case MAX_LENGTH:
		return QString::number(maxl);
	case MID_LENGTH:
		return QString::number(midl);
	case MEAN_LENGTH:
		return QString::number(meanl);
	case NB_NULL_LENGTH:
		return QString::number(d->number_of_null_length_edges);

	case MIN_ANGLE:
		return QString::number(mini);
	case MAX_ANGLE:
		return QString::number(maxi);
	case MEAN_ANGLE:
		return QString::number(ave);
	case HOLES:
		return QString::number(nb_holes(d->smesh_));

	case MIN_AREA:
		return QString::number(min_area);
	case MAX_AREA:
		return QString::number(max_area);
	case MED_AREA:
		return QString::number(med_area);
	case MEAN_AREA:
		return QString::number(mean_area);
	case MIN_ALTITUDE:
		return QString::number(min_altitude);
	case MIN_ASPECT_RATIO:
		return QString::number(min_ar);
	case MAX_ASPECT_RATIO:
		return QString::number(max_ar);
	case MEAN_ASPECT_RATIO:
		return QString::number(mean_ar);
	case IS_PURE_TRIANGLE:
		if (is_triangle_mesh(*d->smesh_))
			return QString("yes");
		else
			return QString("no");
	case IS_PURE_QUAD:
		if (is_quad_mesh(*d->smesh_))
			return QString("yes");
		else
			return QString("no");
	}
	return QString();
}

CGAL::Three::Scene_item::Header_data Scene_surface_mesh_item::header() const
{
	CGAL::Three::Scene_item::Header_data data;
	//categories

	data.categories.append(std::pair<QString, int>(QString("Properties"), 9));
	data.categories.append(std::pair<QString, int>(QString("Faces"), 10));
	data.categories.append(std::pair<QString, int>(QString("Edges"), 7));
	data.categories.append(std::pair<QString, int>(QString("Angles"), 2));


	//titles
	data.titles.append(QString("#Vertices"));
	data.titles.append(QString("#Connected Components"));
	data.titles.append(QString("#Border Edges"));
	data.titles.append(QString("Pure Triangle"));
	data.titles.append(QString("Pure Quad"));
	data.titles.append(QString("#Degenerate Faces"));
	data.titles.append(QString("Connected Components of the Boundary"));
	data.titles.append(QString("Area"));
	data.titles.append(QString("Volume"));
	data.titles.append(QString("Self-Intersecting"));
	data.titles.append(QString("#Faces"));
	data.titles.append(QString("Min Area"));
	data.titles.append(QString("Max Area"));
	data.titles.append(QString("Median Area"));
	data.titles.append(QString("Mean Area"));
	data.titles.append(QString("Min Altitude"));
	data.titles.append(QString("Min Aspect-Ratio"));
	data.titles.append(QString("Max Aspect-Ratio"));
	data.titles.append(QString("Mean Aspect-Ratio"));
	data.titles.append(QString("Genus"));
	data.titles.append(QString("#Edges"));
	data.titles.append(QString("Minimum Length"));
	data.titles.append(QString("Maximum Length"));
	data.titles.append(QString("Median Length"));
	data.titles.append(QString("Mean Length"));
	data.titles.append(QString("#Degenerate Edges"));
	data.titles.append(QString("Minimum"));
	data.titles.append(QString("Maximum"));
	data.titles.append(QString("Average"));
	return data;
}

void Scene_surface_mesh_item::zoomToPosition(const QPoint& point, CGAL::Three::Viewer_interface* viewer) const
{
	typedef Input_facets_AABB_tree Tree;
	typedef Tree::Intersection_and_primitive_id<Kernel::Ray_3>::Type Intersection_and_primitive_id;

	Tree* aabb_tree = static_cast<Input_facets_AABB_tree*>(d->get_aabb_tree());
	if (aabb_tree) {

		const CGAL::qglviewer::Vec offset = static_cast<CGAL::Three::Viewer_interface*>(CGAL::QGLViewer::QGLViewerPool().first())->offset();
		//find clicked facet
		bool found = false;
		CGAL::qglviewer::Vec point_under = viewer->camera()->pointUnderPixel(point, found, viewer->devicePixelRatio());
		Kernel::Point_3 ray_origin;
		CGAL::qglviewer::Vec dir;
		if (viewer->camera()->type() == CGAL::qglviewer::Camera::PERSPECTIVE)
		{
			ray_origin = Kernel::Point_3(viewer->camera()->position().x - offset.x,
				viewer->camera()->position().y - offset.y,
				viewer->camera()->position().z - offset.z);
			dir = point_under - viewer->camera()->position();
		}
		else
		{
			dir = viewer->camera()->viewDirection();
			ray_origin = Kernel::Point_3(point_under.x - dir.x,
				point_under.y - dir.y,
				point_under.z - dir.z);
		}
		const Kernel::Vector_3 ray_dir(dir.x, dir.y, dir.z);
		const Kernel::Ray_3 ray(ray_origin, ray_dir);
		typedef std::list<Intersection_and_primitive_id> Intersections;
		Intersections intersections;
		aabb_tree->all_intersections(ray, std::back_inserter(intersections));

		if (!intersections.empty()) {
			Intersections::iterator closest = intersections.begin();
			const Kernel::Point_3* closest_point =
				boost::get<Kernel::Point_3>(&closest->first);
			for (Intersections::iterator
				it = boost::next(intersections.begin()),
				end = intersections.end();
				it != end; ++it)
			{
				if (!closest_point) {
					closest = it;
				}
				else {
					const Kernel::Point_3* it_point =
						boost::get<Kernel::Point_3>(&it->first);
					if (it_point &&
						(ray_dir * (*it_point - *closest_point)) < 0)
					{
						closest = it;
						closest_point = it_point;
					}
				}
			}
			if (closest_point) {
				SMesh::Property_map<vertex_descriptor, SMesh::Point> positions =
					d->smesh_->points();
				face_descriptor selected_fh = closest->second;
				//compute new position and orientation
				Kernel::Vector_3 face_normal = CGAL::Polygon_mesh_processing::
					compute_face_normal(selected_fh,
						*d->smesh_,
						CGAL::Polygon_mesh_processing::parameters::all_default());


				double x(0), y(0), z(0),
					xmin(std::numeric_limits<double>::infinity()), ymin(std::numeric_limits<double>::infinity()), zmin(std::numeric_limits<double>::infinity()),
					xmax(-std::numeric_limits<double>::infinity()), ymax(-std::numeric_limits<double>::infinity()), zmax(-std::numeric_limits<double>::infinity());
				int total(0);
				BOOST_FOREACH(vertex_descriptor vh, vertices_around_face(halfedge(selected_fh, *d->smesh_), *d->smesh_))
				{
					x += positions[vh].x();
					y += positions[vh].y();
					z += positions[vh].z();

					if (positions[vh].x() < xmin)
						xmin = positions[vh].x();
					if (positions[vh].y() < ymin)
						ymin = positions[vh].y();
					if (positions[vh].z() < zmin)
						zmin = positions[vh].z();

					if (positions[vh].x() > xmax)
						xmax = positions[vh].x();
					if (positions[vh].y() > ymax)
						ymax = positions[vh].y();
					if (positions[vh].z() > zmax)
						zmax = positions[vh].z();

					++total;
				}
				Kernel::Point_3 centroid(x / total + offset.x,
					y / total + offset.y,
					z / total + offset.z);

				CGAL::qglviewer::Quaternion new_orientation(CGAL::qglviewer::Vec(0, 0, -1),
					CGAL::qglviewer::Vec(-face_normal.x(), -face_normal.y(), -face_normal.z()));
				double max_side = (std::max)((std::max)(xmax - xmin, ymax - ymin),
					zmax - zmin);
				//put the camera in way we are sure the longest side is entirely visible on the screen
				//See openGL's frustum definition
				double factor = CGAL::abs(max_side / (tan(viewer->camera()->aspectRatio() /
					(viewer->camera()->fieldOfView() / 2))));

				Kernel::Point_3 new_pos = centroid + factor * face_normal;
				viewer->camera()->setSceneCenter(CGAL::qglviewer::Vec(centroid.x(),
					centroid.y(),
					centroid.z()));
				viewer->moveCameraToCoordinates(QString("%1 %2 %3 %4 %5 %6 %7").arg(new_pos.x())
					.arg(new_pos.y())
					.arg(new_pos.z())
					.arg(new_orientation[0])
					.arg(new_orientation[1])
					.arg(new_orientation[2])
					.arg(new_orientation[3]));

			}
		}
	}
}

void Scene_surface_mesh_item::resetColors()
{
	setItemIsMulticolor(false);
	if (d->has_feature_edges) {
		BOOST_FOREACH(boost::graph_traits<SMesh>::edge_descriptor e, edges(*d->smesh_)) {
			put(d->e_is_feature_map, e, false);
		}
		d->has_feature_edges = false;
	}
	invalidate(COLORS);
	itemChanged();
}

QMenu* Scene_surface_mesh_item::contextMenu()
{
	QMenu* menu = Scene_item::contextMenu();

	const char* prop_name = "Menu modified by Scene_surface_mesh_item.";
	bool menuChanged = menu->property(prop_name).toBool();

	if (!menuChanged) {
		QMenu* container = new QMenu(tr("Alpha value"));
		container->menuAction()->setProperty("is_groupable", true);
		QWidgetAction* sliderAction = new QWidgetAction(0);
		sliderAction->setDefaultWidget(d->alphaSlider);
		connect(d->alphaSlider, &QSlider::valueChanged,
			[this]() {redraw(); });
		container->addAction(sliderAction);
		menu->addMenu(container);


		menu->addSeparator();

		QAction* actionDisplayTriangleVertices =
			menu->addAction(tr("Display Triangle Vertices"));

		actionDisplayTriangleVertices->setCheckable(true);
		actionDisplayTriangleVertices->setChecked(pointShow);
		actionDisplayTriangleVertices->setObjectName("actionDisplayTriangleVertices");
		connect(actionDisplayTriangleVertices, SIGNAL(triggered(bool)),
			this, SLOT(showFacetVertices(bool)));

		actionDisplayTriangleVertices->setEnabled(false);
		actionDisplayTriangleVertices->setVisible(false);

		//
		QAction* actionDisplayTriangleEdges =
			menu->addAction(tr("Display Triangle Edges"));

		actionDisplayTriangleEdges->setCheckable(true);
		actionDisplayTriangleEdges->setChecked(edgesShow);
		actionDisplayTriangleEdges->setObjectName("actionDisplayTriangleEdges");
		connect(actionDisplayTriangleEdges, SIGNAL(triggered(bool)),
			this, SLOT(showFacetEdges(bool)));
		//
		QAction* actionDisplaySegmentBorder =
			menu->addAction(tr("Display Segment Border"));

		actionDisplaySegmentBorder->setCheckable(true);
		actionDisplaySegmentBorder->setChecked(segmentBoundryShow);
		actionDisplaySegmentBorder->setObjectName("actionDisplaySegmentBorder");
		connect(actionDisplaySegmentBorder, SIGNAL(triggered(bool)),
			this, SLOT(showSegmentBorders(bool)));

		//
		QAction* actionDisplayMeshBoundary =
			menu->addAction(tr("Display Mesh Boundary"));

		actionDisplayMeshBoundary->setCheckable(true);
		actionDisplayMeshBoundary->setChecked(meshBoundaryShow);
		actionDisplayMeshBoundary->setObjectName("actionDisplayMeshBoundary");
		connect(actionDisplayMeshBoundary, SIGNAL(triggered(bool)),
			this, SLOT(showMeshBoundary(bool)));

		menu->addSeparator();

		QAction* actionPrintVertices =
			menu->addAction(tr("Display Vertices Ids"));

		actionPrintVertices->setEnabled(false);
		actionPrintVertices->setVisible(false);

		actionPrintVertices->setCheckable(true);
		actionPrintVertices->setObjectName("actionPrintVertices");
		connect(actionPrintVertices, SIGNAL(triggered(bool)),
			this, SLOT(showVertices(bool)));

		QAction* actionPrintEdges =
			menu->addAction(tr("Display Edges Ids"));

		actionPrintEdges->setEnabled(false);
		actionPrintEdges->setVisible(false);

		actionPrintEdges->setCheckable(true);
		actionPrintEdges->setObjectName("actionPrintEdges");
		connect(actionPrintEdges, SIGNAL(triggered(bool)),
			this, SLOT(showEdges(bool)));

		QAction* actionPrintFaces =
			menu->addAction(tr("Display Faces Ids"));

		actionPrintFaces->setEnabled(false);
		actionPrintFaces->setVisible(false);

		actionPrintFaces->setCheckable(true);
		actionPrintFaces->setObjectName("actionPrintFaces");
		connect(actionPrintFaces, SIGNAL(triggered(bool)),
			this, SLOT(showFaces(bool)));

		QAction* actionZoomToId =
			menu->addAction(tr("Zoom to Index"));
		actionZoomToId->setObjectName("actionZoomToId");
		connect(actionZoomToId, &QAction::triggered,
			this, &Scene_surface_mesh_item::zoomToId);


		setProperty("menu_changed", true);
		menu->setProperty(prop_name, true);
	}

	QAction* action = menu->findChild<QAction*>("actionPrintVertices");
	if (action) action->setChecked(d->vertices_displayed);
	action = menu->findChild<QAction*>("actionPrintEdges");
	if (action) action->setChecked(d->edges_displayed);
	action = menu->findChild<QAction*>("actionPrintFaces");
	if (action) action->setChecked(d->faces_displayed);

	action = menu->findChild<QAction*>("actionDisplayTriangleVertices");
	if (action) action->setChecked(pointShow);
	action = menu->findChild<QAction*>("actionDisplayTriangleEdges");
	if (action) action->setChecked(edgesShow);
	action = menu->findChild<QAction*>("actionDisplaySegmentBorder");
	if (action) action->setChecked(segmentBoundryShow);

	return menu;
}
void Scene_surface_mesh_item::printPrimitiveId(QPoint point, CGAL::Three::Viewer_interface* viewer)
{
	typedef Input_facets_AABB_tree Tree;
	Tree* aabb_tree = static_cast<Input_facets_AABB_tree*>(d->get_aabb_tree());
	if (!aabb_tree)
		return;
	face_descriptor selected_fh;
	Kernel::Point_3 pt_under;
	const CGAL::qglviewer::Vec offset = static_cast<CGAL::Three::Viewer_interface*>(CGAL::QGLViewer::QGLViewerPool().first())->offset();
	if (find_primitive_id(point, aabb_tree, viewer, selected_fh, pt_under))
		d->fillTargetedIds(selected_fh, pt_under, viewer, offset);

}
void Scene_surface_mesh_item_priv::fillTargetedIds(const face_descriptor& selected_fh,
	const Kernel::Point_3& pt_under,
	CGAL::Three::Viewer_interface* viewer,
	const CGAL::qglviewer::Vec& offset)
{
	compute_displayed_ids(*smesh_,
		viewer,
		selected_fh,
		pt_under,
		offset,
		textVItems,
		textEItems,
		textFItems,
		&targeted_id);


	if (vertices_displayed
		&& !textVItems->isEmpty())
		item->showVertices(true);
	if (edges_displayed
		&& !textEItems->isEmpty())
		item->showEdges(true);
	if (faces_displayed
		&& !textFItems->isEmpty())
		item->showFaces(true);

}

bool Scene_surface_mesh_item::printVertexIds(CGAL::Three::Viewer_interface* viewer) const
{
	if (d->vertices_displayed)
	{
		d->all_displayed = true;
		return ::printVertexIds(*d->smesh_,
			d->textVItems,
			viewer);
	}
	return true;
}

bool Scene_surface_mesh_item::printEdgeIds(CGAL::Three::Viewer_interface* viewer) const
{
	if (d->edges_displayed)
	{
		d->all_displayed = true;
		return ::printEdgeIds(*d->smesh_,
			d->textEItems,
			viewer);
	}
	return true;
}

bool Scene_surface_mesh_item::printFaceIds(CGAL::Three::Viewer_interface* viewer) const
{
	if (d->faces_displayed)
	{
		d->all_displayed = true;
		return ::printFaceIds(*d->smesh_,
			d->textFItems,
			viewer);
	}
	return true;
}

void Scene_surface_mesh_item_priv::killIds()
{
	CGAL::Three::Viewer_interface* viewer =
		qobject_cast<CGAL::Three::Viewer_interface*>(CGAL::QGLViewer::QGLViewerPool().first());
	deleteIds(viewer,
		textVItems,
		textEItems,
		textFItems,
		&targeted_id);
	all_displayed = false;
}

void Scene_surface_mesh_item::printAllIds(CGAL::Three::Viewer_interface* viewer)
{
	static bool all_ids_displayed = false;

	all_ids_displayed = !all_ids_displayed;
	if (all_ids_displayed)
	{
		bool s1(printVertexIds(viewer)),
			s2(printEdgeIds(viewer)),
			s3(printFaceIds(viewer));
		if ((s1 && s2 && s3))
		{
			viewer->update();
		}
		return;
	}
	d->killIds();
}

bool Scene_surface_mesh_item::testDisplayId(double x, double y, double z, CGAL::Three::Viewer_interface* viewer)const
{
	const CGAL::qglviewer::Vec offset = static_cast<CGAL::Three::Viewer_interface*>(CGAL::QGLViewer::QGLViewerPool().first())->offset();
	Kernel::Point_3 src(x - offset.x,
		y - offset.y,
		z - offset.z);

	CGAL::qglviewer::Camera* cam = viewer->camera();
	Kernel::Point_3 dest(cam->position().x - offset.x,
		cam->position().y - offset.y,
		cam->position().z - offset.z);
	Kernel::Vector_3 v(src, dest);
	Kernel::Vector_3 dir(cam->viewDirection().x,
		cam->viewDirection().y,
		cam->viewDirection().z);
	if (-CGAL::scalar_product(v, dir) < cam->zNear()) //if src is behind the near plane, don't display.
		return false;
	v = 0.01 * v;
	Kernel::Point_3 point = src;
	point = point + v;
	Kernel::Segment_3 query(point, dest);
	return !static_cast<Input_facets_AABB_tree*>(d->get_aabb_tree())->do_intersect(query);
}

void Scene_surface_mesh_item::update_labels_for_selection()
{
	semantic_facet_map.clear();
	int label_size = 0;
	std::string comments;
	comments = this->comments();
	std::istringstream iss(comments);
	std::string line, word;
	while (getline(iss, line))
	{
		if (line != "Generated by the CGAL library")
		{
			std::istringstream temp_stream(line);
			bool is_texture = false;
			while (temp_stream >> word)
			{
				if (word == "TextureFile")
					is_texture = true;
			}
			if (is_texture)
				continue;
			else
				++label_size;
		}
	}

	for (std::size_t i = 0; i < label_size; ++i)
	{
		semantic_facet_map[i] = std::vector<face_descriptor>();
	}

	if (label_size > 0)
	{
		for (std::map<face_descriptor, int>::iterator p = face_label.begin(); p != face_label.end(); p++)
		{
			if (p->first.is_valid())
			{
				semantic_facet_map[p->second].emplace_back(p->first);
			}
		}
	}
}

void Scene_surface_mesh_item::fill_classes_combo_box(QComboBox* cb)
{
	std::string comments;
	comments = this->comments();
	std::istringstream iss(comments);
	std::string line, word;
	std::vector<std::string> face_label_comment_tmp;
	while (getline(iss, line))
	{
		if (line != "Generated by the CGAL library") // Avoid repeating the line if multiple savings
		{
			std::istringstream temp_stream(line);
			int space_count = 0;
			bool is_label = false, is_texture_name = false;
			while (temp_stream >> word)
			{
				if (word == "label" && is_label == false)
					is_label = true;

				++space_count;
				if (space_count == 3 && is_label == true)
				{
					//Notice the class name must be one word or words connect with underscore "_"
					face_label_comment_tmp.push_back(word);
				}
			}
		}
	}

	cb->clear();
	cb->addItem("None");
	cb->addItem("unlabelled");
	for (std::size_t i = 0; i < face_label_comment_tmp.size(); ++i)
	{
		std::ostringstream oss;
		oss << face_label_comment_tmp[i].c_str();
		cb->addItem(oss.str().c_str());
	}

	if (this->label_selection_combox_tmp == NULL)
		this->label_selection_combox_tmp = cb;
}

void Scene_surface_mesh_item::showMeshBoundary(bool b)
{
	CGAL::Three::Viewer_interface* viewer =
		qobject_cast<CGAL::Three::Viewer_interface*>(CGAL::QGLViewer::QGLViewerPool().first());
	meshBoundaryShow = b;
	viewer->update();
}

void Scene_surface_mesh_item::showSegmentBorders(bool b)
{
	CGAL::Three::Viewer_interface* viewer =
		qobject_cast<CGAL::Three::Viewer_interface*>(CGAL::QGLViewer::QGLViewerPool().first());
	segmentBoundryShow = b;
	viewer->update();
}

void Scene_surface_mesh_item::showFacetEdges(bool b)
{
	CGAL::Three::Viewer_interface* viewer =
		qobject_cast<CGAL::Three::Viewer_interface*>(CGAL::QGLViewer::QGLViewerPool().first());
	edgesShow = b;
	viewer->update();
}

void Scene_surface_mesh_item::showFacetVertices(bool b)
{
	CGAL::Three::Viewer_interface* viewer =
		qobject_cast<CGAL::Three::Viewer_interface*>(CGAL::QGLViewer::QGLViewerPool().first());
	pointShow = b;
	if (pointShow)
		this->drawPoints(viewer);
	viewer->update();
}

void Scene_surface_mesh_item::showVertices(bool b)
{
	CGAL::Three::Viewer_interface* viewer =
		qobject_cast<CGAL::Three::Viewer_interface*>(CGAL::QGLViewer::QGLViewerPool().first());
	TextRenderer* renderer = viewer->textRenderer();
	if (b)
		if (d->textVItems->isEmpty())
		{
			d->vertices_displayed = b;
			printVertexIds(viewer);
		}
		else
			renderer->addTextList(d->textVItems);
	else
		renderer->removeTextList(d->textVItems);
	viewer->update();
	d->vertices_displayed = b;
}

void Scene_surface_mesh_item::showEdges(bool b)
{
	CGAL::Three::Viewer_interface* viewer =
		qobject_cast<CGAL::Three::Viewer_interface*>(CGAL::QGLViewer::QGLViewerPool().first());
	TextRenderer* renderer = viewer->textRenderer();
	if (b)
	{
		if (d->textEItems->isEmpty())
		{
			d->edges_displayed = b;
			printEdgeIds(viewer);
		}
		else
			renderer->addTextList(d->textEItems);
	}
	else
		renderer->removeTextList(d->textEItems);
	viewer->update();
	d->edges_displayed = b;
}

void Scene_surface_mesh_item::showFaces(bool b)
{
	CGAL::Three::Viewer_interface* viewer =
		qobject_cast<CGAL::Three::Viewer_interface*>(CGAL::QGLViewer::QGLViewerPool().first());
	TextRenderer* renderer = viewer->textRenderer();
	if (b)
	{
		if (d->textFItems->isEmpty())
		{
			d->faces_displayed = b;
			printFaceIds(viewer);
		}
		else
			renderer->addTextList(d->textFItems);
	}
	else
		renderer->removeTextList(d->textFItems);
	viewer->update();
	d->faces_displayed = b;
}

void Scene_surface_mesh_item::showPrimitives(bool)
{
	CGAL::Three::Viewer_interface* viewer =
		qobject_cast<CGAL::Three::Viewer_interface*>(CGAL::QGLViewer::QGLViewerPool().first());
	printAllIds(viewer);
}

void Scene_surface_mesh_item::zoomToId()
{
	face_descriptor selected_fh;
	bool ok;
	QString text = QInputDialog::getText(QApplication::activeWindow(), tr("Zoom to Index"),
		tr("Simplex"), QLineEdit::Normal,
		tr("v0"), &ok);
	if (!ok)
		return;

	CGAL::Three::Viewer_interface* viewer =
		qobject_cast<CGAL::Three::Viewer_interface*>(CGAL::QGLViewer::QGLViewerPool().first());
	Point_3 p;
	QString id = text.right(text.length() - 1);
	int return_value = ::zoomToId(*d->smesh_, text, viewer, selected_fh, p);
	switch (return_value)
	{
	case 1:
		QMessageBox::warning(QApplication::activeWindow(),
			"ERROR",
			tr("Input must be of the form [v/e/f][int]")
		);
		return;
	case 2:
		QMessageBox::warning(QApplication::activeWindow(),
			"ERROR",
			tr("No vertex with id %1").arg(id)
		);
		return;
	case 3:
		QMessageBox::warning(QApplication::activeWindow(),
			"ERROR",
			tr("No edge with id %1").arg(id)
		);
		return;
	case 4:
		QMessageBox::warning(QApplication::activeWindow(),
			"ERROR",
			tr("No face with id %1").arg(id)
		);
		return;
	default: //case 0
		d->fillTargetedIds(selected_fh, p, viewer, viewer->offset());
		break;
	}
}

bool Scene_surface_mesh_item::shouldDisplayIds(CGAL::Three::Scene_item* current_item) const
{
	return this == current_item;
}

float Scene_surface_mesh_item::alpha() const
{
	if (!d->alphaSlider)
		return 1.0f;
	return (float)d->alphaSlider->value() / 255.0f;
}

void Scene_surface_mesh_item::setAlpha(int alpha)
{
	if (!d->alphaSlider)
		d->compute_elements(Scene_item_rendering_helper::ALL);
	d->alphaSlider->setValue(alpha);
	redraw();
}

QSlider* Scene_surface_mesh_item::alphaSlider() { return d->alphaSlider; }

void Scene_surface_mesh_item::computeElements()const
{
	d->compute_elements(ALL);
	setBuffersFilled(true);
}

void Scene_surface_mesh_item::copyProperties(Scene_item* item)
{
	Scene_surface_mesh_item* sm_item = qobject_cast<Scene_surface_mesh_item*>(item);
	if (!sm_item)
		return;
	int value = sm_item->alphaSlider()->value();
	alphaSlider()->setValue(value);
}

void Scene_surface_mesh_item::computeItemColorVectorAutomatically(bool b)
{
	this->setProperty("recompute_colors", b);
}