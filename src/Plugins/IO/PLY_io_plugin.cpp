#include "Scene_polygon_soup_item.h"
#include "Scene_surface_mesh_item.h"

#include <CGAL/Three/Polyhedron_demo_io_plugin_interface.h>
#include <CGAL/Three/Three.h>
#include <QInputDialog>
#include <QApplication>
#include <fstream>

#include <CGAL/IO/PLY_reader.h>
#include <CGAL/IO/PLY_writer.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <QMessageBox>

class Polyhedron_demo_ply_plugin :
	public QObject,
	public CGAL::Three::Polyhedron_demo_io_plugin_interface
{
	Q_OBJECT
		Q_INTERFACES(CGAL::Three::Polyhedron_demo_io_plugin_interface)
		Q_PLUGIN_METADATA(IID "com.geometryfactory.PolyhedronDemo.IOPluginInterface/1.0" FILE "ply_io_plugin.json")

public:
	bool isDefaultLoader(const CGAL::Three::Scene_item* item) const
	{
		return false;
	}
	QString name() const { return "ply_plugin"; }
	QString nameFilters() const { return "PLY files (*.ply)"; }
	bool canLoad() const;
	CGAL::Three::Scene_item* load(QFileInfo fileinfo);

	bool canSave(const CGAL::Three::Scene_item*);
	bool save(const CGAL::Three::Scene_item*, QFileInfo fileinfo);

private:
	/*void set_vcolors(SMesh* smesh, const std::vector<CGAL::Color>& colors)
	{
		typedef SMesh SMesh;
		typedef boost::graph_traits<SMesh>::vertex_descriptor vertex_descriptor;
		SMesh::Property_map<vertex_descriptor, CGAL::Color> vcolors =
			smesh->property_map<vertex_descriptor, CGAL::Color >("v:color").first;
		bool created;
		boost::tie(vcolors, created) = smesh->add_property_map<SMesh::Vertex_index, CGAL::Color>("v:color", CGAL::Color(0, 0, 0));
		assert(colors.size() == smesh->number_of_vertices());
		int color_id = 0;
		BOOST_FOREACH(vertex_descriptor vd, vertices(*smesh))
			vcolors[vd] = colors[color_id++];
	}

	void set_fcolors(SMesh* smesh, const std::vector<CGAL::Color>& colors)
	{
		typedef SMesh SMesh;
		typedef boost::graph_traits<SMesh>::face_descriptor face_descriptor;
		SMesh::Property_map<face_descriptor, CGAL::Color> fcolors =
			smesh->property_map<face_descriptor, CGAL::Color >("f:color").first;
		bool created;
		boost::tie(fcolors, created) = smesh->add_property_map<SMesh::Face_index, CGAL::Color>("f:color", CGAL::Color(0, 0, 0));
		assert(colors.size() == smesh->number_of_faces());
		int color_id = 0;
		BOOST_FOREACH(face_descriptor fd, faces(*smesh))
			fcolors[fd] = colors[color_id++];
	}*/

	void set_vcolors(SMesh* smesh, const std::vector<CGAL::Color>& colors, SMesh::Property_map<vertex_descriptor, CGAL::Color> &vcolors)
	{
		typedef SMesh SMesh;
		typedef boost::graph_traits<SMesh>::vertex_descriptor vertex_descriptor;
		vcolors = smesh->property_map<vertex_descriptor, CGAL::Color >("v:color").first;
		bool created;
		boost::tie(vcolors, created) = smesh->add_property_map<SMesh::Vertex_index, CGAL::Color>("v:color", CGAL::Color(0, 0, 0));
		assert(colors.size() == smesh->number_of_vertices());
	}

	void set_fcolors(SMesh* smesh, const std::vector<CGAL::Color>& colors, SMesh::Property_map<face_descriptor, CGAL::Color> &fcolors)
	{
		typedef SMesh SMesh;
		typedef boost::graph_traits<SMesh>::face_descriptor face_descriptor;
		fcolors = smesh->property_map<face_descriptor, CGAL::Color >("f:color").first;
		bool created;
		boost::tie(fcolors, created) = smesh->add_property_map<SMesh::Face_index, CGAL::Color>("f:color", CGAL::Color(0, 0, 0));
		assert(colors.size() == smesh->number_of_faces());
	}
	//**************************************************//
};

bool Polyhedron_demo_ply_plugin::canLoad() const {
	return true;
}

CGAL::Three::Scene_item*
Polyhedron_demo_ply_plugin::load(QFileInfo fileinfo) {
	std::ifstream in(fileinfo.filePath().toUtf8(), std::ios_base::binary);

	//std::cout << "file path = " << fileinfo.absolutePath().toStdString() << std::endl;

	if (!in)
		std::cerr << "Error!\n";

	QApplication::setOverrideCursor(Qt::WaitCursor);

	if (fileinfo.size() == 0)
	{
		CGAL::Three::Three::warning(tr("The file you are trying to load is empty."));
		return 0;
	}

	// Test if input is mesh or point set
	bool input_is_mesh = false;
	std::string line;
	std::istringstream iss;

	// test whether input is mesh
	while (getline(in, line))
	{
		iss.clear();
		iss.str(line);
		std::string keyword;
		if (iss >> keyword)
		{
			if (keyword == "element")
			{
				std::string type;
				int nb;
				if (iss >> type >> nb)
					if (type == "face" && nb > 0)
					{
						input_is_mesh = true;
						break;
					}
			}
			else if (keyword == "end_header")
				break;
		}
	}

	in.seekg(0);

	if (input_is_mesh) // Open mesh or polygon soup
	{
		std::vector<Kernel::Point_3> points;
		std::vector<std::vector<std::size_t> > polygons;
		std::vector<CGAL::Color> fcolors;
		std::vector<CGAL::Color> vcolors;

		//if (!(CGAL::read_PLY (in, points, polygons, fcolors, vcolors)))
		//{
		//  QApplication::restoreOverrideCursor();
		//  return NULL;
		//}

		//if (binary)
		//	CGAL::set_binary_mode(in);

		std::vector< int > flabels;
		std::vector<std::string> face_label_comment;
		std::vector<std::vector<float>> fi_texcoord;
		std::vector<int> texture_id;
		std::vector<std::string> texture_name;
		std::vector<float> fi_prob;
		std::vector<int> fi_segment_id;
		std::string input_comments;
		int total_labeled_faces_i = 0;
		if (!(CGAL::read_PLY(in, points, polygons, fcolors, vcolors, flabels, face_label_comment, fi_texcoord, texture_id, texture_name, fi_prob, fi_segment_id, input_comments, total_labeled_faces_i)))
		{
			QApplication::restoreOverrideCursor();
			return NULL;
		}
		//*******************************************************************//
		if (CGAL::Polygon_mesh_processing::is_polygon_soup_a_polygon_mesh(polygons))
		{
			SMesh* surface_mesh = new SMesh();
			CGAL::Polygon_mesh_processing::polygon_soup_to_polygon_mesh(points, polygons,
				*surface_mesh);

			int v_color_id = 0;
			SMesh::Property_map<vertex_descriptor, CGAL::Color> vcolors_;
			if (!(vcolors.empty()))
			{
				set_vcolors(surface_mesh, vcolors, vcolors_);
			}

			int f_color_id = 0;
			SMesh::Property_map<face_descriptor, CGAL::Color> fcolors_;
			if (!(fcolors.empty()))
			{
				set_fcolors(surface_mesh, fcolors, fcolors_);
			}

			Scene_surface_mesh_item* sm_item = new Scene_surface_mesh_item(surface_mesh);
			sm_item->setName(fileinfo.completeBaseName());
		
			int v_ind = 0, duplicate_count = 0;
			bool is_duplicate_points = false;
			std::map<Kernel::Point_3, int> duplicate_map;
			BOOST_FOREACH(vertex_descriptor vd, vertices(*(sm_item->polyhedron())))
			{
				if (vcolors.empty() == false)
				{
					sm_item->vertex_color[vd] = QColor(vcolors[v_ind].red(), vcolors[v_ind].green(), vcolors[v_ind].blue());
					vcolors_[vd] = vcolors[v_color_id++];
				}

				sm_item->vertices_coords[v_ind] = points[v_ind];

				//check duplicate vertices for test if is merged mesh
				Kernel::Point_3 pd = points[v_ind];
				auto it = duplicate_map.find(pd);
				if (it == duplicate_map.end())
				{
					duplicate_map[pd] = v_ind;
				}
				else
				{
					++duplicate_count;
					is_duplicate_points = true;
				}

				++v_ind;
			}
			//test if it is merged mes\h
			if (is_duplicate_points)
			{
				float perc = float(duplicate_count) / float((*(sm_item->polyhedron())).num_vertices());
				if (perc > 0.05f)//rough guess. if duplicate vertices > 5% of total vertices 
				{
					sm_item->is_merged_batch = true;
					CGAL::Three::Three::warning("The input data is a merged mesh and it contains " + QString::number(duplicate_count) + " duplicate vertices.");
				}
			}
			
			sm_item->file_path = fileinfo.absolutePath().toStdString();
			sm_item->texture_name = texture_name;
			//read texture image 
			std::vector<std::string> texture_name_temp = texture_name;
			if (texture_name.empty())
			{
				qWarning("Could not read texture files!");
				CGAL::Three::Three::warning("Could not read texture files!");
				QImage tex, buf;
				QImage dummy(128, 128, QImage::Format_RGB32);
				dummy.fill(Qt::green);
				buf = dummy;
				tex = QGLWidget::convertToGLFormat(buf);
				sm_item->texture_images.push_back(tex);

				fi_texcoord = std::vector<std::vector<float>>(polygons.size(), std::vector<float>(6, 0.0f));
			}
			if (texture_id.empty())
			{
				texture_id.clear();
				texture_id = std::vector<int>(polygons.size(), 0);
			}

			for (size_t t = 0; t < texture_name.size(); ++t)
			{
				texture_name_temp[t] = sm_item->file_path + "/" + texture_name_temp[t];
				
				QImage tex, buf;
				if (!buf.load(texture_name_temp[t].c_str()))//
				{
					qWarning("Could not read image file!");
					CGAL::Three::Three::warning("Could not read the input texture file!");
					QImage dummy(128, 128, QImage::Format_RGB32);
					dummy.fill(Qt::green);
					buf = dummy;
				}
				tex = QGLWidget::convertToGLFormat(buf);
				sm_item->texture_images.push_back(tex);
			}

			//read facet properties
			sm_item->face_center_point_set.add_normal_map();//add normal 
			int f_ind = 0, segment_size = -1;
			std::map<face_descriptor, bool> face_visited_check;
			std::map<int, std::set<int>> textureID_vertices_grouping;
			std::map<int, std::vector<face_vind_texcoord>> textureID_facets_grouping;
			std::set<std::pair<face_descriptor, float>, FaceAreaComp> sorted_ascending_order;
			BOOST_FOREACH(face_descriptor fd, faces(*(sm_item->polyhedron())))
			{
				if (flabels.empty() == false)
					sm_item->face_label[fd] = flabels[f_ind];

				if (fcolors.empty() == false)
				{
					sm_item->face_color[fd] = QColor(fcolors[f_ind].red(), fcolors[f_ind].green(), fcolors[f_ind].blue());
					fcolors_[fd] = fcolors[f_color_id++];
				}

				//if (fi_texcoord.empty() == false)
					sm_item->face_texcoord[fd] = fi_texcoord[f_ind];

				sm_item->face_textureid[fd] = texture_id[f_ind];
				auto it_tex = textureID_facets_grouping.find(texture_id[f_ind]);
				if (it_tex == textureID_facets_grouping.end())
				{
					textureID_facets_grouping[texture_id[f_ind]] = std::vector<face_vind_texcoord>();
					textureID_vertices_grouping[texture_id[f_ind]] = std::set<int>();
				}
				textureID_facets_grouping[texture_id[f_ind]].emplace_back(std::make_tuple(fd, polygons[f_ind],fi_texcoord[f_ind]));
				textureID_vertices_grouping[texture_id[f_ind]].insert(polygons[f_ind][0]);
				textureID_vertices_grouping[texture_id[f_ind]].insert(polygons[f_ind][1]);
				textureID_vertices_grouping[texture_id[f_ind]].insert(polygons[f_ind][2]);

				sm_item->face_vinds[fd] = std::vector<int>();
				sm_item->face_vinds[fd].push_back(polygons[f_ind][0]);
				sm_item->face_vinds[fd].push_back(polygons[f_ind][1]);
				sm_item->face_vinds[fd].push_back(polygons[f_ind][2]);

				if (fi_prob.empty() == false) 
				{
					sm_item->label_probabilities[fd] = fi_prob[f_ind];
				}

				if (fi_segment_id.empty() == false)
					sm_item->face_segment_id[fd] = fi_segment_id[f_ind];
				else
					sm_item->face_segment_id[fd] = 0; //f_ind; merge to one segment
				segment_size = segment_size < sm_item->face_segment_id[fd] ? sm_item->face_segment_id[fd] : segment_size;

				//get face normal
				sm_item->face_normals[fd] = surface_mesh->property_map<face_descriptor, Kernel::Vector_3 >("f:normal").first[fd];
				
				face_visited_check[fd] = false;
				sm_item->face_shown[fd] = true;

				sm_item->id_face[f_ind] = fd;

				//add face center
				double fvx = (points[polygons[f_ind][0]].x() +points[polygons[f_ind][1]].x() +points[polygons[f_ind][2]].x()) / 3.0f;
				double fvy = (points[polygons[f_ind][0]].y() +points[polygons[f_ind][1]].y() +points[polygons[f_ind][2]].y()) / 3.0f;
				double fvz = (points[polygons[f_ind][0]].z() +points[polygons[f_ind][1]].z() +points[polygons[f_ind][2]].z()) / 3.0f;
				Point_3 p_tmp(fvx, fvy, fvz);
				Vector_3 n_tmp(sm_item->face_normals[fd]);
				sm_item->face_center_point_set.insert(p_tmp, n_tmp);

				//add face area
				sm_item->face_area[fd] = CGAL::Polygon_mesh_processing::face_area(fd, *(sm_item->polyhedron()));
				sm_item->seg_area_sorted_percentile[fd] = 0.0f;
				sorted_ascending_order.insert(std::make_pair(fd, sm_item->face_area[fd]));
				++f_ind;
			}
			//update total labeled faces
			sm_item->total_labeled_faces = total_labeled_faces_i;

			//update face area percetile 
			if (fi_segment_id.empty())
			{
				std::set<std::pair<face_descriptor, float>, FaceAreaComp>::iterator it_fdarea = sorted_ascending_order.begin();
				for (int f_area_id = 0; it_fdarea != sorted_ascending_order.end(); it_fdarea++, f_area_id++)
				{
					sm_item->seg_area_sorted_percentile[it_fdarea->first] = float(f_area_id) / float(faces(*(sm_item->polyhedron())).size());
				}
			}

			//update face segment id (check if isolated segments are merged as one)
			//if (fi_segment_id.empty() == false)
			//{
				CGAL::Three::Three::information("Pre-processing of input mesh ...");
				if (!sm_item->is_merged_batch)
					segment_size = sm_item->updateSegmentId(face_visited_check);
				else
					sm_item->findDuplicateVertices(sm_item, points, polygons);

				//grouping facets for texture rendering
				sm_item->grouping_facets_for_multi_texture_rendering(sm_item, points, textureID_vertices_grouping, textureID_facets_grouping);

				CGAL::Three::Three::information("The number of facet is " + QString::number(polygons.size())
				+ ", the number of segment is " + QString::number(segment_size));
			//}
			//else
			//{
			//	CGAL::Three::Three::information("The number of facet is " + QString::number(segment_size));
			//}

			sm_item->computeSegmentBoundary();

			sm_item->face_label_comment = face_label_comment;
			sm_item->input_comments = input_comments;
			sm_item->setRenderingMode(CGAL::Three::Three::defaultSurfaceMeshRenderingMode());
			if (!input_comments.empty())
			{
				sm_item->set_comments(input_comments);
			}

			CGAL::Three::Three::SetdefaultSurfaceMeshRenderingMode(TextureModePlusFlatEdges);
			sm_item->setRenderingMode(TextureModePlusFlatEdges);
			//CGAL::Three::Three::information(QString("Reset the default rendering mode to TextureModePlusFlatEdges"));
			//*******************************************************************//

			return sm_item;
		}
		else
		{
			CGAL::Three::Three::error(QString("We are currently not supporting binary *.ply and non-manifold meshes."));
			Scene_polygon_soup_item* soup_item = new Scene_polygon_soup_item;
			soup_item->setName(fileinfo.completeBaseName());
			soup_item->load(points, polygons, fcolors, vcolors);
			QApplication::restoreOverrideCursor();
			return soup_item;
		}
	}
	QApplication::restoreOverrideCursor();
	return NULL;
}

bool Polyhedron_demo_ply_plugin::canSave(const CGAL::Three::Scene_item* item)
{
	// This plugin supports point sets and any type of surface
	return (qobject_cast<const Scene_polygon_soup_item*>(item)
		|| qobject_cast<const Scene_surface_mesh_item*>(item));
}

bool Polyhedron_demo_ply_plugin::save(const CGAL::Three::Scene_item* item, QFileInfo fileinfo)
{
	// Check extension (quietly)
	std::string extension = fileinfo.suffix().toUtf8().data();
	if (extension != "ply" && extension != "PLY")
		return false;

	//QStringList list;
	//list << tr("Binary");
	//list << tr("Ascii");
	//bool ok = false;
	//QString choice
	//	= QInputDialog::getItem(NULL, tr("Save PLY file"), tr("Format"), list, 0, false, &ok);

	//if (!ok)
	//	return false;

	QString choice = tr("Ascii");

	std::ofstream out(fileinfo.filePath().toUtf8().data(), std::ios::binary);
	out.precision(std::numeric_limits<double>::digits10 + 2);

	// This plugin supports polygon soups
	const Scene_polygon_soup_item* soup_item =
		qobject_cast<const Scene_polygon_soup_item*>(item);
	if (soup_item)
		return CGAL::write_PLY(out, soup_item->points(), soup_item->polygons());

	// This plugin supports surface meshes
	const Scene_surface_mesh_item* sm_item =
		qobject_cast<const Scene_surface_mesh_item*>(item);

	if (sm_item)
	{
		return sm_item->write_ply_mesh(out, (choice == tr("Binary")));
		//return CGAL::write_PLY (out, *(sm_item->polyhedron()));
	}

	return false;
}


#include "PLY_io_plugin.moc"
