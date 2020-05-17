#include "Scene_polygon_soup_item.h"
#include "Scene_surface_mesh_item.h"
#include "Kernel_type.h"

#include <CGAL/Three/Polyhedron_demo_io_plugin_interface.h>
#include <CGAL/Three/Three.h>
#include <QInputDialog>
#include <QApplication>
#include <fstream>

#include <CGAL/IO/PLY_reader.h>
#include <CGAL/IO/PLY_writer.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
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
	void set_vcolors(SMesh* smesh, const std::vector<CGAL::Color>& colors)
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
	}
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

		//***********************Weixiao Update read ply*******************************//
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
		std::string* comments = NULL;
		if (!(CGAL::read_PLY(in, points, polygons, fcolors, vcolors, flabels, face_label_comment, fi_texcoord, texture_id, texture_name, fi_prob, fi_segment_id, input_comments)))
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
			if (!(vcolors.empty()))
				set_vcolors(surface_mesh, vcolors);
			if (!(fcolors.empty()))
				set_fcolors(surface_mesh, fcolors);

			Scene_surface_mesh_item* sm_item = new Scene_surface_mesh_item(surface_mesh);
			sm_item->setName(fileinfo.completeBaseName());

			QApplication::restoreOverrideCursor();

			//***********************Weixiao Update read ply*******************************//
			if (vcolors.empty() == false)
			{
				int ind = 0;
				BOOST_FOREACH(vertex_descriptor vd, vertices(*(sm_item->polyhedron())))
				{
					sm_item->vertex_color[vd] = QColor(vcolors[ind].red(), vcolors[ind].green(), vcolors[ind].blue());
					++ind;
				}
			}

			sm_item->file_path = fileinfo.absolutePath().toStdString();
			sm_item->texture_name = texture_name;

			int ind = 0;
			std::map<face_descriptor, bool> face_visited_check;
			BOOST_FOREACH(face_descriptor fd, faces(*(sm_item->polyhedron())))
			{
				if (flabels.empty() == false)
					sm_item->face_label[fd] = flabels[ind];

				if (fcolors.empty() == false)
					sm_item->face_color[fd] = QColor(fcolors[ind].red(), fcolors[ind].green(), fcolors[ind].blue());

				if (fi_texcoord.empty() == false)
					sm_item->face_texcoord[fd] = fi_texcoord[ind];

				if (texture_id.empty() == false)
					sm_item->face_textureid[fd] = texture_id[ind];

				if (fi_prob.empty() == false) {
					sm_item->label_probabilities[fd] = fi_prob[ind];
				}

				if (fi_segment_id.empty() == false)
					sm_item->face_segment_id[fd] = fi_segment_id[ind];

				face_visited_check[fd] = false;
				/********************************Ziqian****************************/
				sm_item->face_shown[fd] = true;
				/******************************************************************/

				++ind;
			}

			//update face segment id (check if isolated segments are merged as one)
			int segment_size = sm_item->updateSegmentId(face_visited_check);
			CGAL::Three::Three::information("The number of segment is " + QString::number(segment_size));

			sm_item->computeSegmentBoundary();

			sm_item->face_label_comment = face_label_comment;
			sm_item->input_comments = input_comments;
			sm_item->setRenderingMode(CGAL::Three::Three::defaultSurfaceMeshRenderingMode());
			if (!input_comments.empty())
			{
				sm_item->set_comments(input_comments);
			}
			//*******************************************************************//

			return sm_item;
		}
		else
		{
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

	//*****************Weixiao*********************//
	//QStringList list;
	//list << tr("Binary");
	//list << tr("Ascii");
	//bool ok = false;
	//QString choice
	//	= QInputDialog::getItem(NULL, tr("Save PLY file"), tr("Format"), list, 0, false, &ok);

	//if (!ok)
	//	return false;

	QString choice = tr("Ascii");
	//*********************************************//

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
		//***********************Weixiao Update write ply binary*******************************//
		return sm_item->write_ply_mesh(out, (choice == tr("Binary")));
		//*******************************************************************//
			  //return CGAL::write_PLY (out, *(sm_item->polyhedron()));
	}

	return false;
}


#include "PLY_io_plugin.moc"
