#include "Surface_mesh_item_classification.h"
#include "Color_ramp.h"

#include <CGAL/Timer.h>
#include <CGAL/Memory_sizer.h>

#include <CGAL/Three/Viewer_interface.h>

#include <QLineEdit>

#include <set>
#include <stack>
#include <algorithm>

Surface_mesh_item_classification::Surface_mesh_item_classification(Scene_surface_mesh_item* mesh)
	: m_mesh(mesh),
	m_selection(NULL)
{
	if (!m_mesh->face_label.empty() && !m_mesh->face_label_comment.empty())
	{
		m_index_color = 1;
		backup_existing_colors_and_add_new();
		m_training = m_mesh->polyhedron()->add_property_map<face_descriptor, std::size_t>("f:training", std::size_t(-1)).first;
		m_classif = m_mesh->polyhedron()->add_property_map<face_descriptor, std::size_t>("f:label", std::size_t(-1)).first;
		m_label_prob = m_mesh->polyhedron()->add_property_map<face_descriptor, float>("f:label_prob", std::size_t(-1)).first;

		bool is_boat = false;
		for (std::size_t i = 0; i < m_mesh->face_label_comment.size(); ++i)
		{
			m_labels.add((m_mesh->face_label_comment[i]).c_str());
		}

		m_label_colors.resize(m_labels.size());
		int face_count = 0, unlabeled_count = 0;
		BOOST_FOREACH(face_descriptor fd, faces(*(m_mesh->polyhedron())))
		{
			++face_count;
			m_training[fd] = m_mesh->face_label[fd];
			m_classif[fd] = m_mesh->face_label[fd];
			if (m_mesh->face_label[fd] != -1)
			{
				m_label_colors[m_training[fd]] = m_mesh->face_color[fd];
			}
			else
			{
				m_label_colors[m_training[fd] + 1] = QColor(0, 0, 0);
				++unlabeled_count;
			}

			m_color[fd] = CGAL::Color(m_mesh->face_color[fd].red(),
				m_mesh->face_color[fd].green(),
				m_mesh->face_color[fd].blue());

			m_mesh->face_shown[fd] = true;
			if (!m_mesh->label_probabilities.empty())
				m_label_prob[fd] = m_mesh->label_probabilities[fd];
		}

		if (unlabeled_count == face_count)
		{
			for (std::size_t i = 0; i < m_labels.size(); ++i)
			{
				if (i == 0)
					m_label_colors[i] = QColor(0, 0, 0);
				else
					m_label_colors[i] = this->get_new_label_color(m_labels[i]->name());
			}
		}
		else
		{
			for (std::size_t i = 0; i < m_labels.size(); ++i)
			{
				if (i != 0 && m_label_colors[i].red() == 0 && m_label_colors[i].green() == 0 && m_label_colors[i].blue() == 0)
					m_label_colors[i] = this->get_new_label_color(m_labels[i]->name());
			}
		}

		update_comments_of_facet_set_item();
	}
	else
	{
		m_index_color = 1;

		backup_existing_colors_and_add_new();
		m_training = m_mesh->polyhedron()->add_property_map<face_descriptor, std::size_t>("f:training", std::size_t(-1)).first;
		m_classif = m_mesh->polyhedron()->add_property_map<face_descriptor, std::size_t>("f:label", std::size_t(-1)).first;

		m_labels.add("unclassified");
		m_labels.add("ground");
		m_labels.add("building");
		m_labels.add("vegetation");
		m_labels.add("vehicle");
		m_labels.add("water");

		//for (std::size_t i = 0; i < m_labels.size(); ++i)
		//	m_label_colors.push_back(this->get_new_label_color(m_labels[i]->name()));

		for (std::size_t i = 0; i < m_labels.size(); ++i)
		{
			if (i == 0)
				m_label_colors.push_back(QColor(0, 0, 0));
			else
				m_label_colors.push_back(this->get_new_label_color(m_labels[i]->name()));
		}

		BOOST_FOREACH(face_descriptor fd, faces(*(m_mesh->polyhedron())))
		{
			m_mesh->face_label[fd] = -1;
		}
		update_comments_of_facet_set_item();
	}
}


Surface_mesh_item_classification::~Surface_mesh_item_classification()
{
}

void Surface_mesh_item_classification::backup_existing_colors_and_add_new()
{
	bool has_colors = false;
	boost::tie(m_color, has_colors) = m_mesh->polyhedron()->property_map<face_descriptor, CGAL::Color>("f:color");
	if (has_colors)
	{
		m_real_color
			= m_mesh->polyhedron()->add_property_map<face_descriptor, CGAL::Color>("f:real_color").first;
		BOOST_FOREACH(face_descriptor fd, faces(*(m_mesh->polyhedron())))
		{
			m_real_color[fd] = m_color[fd];
			m_color[fd] = CGAL::Color(128, 128, 128);
			m_face_checked[fd] = false;
			m_label_updated[fd] = m_mesh->face_label[fd];
		}
	}
	else
		m_color =
		m_mesh->polyhedron()->add_property_map<face_descriptor, CGAL::Color>("f:color", CGAL::Color(128, 128, 128)).first;
}

void Surface_mesh_item_classification::change_color(int index, float* vmin, float* vmax)
{
	m_index_color = index;
	int index_color = index;
	if (index == 0 && m_real_color == Mesh::Property_map<face_descriptor, CGAL::Color>())
		index_color = -1;

	//static Color_ramp ramp;
	//ramp.build_rainbow();

	if (index_color == -1) // item color
	{
		BOOST_FOREACH(face_descriptor fd, faces(*(m_mesh->polyhedron())))
			m_color[fd] = CGAL::Color(128, 128, 128);
	}
	else if (index_color == 0) // real colors
	{
		BOOST_FOREACH(face_descriptor fd, faces(*(m_mesh->polyhedron())))
			m_color[fd] = m_real_color[fd];
	}
	else if (index_color == 1) //(index_color == 2) // editing color
	{
		//BOOST_FOREACH(face_descriptor fd, faces(*(m_mesh->polyhedron())))
		BOOST_FOREACH(face_descriptor fd, m_selection->selected_facets)
		{
			QColor color(0, 0, 0);
			std::size_t c = m_training[fd];
			std::size_t c2 = m_classif[fd];
			float div = 1;
			if (c != std::size_t(-1) && c < std::size_t(100))// 
			{
				m_mesh->face_label[fd] = c;
				color = m_label_colors[c];

				//if (c != c2)
				   // div = 2;

				m_mesh->face_color[fd] = color;

				m_color[fd] = CGAL::Color(color.red() / div,
					color.green() / div,
					color.blue() / div);
			}
			else
			{
				QColor color_unlabelled(128, 0, 0, 120);
				m_mesh->face_label[fd] = -1;
				m_mesh->face_color[fd] = color_unlabelled;

				m_color[fd] = CGAL::Color(color_unlabelled.red(),
					color_unlabelled.green(),
					color_unlabelled.blue(),
					color_unlabelled.alpha());
			}

			//if (c != std::size_t(-1))
			//  color = m_label_colors[c];
			//float div = 1;
			//if (c != c2)
			//  div = 2;
			//m_color[fd] = CGAL::Color(color.red() / div,
			   // color.green() / div,
			   // color.blue() / div);
		}
		update_comments_of_facet_set_item();
	}
	else
	{
		std::size_t corrected_index = index_color - 3;
		BOOST_FOREACH(face_descriptor fd, faces(*(m_mesh->polyhedron())))
		{
			if (m_training[fd] != corrected_index || m_classif[fd] != corrected_index)
			{
				m_color[fd] = CGAL::Color((unsigned char)(255),
					(unsigned char)(255),
					(unsigned char)(255),
					(unsigned char)(0));
			}
			else
			{
				QColor color(0, 0, 0);
				std::size_t c = m_training[fd];

				if (c != std::size_t(-1) && c < std::size_t(100))//
				{
					color = m_label_colors[c];
					m_mesh->face_color[fd] = color;

					m_color[fd] = CGAL::Color(color.red(), color.green(), color.blue());

					//float v = std::max(0.f, std::min(1.f, m_label_prob[fd]));
					//m_color[fd] = CGAL::Color((unsigned char)(ramp.r(v) * 255),
					   // (unsigned char)(ramp.g(v) * 255),
					   // (unsigned char)(ramp.b(v) * 255));

				}
				else
				{
					QColor color_unlabelled(128, 0, 0, 120);
					m_mesh->face_label[fd] = -1;
					m_mesh->face_color[fd] = color_unlabelled;

					m_color[fd] = CGAL::Color(color_unlabelled.red(),
						color_unlabelled.green(),
						color_unlabelled.blue(),
						color_unlabelled.alpha());
				}
			}
		}
	}
}

void Surface_mesh_item_classification::update_all_label_color(int &label_ind)
{
	BOOST_FOREACH(face_descriptor fd, faces(*(m_mesh->polyhedron())))
	{
		if (label_ind != m_classif[fd] && label_ind != m_training[fd])
			continue;
		QColor color(0, 0, 0);
		std::size_t c = m_training[fd];
		if (c != std::size_t(-1) && c < std::size_t(100))//
		{
			color = m_label_colors[c];
			m_mesh->face_color[fd] = color;

			m_color[fd] = CGAL::Color(color.red(), color.green(), color.blue());
		}
		else
		{
			QColor color_unlabelled(128, 0, 0, 120);
			m_mesh->face_label[fd] = -1;
			m_mesh->face_color[fd] = color_unlabelled;

			m_color[fd] = CGAL::Color(color_unlabelled.red(),
				color_unlabelled.green(),
				color_unlabelled.blue(),
				color_unlabelled.alpha());
		}
	}
}

void Surface_mesh_item_classification::threshold_based_change_color(int index, int threshold = 100, bool below = true, float* vmin, float* vmax)
{
	m_index_color = index;
	int index_color = index;
	if (index == 0 && m_real_color == Mesh::Property_map<face_descriptor, CGAL::Color>())
		index_color = -1;

	static Color_ramp ramp;
	ramp.build_rainbow();

	if (index_color == -1) // item color
	{
		BOOST_FOREACH(face_descriptor fd, faces(*(m_mesh->polyhedron())))
			m_color[fd] = CGAL::Color(128, 128, 128);// dont know what this is used for.
		//std::cout << "inside of this DK thing." << std::endl;
	}
	else if (index_color == 0) // real colors
	{
		BOOST_FOREACH(face_descriptor fd, faces(*(m_mesh->polyhedron()))) {
			m_color[fd] = m_real_color[fd];
			m_mesh->face_shown[fd] = true;
		}
	}
	else if (index_color == 1) //(index_color == 2) // editing color
	{
		BOOST_FOREACH(face_descriptor fd, faces(*(m_mesh->polyhedron())))
		{
			QColor color(0, 0, 0);
			std::size_t c = m_training[fd];
			std::size_t c2 = m_classif[fd];

			float div = 1;

			float prob_of_face;
			if (!m_mesh->label_probabilities.empty())
			{
				prob_of_face = m_mesh->label_probabilities[fd];
				prob_of_face *= 100;
			}

			if (m_mesh->label_probabilities.empty() ||
				(below && prob_of_face <= threshold) ||
				(!below && prob_of_face >= threshold))
			{
				if (c != std::size_t(-1) && c < std::size_t(100))//c != std::size_t(-1) && 
				{
					m_mesh->face_label[fd] = c;
					color = m_label_colors[c];

					m_mesh->face_color[fd] = color;

					m_color[fd] = CGAL::Color(color.red() / div,
						color.green() / div,
						color.blue() / div);
				}
				else
				{
					QColor color_unlabelled(128, 0, 0, 120);
					m_mesh->face_label[fd] = -1;
					m_mesh->face_color[fd] = color_unlabelled;

					m_color[fd] = CGAL::Color(color_unlabelled.red(),
						color_unlabelled.green(),
						color_unlabelled.blue(),
						color_unlabelled.alpha());

				}
				m_mesh->face_shown[fd] = true;
			}
			else
			{
				m_color[fd] = CGAL::Color((unsigned char)(255), (unsigned char)(255), (unsigned char)(255), (unsigned char)(0));
				m_mesh->face_shown[fd] = false;
			}

			//if (c != std::size_t(-1))
			//  color = m_label_colors[c];
			//float div = 1;
			//if (c != c2)
			//  div = 2;
			//m_color[fd] = CGAL::Color(color.red() / div,
			   // color.green() / div,
			   // color.blue() / div);
		}

		update_comments_of_facet_set_item();
	}
	else
	{
		std::size_t corrected_index = index_color - 3;
		BOOST_FOREACH(face_descriptor fd, faces(*(m_mesh->polyhedron())))
		{
			if (m_training[fd] != corrected_index || m_classif[fd] != corrected_index)
			{
				m_color[fd] = CGAL::Color((unsigned char)(255),
					(unsigned char)(255),
					(unsigned char)(255),
					(unsigned char)(0));
				m_mesh->face_shown[fd] = false;
			}
			else
			{
				QColor color(0, 0, 0);
				std::size_t c = m_training[fd];

				float prob_of_face;
				if (!m_mesh->label_probabilities.empty())
				{
					prob_of_face = m_mesh->label_probabilities[fd];
					prob_of_face *= 100;
				}

				if (m_mesh->label_probabilities.empty() ||
					(below && prob_of_face <= threshold) ||
					(!below && prob_of_face >= threshold))
				{
					//show them!
					if (c != std::size_t(-1) && c < std::size_t(100))//c != std::size_t(-1) && 
					{
						color = m_label_colors[c];
						m_mesh->face_color[fd] = color;
						m_color[fd] = CGAL::Color(color.red(), color.green(), color.blue());
					}
					else
					{
						QColor color_unlabelled(128, 0, 0, 120);
						m_mesh->face_label[fd] = -1;
						m_mesh->face_color[fd] = color_unlabelled;

						m_color[fd] = CGAL::Color(color_unlabelled.red(),
							color_unlabelled.green(),
							color_unlabelled.blue(),
							color_unlabelled.alpha());
					}
					m_mesh->face_shown[fd] = true;
				}
				else
				{
					m_color[fd] = CGAL::Color((unsigned char)(255), (unsigned char)(255), (unsigned char)(255), (unsigned char)(0));
					m_mesh->face_shown[fd] = false;
				}
			}
		}
	}
}

bool Surface_mesh_item_classification::can_show_probability()
{
	if (m_mesh->label_probabilities.empty())
	{
		return false;
	}
	BOOST_FOREACH(face_descriptor fd, faces(*(m_mesh->polyhedron())))
	{
		if (m_mesh->label_probabilities[fd] > 1 || m_mesh->label_probabilities[fd] < 0)
		{
			return false;
		}
	}
	return true;
}

int Surface_mesh_item_classification::get_total_number_facets()
{
	return m_mesh->polyhedron()->faces().size();
};

int Surface_mesh_item_classification::get_unlabelled_number_facets()
{
	int unlabelled_num = 0;
	if (unlabelled_num_faces_global == 0)
	{
		BOOST_FOREACH(face_descriptor fd, faces(*(m_mesh->polyhedron())))
		{
			if (m_mesh->face_label[fd] == -1 /*|| m_mesh->face_label[fd] == 0*/) ++unlabelled_num;
		}
		unlabelled_num_faces_global = unlabelled_num;
	}
	else
	{
		unlabelled_num = unlabelled_num_faces_global;
	}
	return unlabelled_num;
};

int Surface_mesh_item_classification::get_total_labeled_facets()
{
	return m_mesh->total_labeled_faces;
};

int Surface_mesh_item_classification::get_total_error_facets()
{
	return m_mesh->total_error_facets;
};