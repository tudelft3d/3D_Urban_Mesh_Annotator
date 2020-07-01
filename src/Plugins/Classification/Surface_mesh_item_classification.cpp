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
	//***********************Weixiao update initialize label and color*******************************//
	if (!m_mesh->face_label.empty() && !m_mesh->face_label_comment.empty())
	{
		m_index_color = 1;
		backup_existing_colors_and_add_new();
		m_training = m_mesh->polyhedron()->add_property_map<face_descriptor, std::size_t>("f:training", std::size_t(-1)).first;
		m_classif = m_mesh->polyhedron()->add_property_map<face_descriptor, std::size_t>("f:label", std::size_t(-1)).first;
		m_label_prob = m_mesh->polyhedron()->add_property_map<face_descriptor, float>("f:label_prob", std::size_t(-1)).first;

		for (std::size_t i = 0; i < m_mesh->face_label_comment.size(); ++i)
		{
			//if (m_mesh->face_label_comment[i] != "unclassified")
			//{
			//	m_labels.add((m_mesh->face_label_comment[i]).c_str());
			//}
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
	//*******************************************************************//
	else
	{
		m_index_color = 1;

		backup_existing_colors_and_add_new();
		m_training = m_mesh->polyhedron()->add_property_map<face_descriptor, std::size_t>("f:training", std::size_t(-1)).first;
		m_classif = m_mesh->polyhedron()->add_property_map<face_descriptor, std::size_t>("f:label", std::size_t(-1)).first;

		//***********************Weixiao update unclassified*******************************//
		m_labels.add("unclassified");
		//*******************************************************************//
		m_labels.add("ground");
		m_labels.add("building");
		m_labels.add("vegetation");
		m_labels.add("vehicle");
		m_labels.add("water");

		//for (std::size_t i = 0; i < m_labels.size(); ++i)
		//	m_label_colors.push_back(this->get_new_label_color(m_labels[i]->name()));

		//***********************Weixiao update default color*******************************//
		for (std::size_t i = 0; i < m_labels.size(); ++i)
		{
			if (i == 0)
				m_label_colors.push_back(QColor(0, 0, 0));
			else
				m_label_colors.push_back(this->get_new_label_color(m_labels[i]->name()));
		}

		//*******************************************************************//

		//***********************Weixiao Update default label*******************************//
		BOOST_FOREACH(face_descriptor fd, faces(*(m_mesh->polyhedron())))
		{
			m_mesh->face_label[fd] = -1;
		}
		update_comments_of_facet_set_item();
		//*******************************************************************//
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

	static Color_ramp ramp;
	ramp.build_rainbow();

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
	// else if (index_color == 1) // classif
	// {
	//   BOOST_FOREACH(face_descriptor fd, faces(*(m_mesh->polyhedron())))
	//   {
	//     QColor color (128, 128, 128);
	//     std::size_t c = m_classif[fd];
	//     
	//     if (c != std::size_t(-1))
	//       color = m_label_colors[c];
	//     m_color[fd] = CGAL::Color(color.red(), color.green(), color.blue());
	   //  //***********************Weixiao Update*******************************//
	   //  if (c < std::size_t(100))//c != std::size_t(-1) && 
	   //  {
	   //	  m_mesh->face_label[fd] = c;
	   //	  m_mesh->face_color[fd] = color;
	   //  }
	   //  else
	   //  {
	   //	  m_mesh->face_label[fd] = -1;
	   //	  m_mesh->face_color[fd] = m_label_colors[0];
	   //  }
	   //  //*******************************************************************//
	//   }
	   ////***********************Weixiao Update*******************************//
	   //update_comments_of_facet_set_item();
	   ////*******************************************************************//
	// }
	else if (index_color == 1) //(index_color == 2) // editing color
	{
		BOOST_FOREACH(face_descriptor fd, faces(*(m_mesh->polyhedron())))
		{
			QColor color(0, 0, 0);
			std::size_t c = m_training[fd];
			std::size_t c2 = m_classif[fd];
			//***********************Weixiao Update view editing*******************************//
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

			//*******************************************************************//
			//if (c != std::size_t(-1))
			//  color = m_label_colors[c];
			//float div = 1;
			//if (c != c2)
			//  div = 2;
			//m_color[fd] = CGAL::Color(color.red() / div,
			   // color.green() / div,
			   // color.blue() / div);
		}
		//***********************Weixiao Update comments*******************************//
		update_comments_of_facet_set_item();
		//*******************************************************************//
	}
	else
	{
		//***********************Weixiao Update view labels*******************************//
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
		//*******************************************************************//
	  //*******************************************************************//
	  //if (corrected_index < m_labels.size()) // Display label probabilities
	  //{
	  //  if (m_label_probabilities.size() <= corrected_index ||
	  //      m_label_probabilities[corrected_index].size() != num_faces(*(m_mesh->polyhedron())))
	  //  {
			////BOOST_FOREACH(face_descriptor fd, faces(*(m_mesh->polyhedron())))
			////{
			   //// m_color[fd] = CGAL::Color((unsigned char)(128),
				  ////  (unsigned char)(128),
				  ////  (unsigned char)(128));
			////}
	  //  }
	  //  else
	  //  {
	  //    BOOST_FOREACH(face_descriptor fd, faces(*(m_mesh->polyhedron())))
	  //    {
	  //      float v = std::max (0.f, std::min(1.f, m_label_probabilities[corrected_index][fd]));
	  //      m_color[fd] = CGAL::Color((unsigned char)(ramp.r(v) * 255),
	  //                                (unsigned char)(ramp.g(v) * 255),
	  //                                (unsigned char)(ramp.b(v) * 255));
	  //    }
	  //  }
	  //}
	  //else
	  //{
	  //  corrected_index -= m_labels.size();
	  //  if (corrected_index >= m_features.size())
	  //  {
	  //    std::cerr << "Error: trying to access feature " << corrected_index << " out of " << m_features.size() << std::endl;
	  //    return;
	  //  }
	  //
	  //  Feature_handle feature = m_features[corrected_index];
	  //  float min = std::numeric_limits<float>::max();
	  //  float max = -std::numeric_limits<float>::max();
	  //  
	  //  if (vmin != NULL && vmax != NULL
	  //      && *vmin != std::numeric_limits<float>::infinity()
	  //      && *vmax != std::numeric_limits<float>::infinity())
	  //  {
	  //    min = *vmin;
	  //    max = *vmax;
	  //  }
	  //  else
	  //  {
	  //    BOOST_FOREACH(face_descriptor fd, faces(*(m_mesh->polyhedron())))
	  //    {
	  //      if (feature->value(fd) > max)
	  //        max = feature->value(fd);
	  //      if (feature->value(fd) < min)
	  //        min = feature->value(fd);
	  //    }
	  //  }
	  //  
	  //  BOOST_FOREACH(face_descriptor fd, faces(*(m_mesh->polyhedron())))
	  //  {
	  //    float v = (feature->value(fd) - min) / (max - min);
	  //    if (v < 0.f) v = 0.f;
	  //    if (v > 1.f) v = 1.f;
	  //    
	  //    m_color[fd] = CGAL::Color((unsigned char)(ramp.r(v) * 255),
	  //                              (unsigned char)(ramp.g(v) * 255),
	  //                              (unsigned char)(ramp.b(v) * 255));
	  //  }
	  //  
	  //  if (vmin != NULL && vmax != NULL)
	  //  {
	  //    *vmin = min;
	  //    *vmax = max;
	  //  }
	  //}
	}
}
//***********************Ziqian*******************************//

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

			//***********************Weixiao & Ziqian Update view editing*******************************//
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

			//*******************************************************************//
			//if (c != std::size_t(-1))
			//  color = m_label_colors[c];
			//float div = 1;
			//if (c != c2)
			//  div = 2;
			//m_color[fd] = CGAL::Color(color.red() / div,
			   // color.green() / div,
			   // color.blue() / div);
		}
		//***********************Weixiao Update comments*******************************//
		update_comments_of_facet_set_item();
		//*******************************************************************//
	}
	else
	{
		//***********************Weixiao Update view labels*******************************//
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
//
//void Surface_mesh_item_classification::threshold_based_change_color_(int index, int threshold = 100, bool below = true, float* vmin, float* vmax) {
//	m_index_color = index;
//	int index_color = index;
//	if (index == 0 && m_real_color == Mesh::Property_map<face_descriptor, CGAL::Color>())
//		index_color = -1;
//
//	//static Color_ramp ramp;
//	//ramp.build_rainbow();
//
//	assert(index_color >= 2);
//	{
//		std::size_t corrected_index = index_color - 2;
//
//		//std::cout << "threshold "<<threshold << std::endl;
//		BOOST_FOREACH(face_descriptor fd, faces(*(m_mesh->polyhedron())))
//		{
//			if ((m_training[fd] != corrected_index
//				|| m_classif[fd] != corrected_index)
//				&& m_training[fd] != -1)
//			{// not chosen classification.
//				m_color[fd] = CGAL::Color((unsigned char)(255), (unsigned char)(255), (unsigned char)(255), (unsigned char)(0));
//			}
//			else
//			{// the chosen class
//				QColor color(0, 0, 0);
//				std::size_t c = m_training[fd];
//
//				float prob_of_face;
//				prob_of_face = m_mesh->label_probabilities[fd];
//				prob_of_face *= 100;
//
//				if ((below && prob_of_face < threshold) || (!below && prob_of_face >= threshold)) {
//					//show them!
//					if (c < std::size_t(100))//c != std::size_t(-1) && 
//					{
//						color = m_label_colors[c];
//						m_mesh->face_color[fd] = color;
//						m_color[fd] = CGAL::Color(color.red(), color.green(), color.blue());
//					}
//					else
//					{
//						m_mesh->face_label[fd] = -1;
//						m_mesh->face_color[fd] = m_label_colors[0];
//						m_color[fd] = CGAL::Color(m_label_colors[0].red(),
//							m_label_colors[0].green(),
//							m_label_colors[0].blue());
//					}
//				}
//				else {
//					m_color[fd] = CGAL::Color((unsigned char)(255), (unsigned char)(255), (unsigned char)(255), (unsigned char)(0));
//				}
//			}
//		}
//	}
//}

/********************************************************************/


//***********************Weixiao*******************************//
int Surface_mesh_item_classification::get_total_number_facets()
{
	return m_mesh->polyhedron()->faces().size();
};

int Surface_mesh_item_classification::get_unlabelled_number_facets()
{
	int unlabelled_num = 0;
	BOOST_FOREACH(face_descriptor fd, faces(*(m_mesh->polyhedron())))
	{
		if (m_mesh->face_label[fd] == -1 /*|| m_mesh->face_label[fd] == 0*/) ++unlabelled_num;
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
//************************************************************//
