#include "Surface_mesh_item_classification.h"
#include "Color_ramp.h"

#include <CGAL/Timer.h>
#include <CGAL/Memory_sizer.h>

#include <CGAL/Three/Viewer_interface.h>

#include <QLineEdit>

#include <set>
#include <stack>
#include <algorithm>
#include <boost/array.hpp>

Surface_mesh_item_classification::Surface_mesh_item_classification(Scene_surface_mesh_item* mesh)
	: m_mesh(mesh),
	m_selection(NULL),
	m_generator(NULL)
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
		int ind = 0;
		BOOST_FOREACH(face_descriptor fd, faces(*(m_mesh->polyhedron())))
		{
			//std::cout << "ind = " << ind << std::endl;
			//++ind;

			//std::cout << "m_mesh->face_label[fd] = " << m_mesh->face_label[fd] << std::endl;
			//std::cout << "m_mesh->face_color[fd] = " << m_mesh->face_color[fd].red() 
			//	<<", " << m_mesh->face_color[fd].green() 
			//	<< ", " << m_mesh->face_color[fd].blue() << std::endl;

			m_training[fd] = m_mesh->face_label[fd];
			m_classif[fd] = m_mesh->face_label[fd];
			if (m_mesh->face_label[fd] != -1)
				m_label_colors[m_training[fd]] = m_mesh->face_color[fd];
			else
				m_label_colors[m_training[fd] + 1] = QColor(0, 0, 0);

			m_color[fd] = CGAL::Color(m_mesh->face_color[fd].red(),
				m_mesh->face_color[fd].green(),
				m_mesh->face_color[fd].blue());

			m_mesh->face_shown[fd] = true;
			if (!m_mesh->label_probabilities.empty())
				m_label_prob[fd] = m_mesh->label_probabilities[fd];
		}

		m_sowf = new Sum_of_weighted_features(m_labels, m_features);
		m_ethz = NULL;
#ifdef CGAL_LINKED_WITH_OPENCV
		m_random_forest = NULL;
#endif
#ifdef CGAL_LINKED_WITH_TENSORFLOW
		m_neural_network = NULL;
#endif
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

		m_sowf = new Sum_of_weighted_features(m_labels, m_features);
		m_ethz = NULL;
#ifdef CGAL_LINKED_WITH_OPENCV
		m_random_forest = NULL;
#endif
#ifdef CGAL_LINKED_WITH_TENSORFLOW
		m_neural_network = NULL;
#endif

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
	if (m_sowf != NULL)
		delete m_sowf;
	if (m_ethz != NULL)
		delete m_ethz;
#ifdef CGAL_LINKED_WITH_OPENCV
	if (m_random_forest != NULL)
		delete m_random_forest;
#endif
#ifdef CGAL_LINKED_WITH_TENSORFLOW
	if (m_neural_network != NULL)
		delete m_neural_network;
#endif
	if (m_generator != NULL)
		delete m_generator;
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
//************************************************************//

void Surface_mesh_item_classification::compute_features(std::size_t nb_scales, float voxel_size)
{
	std::cerr << "Computing features with " << nb_scales << " scale(s) and ";
	if (voxel_size == -1)
		std::cerr << "automatic voxel size" << std::endl;
	else
		std::cerr << "voxel size = " << voxel_size << std::endl;

	m_features.clear();

	if (m_generator != NULL)
		delete m_generator;

	Face_center_map fc_map(m_mesh->polyhedron());

	m_generator = new Generator(*(m_mesh->polyhedron()), fc_map, nb_scales, voxel_size);

#ifdef CGAL_LINKED_WITH_TBB
	m_features.begin_parallel_additions();
#endif

	m_generator->generate_point_based_features(m_features);
	m_generator->generate_face_based_features(m_features);

#ifdef CGAL_LINKED_WITH_TBB
	m_features.end_parallel_additions();
#endif

	delete m_sowf;
	m_sowf = new Sum_of_weighted_features(m_labels, m_features);
	if (m_ethz != NULL)
	{
		delete m_ethz;
		m_ethz = NULL;
	}
#ifdef CGAL_LINKED_WITH_OPENCV
	if (m_random_forest != NULL)
	{
		delete m_random_forest;
		m_random_forest = NULL;
	}
#endif
#ifdef CGAL_LINKED_WITH_TENSORFLOW
	if (m_neural_network != NULL)
	{
		delete m_neural_network;
		m_neural_network = NULL;
	}
#endif
	std::cerr << "Features = " << m_features.size() << std::endl;
}

void Surface_mesh_item_classification::train(int classifier, const QMultipleInputDialog& dialog)
{
	if (m_features.size() == 0)
	{
		std::cerr << "Error: features not computed" << std::endl;
		return;
	}

	m_label_probabilities.clear();
	m_label_probabilities.resize(m_labels.size());
	for (std::size_t i = 0; i < m_label_probabilities.size(); ++i)
		m_label_probabilities[i].resize(num_faces(*(m_mesh->polyhedron())));

	std::vector<std::size_t> training(num_faces(*(m_mesh->polyhedron())), std::size_t(-1));
	std::vector<std::size_t> indices(num_faces(*(m_mesh->polyhedron())), std::size_t(-1));

	std::vector<std::size_t> nb_label(m_labels.size(), 0);
	std::size_t nb_total = 0;

	BOOST_FOREACH(face_descriptor fd, faces(*(m_mesh->polyhedron())))
	{
		training[fd] = m_training[fd];
		if (training[fd] != std::size_t(-1))
		{
			nb_label[training[fd]] ++;
			++nb_total;
		}
	}

	std::cerr << nb_total << " face(s) used for training ("
		<< 100. * (nb_total / double(m_mesh->polyhedron()->faces().size())) << "% of the total):" << std::endl;
	for (std::size_t i = 0; i < m_labels.size(); ++i)
		std::cerr << " * " << m_labels[i]->name() << ": " << nb_label[i] << " face(s)" << std::endl;

	if (classifier == 0)
	{
		m_sowf->train<Concurrency_tag>(training, dialog.get<QSpinBox>("trials")->value());
		CGAL::Classification::classify<Concurrency_tag>(m_mesh->polyhedron()->faces(),
			m_labels, *m_sowf,
			indices, m_label_probabilities);
	}
	else if (classifier == 1)
	{
		if (m_ethz != NULL)
			delete m_ethz;
		m_ethz = new ETHZ_random_forest(m_labels, m_features);
		m_ethz->train<Concurrency_tag>(training, true,
			dialog.get<QSpinBox>("num_trees")->value(),
			dialog.get<QSpinBox>("max_depth")->value());
		CGAL::Classification::classify<Concurrency_tag>(m_mesh->polyhedron()->faces(),
			m_labels, *m_ethz,
			indices, m_label_probabilities);
	}
	else if (classifier == 2)
	{
#ifdef CGAL_LINKED_WITH_OPENCV
		if (m_random_forest != NULL)
			delete m_random_forest;
		m_random_forest = new Random_forest(m_labels, m_features,
			dialog.get<QSpinBox>("max_depth")->value(), 5, 15,
			dialog.get<QSpinBox>("num_trees")->value());
		m_random_forest->train(training);

		CGAL::Classification::classify<Concurrency_tag>(m_mesh->polyhedron()->faces(),
			m_labels, *m_random_forest,
			indices, m_label_probabilities);
#endif
	}
	else if (classifier == 3)
	{
#ifdef CGAL_LINKED_WITH_TENSORFLOW
		if (m_neural_network != NULL)
		{
			if (m_neural_network->initialized())
			{
				if (dialog.get<QCheckBox>("restart")->isChecked())
				{
					delete m_neural_network;
					m_neural_network = new Neural_network(m_labels, m_features);
				}
			}
			else
			{
				delete m_neural_network;
				m_neural_network = new Neural_network(m_labels, m_features);
			}
		}
		else
			m_neural_network = new Neural_network(m_labels, m_features);

		std::vector<std::size_t> hidden_layers;

		std::string hl_input = dialog.get<QLineEdit>("hidden_layers")->text().toStdString();
		if (hl_input != "")
		{
			std::istringstream iss(hl_input);
			int s;
			while (iss >> s)
				hidden_layers.push_back(std::size_t(s));
		}

		m_neural_network->train(training,
			dialog.get<QCheckBox>("restart")->isChecked(),
			dialog.get<QSpinBox>("trials")->value(),
			dialog.get<QDoubleSpinBox>("learning_rate")->value(),
			dialog.get<QSpinBox>("batch_size")->value(),
			hidden_layers);

		CGAL::Classification::classify<Concurrency_tag>(m_mesh->polyhedron()->faces(),
			m_labels, *m_neural_network,
			indices, m_label_probabilities);
#endif
	}

	BOOST_FOREACH(face_descriptor fd, faces(*(m_mesh->polyhedron())))
		m_classif[fd] = indices[fd];

	if (m_index_color == 1 || m_index_color == 2)
		change_color(m_index_color);
}

bool Surface_mesh_item_classification::run(int method, int classifier,
	std::size_t subdivisions, double smoothing)
{
	if (m_features.size() == 0)
	{
		std::cerr << "Error: features not computed" << std::endl;
		return false;
	}

	if (classifier == 0)
		run(method, *m_sowf, subdivisions, smoothing);
	else if (classifier == 1)
	{
		if (m_ethz == NULL)
		{
			std::cerr << "Error: ETHZ Random Forest must be trained or have a configuration loaded first" << std::endl;
			return false;
		}
		run(method, *m_ethz, subdivisions, smoothing);
	}
	else if (classifier == 2)
	{
#ifdef CGAL_LINKED_WITH_OPENCV
		if (m_random_forest == NULL)
		{
			std::cerr << "Error: OpenCV Random Forest must be trained or have a configuration loaded first" << std::endl;
			return false;
		}
		run(method, *m_random_forest, subdivisions, smoothing);
#endif
	}
	else if (classifier == 3)
	{
#ifdef CGAL_LINKED_WITH_TENSORFLOW
		if (m_neural_network == NULL)
		{
			std::cerr << "Error: TensorFlow Neural Network must be trained or have a configuration loaded first" << std::endl;
			return false;
		}
		run(method, *m_neural_network, subdivisions, smoothing);
#endif
	}

	return true;
}
