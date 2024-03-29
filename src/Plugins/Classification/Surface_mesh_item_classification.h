#ifndef SURFACE_MESH_ITEM_CLASSIFICATION_H
#define SURFACE_MESH_ITEM_CLASSIFICATION_H

//#define CGAL_DO_NOT_USE_BOYKOV_KOLMOGOROV_MAXFLOW_SOFTWARE
#define CGAL_CLASSIFICATION_VERBOSE

#include <CGAL/Three/Scene_item.h>

#include "Scene_surface_mesh_item.h"
#include "Scene_polyhedron_selection_item.h"
#include "Item_classification_base.h"

//#include <CGAL/Classification.h>
#include <CGAL/Classification/Label.h>
#include <CGAL/Classification/Label_set.h>
#include <CGAL/Classification/property_maps.h>

#include <iostream>

typedef CGAL::Sequential_tag Concurrency_tag;

class Surface_mesh_item_classification : public Item_classification_base
{
public:

  typedef SMesh Mesh;
  typedef Kernel::Point_3 Point;
  typedef Scene_polyhedron_selection_item::Selection_set_facet Selection;
  typedef Mesh::Face_range Face_range;
  typedef boost::graph_traits<Mesh>::face_descriptor face_descriptor;
  typedef boost::graph_traits<Mesh>::vertex_descriptor vertex_descriptor;
  typedef CGAL::Identity_property_map<face_descriptor> Face_map;


  typedef CGAL::Classification::Face_descriptor_to_center_of_mass_map<Mesh> Face_center_map;
  typedef CGAL::Classification::Face_descriptor_to_face_descriptor_with_bbox_map<Mesh> Face_descriptor_with_bbox_map;

public:
  
  Surface_mesh_item_classification(Scene_surface_mesh_item* mesh);
  ~Surface_mesh_item_classification();

  void backup_existing_colors_and_add_new();

  CGAL::Three::Scene_item* item() { return m_mesh; }
  void erase_item() { m_mesh = NULL; }

  bool segment_form() {
	  if (m_selection == NULL)
		  return true;

	  if (m_selection->get_active_handle_type_public() == 1 || 
		  m_selection->get_active_handle_type_public() == 7) {
		  if (!m_selection->put_selected_faces_into_one_segment()) {
			  return false;
		  }
	  }
	  return true;
  }

  void add_selection_to_training_set (std::size_t label)
  {
    if (m_selection == NULL)
      return;

	int updated_labeled_faces = 0;
	m_selection->polyhedron_item()->selected_facets_for_annotation.clear();
    for (Selection::iterator it = m_selection->selected_facets.begin();
         it != m_selection->selected_facets.end(); ++ it)
    {
      m_classif[*it] = label;
      m_training[*it] = label;
	  m_selection->polyhedron_item()->selected_facets_for_annotation.push_back(*it);
	  if (!m_face_checked[*it] && label != m_label_updated[*it])
	  {
		  ++updated_labeled_faces;
		  m_face_checked[*it] = true;
	  }

	  if (m_mesh->face_label[*it] == -1)
		  --unlabelled_num_faces_global;
    }

	m_selection->polyhedron_item()->is_in_annotation = true;
    //if (m_index_color == 1 || m_index_color == 2)
    //  change_color (m_index_color);

	m_mesh->total_labeled_faces += updated_labeled_faces;
	change_color(m_index_color);
	m_selection->clear_all();
  }
  
  void reset_training_set (std::size_t label)
  {
    BOOST_FOREACH(face_descriptor fd, faces(*(m_mesh->polyhedron())))
      if (m_training[fd] == label)
        m_training[fd] = std::size_t(-1);
    if (m_index_color == 1 || m_index_color == 2)
      change_color (m_index_color);
  }
  
  void reset_training_set_of_selection ()
  {
    for (Selection::iterator it = m_selection->selected_facets.begin();
         it != m_selection->selected_facets.end(); ++ it)
    {
      m_training[*it] = -1;
      m_classif[*it] = -1;
    }
    m_selection->clear_all();
    
    if (m_index_color == 1 || m_index_color == 2)
      change_color (m_index_color);
  }
  
  void reset_training_sets()
  {
    BOOST_FOREACH(face_descriptor fd, faces(*(m_mesh->polyhedron())))
      m_training[fd] = std::size_t(-1);
    if (m_index_color == 1 || m_index_color == 2)
      change_color (m_index_color);
  }
  
  void select_random_region()
  {
    // TODO
    std::cerr << "Warning: operation not yet available for meshes." << std::endl;
  }
  
  void validate_selection ()
  {
    if (m_selection == NULL)
      return;
    
    for (Selection::iterator it = m_selection->selected_facets.begin();
         it != m_selection->selected_facets.end(); ++ it)
      m_training[*it] = m_classif[*it];
    m_selection->clear_all();
    
    if (m_index_color == 1 || m_index_color == 2)
      change_color (m_index_color);
  }
  
  void train(int classifier, const QMultipleInputDialog& dialog);
  
  bool run (int method, int classifier, std::size_t subdivisions, double smoothing);

  void update_color() { change_color (m_index_color); }
  
  void change_color (int index, float* vmin = NULL, float* vmax = NULL);
  
  void threshold_based_change_color(int index, std::vector<float> &thresholds, std::vector<bool>& belows, float* vmin = NULL, float* vmax = NULL);

  bool can_show_probability();

  void update_all_label_color(int &index);
  int get_total_number_facets();
  int get_unlabelled_number_facets();
  int get_total_labeled_facets();
  int get_total_error_facets();
  int unlabelled_num_faces_global = 0;
 
  CGAL::Three::Scene_item* generate_one_item (const char* /* name */,
                                              int /* label */) const
  {
    // TODO
    std::cerr << "Warning: operation not yet available for meshes." << std::endl;
    return NULL;
  }
  void generate_one_item_per_label(std::vector<CGAL::Three::Scene_item*>&,
                                   const char*) const
  {
    // TODO
    std::cerr << "Warning: operation not yet available for meshes." << std::endl;
  }

  void set_selection_item (Scene_polyhedron_selection_item* selection)
  {
    m_selection = selection;

  }

protected:

  Scene_surface_mesh_item* m_mesh;
  Scene_polyhedron_selection_item* m_selection;
  Mesh::Property_map<face_descriptor, std::size_t> m_training;
  Mesh::Property_map<face_descriptor, std::size_t> m_classif;
  Mesh::Property_map<face_descriptor, CGAL::Color> m_color;
  Mesh::Property_map<face_descriptor, CGAL::Color> m_real_color;
  Mesh::Property_map<face_descriptor, float> m_label_prob;
  std::map<face_descriptor, bool> m_face_checked;
  std::map<face_descriptor, int> m_label_updated;
  std::vector<std::vector<float> > m_label_probabilities;

  int m_index_color;
public:
	QColor add_new_label(const char* name)
	{
		QColor out = Item_classification_base::add_new_label(name);
		update_comments_of_facet_set_item();
		return out;
	}

	void remove_label(std::size_t position)
	{
		Item_classification_base::remove_label(position);

		BOOST_FOREACH(face_descriptor fd, faces(*(m_mesh->polyhedron())))
		{
			if (m_training[fd] == int(position))
				m_training[fd] = -1;
			else if (m_training[fd] > int(position))
				m_training[fd] --;

			if (m_classif[fd] == int(position))
				m_classif[fd] = -1;
			else if (m_classif[fd] > int(position))
				m_classif[fd] --;
		}
		update_comments_of_facet_set_item();
	}

//private:
	void update_comments_of_facet_set_item()
	{
		std::string& comments = m_mesh->comments();

		// Remove previously registered labels from comments
		std::string new_comment;

		std::istringstream stream(comments);
		std::string line;
		while (getline(stream, line))
		{
			std::stringstream iss(line);
			std::string tag;
			if (iss >> tag && tag == "label")
				continue;
			new_comment += line + "\n";
		}
		comments = new_comment;

		//comments += "label -1 unclassified\n";
		for (std::size_t i = 0; i < m_labels.size(); ++i)
		{
			std::ostringstream oss;
			if (i == 0)
				oss << "label " << -1 << " " << m_labels[i]->name() << std::endl;
			else
				oss << "label " << i << " " << m_labels[i]->name() << std::endl;
			comments += oss.str();
		}
	}
};




#endif // SURFACE_MESH_ITEM_CLASSIFICATION_H
