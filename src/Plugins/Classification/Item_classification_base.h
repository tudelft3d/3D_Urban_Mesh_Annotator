#ifndef ITEM_CLASSIFICATION_BASE_H
#define ITEM_CLASSIFICATION_BASE_H

#include <CGAL/Three/Scene_item.h> 

#include <QComboBox>
#include <QLineEdit>
#include <QSpinBox>
#include <QMultipleInputDialog.h>

#include <CGAL/Classification/Label_set.h>

class Item_classification_base
{
public:
  typedef CGAL::Classification::Label_handle   Label_handle;
  typedef CGAL::Classification::Label_set   Label_set;

public:
  
  Item_classification_base() { }
  virtual ~Item_classification_base() { }

  virtual CGAL::Three::Scene_item* item() = 0;
  virtual void erase_item() = 0;

  virtual CGAL::Bbox_3 bbox() { return item()->bbox(); }


  virtual void add_selection_to_training_set (std::size_t label) = 0;
  virtual void reset_training_set (std::size_t label) = 0;
  virtual void reset_training_set_of_selection() = 0;
  virtual void reset_training_sets() = 0;

  virtual void validate_selection () = 0;

  virtual void update_color () = 0;
  virtual void change_color (int index, float* vmin = NULL, float* vmax = NULL) = 0;

  virtual void update_all_label_color(int &) = 0;
  virtual void threshold_based_change_color(int index, int threshold, bool below, float* vmin = NULL, float* vmax = NULL) = 0;
  // presently only implementated the surface_mesh_item_classification class.
  virtual bool can_show_probability() = 0;

  virtual bool segment_form() { 
	  return true; 
  }

  //virtual void show_labeling_progress() = 0;

  virtual int get_total_number_facets() = 0;
  virtual int get_unlabelled_number_facets() = 0;
  virtual int get_total_labeled_facets() = 0;
  virtual int get_total_error_facets() = 0;

  virtual CGAL::Three::Scene_item* generate_one_item (const char* name,
                                                      int label) const = 0;
  virtual void generate_one_item_per_label(std::vector<CGAL::Three::Scene_item*>& items,
                                           const char* name) const = 0;

  virtual QColor add_new_label (const char* name)
  {
    m_labels.add(name);
    m_label_colors.push_back (get_new_label_color (name));

    return m_label_colors.back();
  }
  virtual void remove_label (std::size_t position)
  {
    m_labels.remove(m_labels[position]);
    m_label_colors.erase (m_label_colors.begin() + position);
  }
  
  virtual void clear_labels ()
  {
    m_labels.clear();
    m_label_colors.clear();
  }
  std::size_t number_of_labels() const { return m_labels.size(); }
  Label_handle label(std::size_t i) { return m_labels[i]; }

  virtual void fill_display_combo_box (QComboBox* cb) const
  {
    for (std::size_t i = 0; i < m_labels.size(); ++ i)
      {
        std::ostringstream oss;
        oss << "Label " << m_labels[i]->name();
        cb->addItem (oss.str().c_str());
      }
  }

  const QColor& label_color(std::size_t i) const { return m_label_colors[i]; }
  void change_label_color (std::size_t position, const QColor& color)
  {
    m_label_colors[position] = color;
  }
  void change_label_name (std::size_t position, const std::string& name)
  {
    m_labels[position]->set_name (name);
  }

  QColor get_new_label_color(const std::string& name)
  {
	  QColor color(64 + rand() % 192,
		  64 + rand() % 192,
		  64 + rand() % 192);

	  if (name == "ground")
		  color = QColor(170, 85, 0);//color = QColor (186, 189, 182);
	  else if (name == "low_veget")
		  color = QColor(78, 154, 6);
	  else if (name == "med_veget"
		  || name == "vegetation")
		  color = QColor(0, 255, 0);//color = QColor (138, 226, 52);
	  else if (name == "high_veget")
		  color = QColor(204, 255, 201);
	  else if (name == "building"
		  || name == "roof")
		  color = QColor(255, 255, 0);//color = QColor (245, 121, 0);
	  else if (name == "noise")
		  color = QColor(0, 0, 0);
	  else if (name == "reserved")
		  color = QColor(233, 185, 110);
	  else if (name == "water")
		  color = QColor(0, 255, 255); //color = QColor (114, 159, 207);
	  else if (name == "rail")
		  color = QColor(136, 46, 25);
	  else if (name == "road_surface")
		  color = QColor(56, 56, 56);
	  else if (name == "reserved_2")
		  color = QColor(193, 138, 51);
	  else if (name == "wire_guard")
		  color = QColor(37, 61, 136);
	  else if (name == "wire_conduct")
		  color = QColor(173, 127, 168);
	  else if (name == "trans_tower")
		  color = QColor(136, 138, 133);
	  else if (name == "wire_connect")
		  color = QColor(145, 64, 236);
	  else if (name == "bridge_deck")
		  color = QColor(213, 93, 93);
	  else if (name == "high_noise")
		  color = QColor(255, 0, 0);
	  else if (name == "facade")
		  color = QColor(77, 131, 186);
	  else if (name == "vehicle")
		  color = QColor(255, 0, 255);
	  else if (name == "car")
		  color = QColor(255, 0, 255);
	  else if (name == "boat")
		  color = QColor(0, 0, 153);
	  else if (name == "unclassified")
		  color = QColor(0, 0, 0);
	  else if (name == "building_unknown")
		  color = QColor(50, 0, 50);
	  else if (name == "balcony")
		  color = QColor(0, 204, 204);
	  else if (name == "chimney")
		  color = QColor(255, 0, 127);
	  else if (name == "dormer")
		  color = QColor(85, 0, 255);
	  return color;
  }

protected:

  Label_set m_labels;
  std::vector<QColor> m_label_colors;
};




#endif // ITEM_CLASSIFICATION_BASE_H
