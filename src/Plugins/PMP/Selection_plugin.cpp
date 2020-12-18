#include <QtCore/qglobal.h>
#include <QMessageBox>

#include "Scene_surface_mesh_item.h"
#include "Scene_polyhedron_selection_item.h"
#include "Scene_polylines_item.h"

#include <CGAL/Three/Scene_interface.h>
#include <CGAL/Three/Three.h>
#include <CGAL/Three/Polyhedron_demo_plugin_helper.h>
#include <CGAL/Three/Polyhedron_demo_io_plugin_interface.h>
#include "ui_Selection_widget.h"

#include <QAction>
#include <QMainWindow>
#include <QApplication>

#include <map>

#include <boost/graph/adjacency_list.hpp>
#include <CGAL/boost/graph/split_graph_into_polylines.h>
#include <CGAL/Polygon_mesh_processing/border.h>
#include <CGAL/Polygon_mesh_processing/repair.h>
#include <CGAL/Polygon_mesh_processing/shape_predicates.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <Scene.h>
typedef Scene_surface_mesh_item Scene_face_graph_item;

typedef Scene_face_graph_item::Face_graph Face_graph;
typedef boost::property_map<Face_graph, CGAL::vertex_point_t>::type VPmap;

struct Is_terminal
{
	template <typename VertexDescriptor, typename Graph>
	bool operator ()(VertexDescriptor, const Graph&)
	{
		return false; // degree(vd,g) != 2; is a bad test in case of parallel edges
	}
};


template <typename Graph>
struct Polyline_visitor
{
	Scene_polylines_item* item;
	const Graph& points_pmap;

	Polyline_visitor(Scene_polylines_item* item_,
		const Graph& points_property_map)
		: item(item_),
		points_pmap(points_property_map)
	{}

	void start_new_polyline()
	{
		item->polylines.push_back(Scene_polylines_item::Polyline());
	}

	void add_node(typename boost::graph_traits<Graph>::vertex_descriptor vd)
	{
		item->polylines.back().push_back(points_pmap[vd]);
	}
	void end_polyline() {}
};
using namespace CGAL::Three;
class Polyhedron_demo_selection_plugin :
	public QObject,
	public Polyhedron_demo_plugin_helper,
	public Polyhedron_demo_io_plugin_interface
{
	Q_OBJECT
		Q_INTERFACES(CGAL::Three::Polyhedron_demo_plugin_interface CGAL::Three::Polyhedron_demo_io_plugin_interface)
		Q_PLUGIN_METADATA(IID "com.geometryfactory.PolyhedronDemo.PluginInterface/1.0" FILE "selection_plugin.json")
		Q_PLUGIN_METADATA(IID "com.geometryfactory.PolyhedronDemo.IOPluginInterface/1.0")
public:
	QString nameFilters() const { return "Selection files(*.selection.txt)"; }
	QString name() const { return "selection_sm_plugin"; }

	bool canLoad() const {
		Scene_item* item = CGAL::Three::Three::scene()->item(
			CGAL::Three::Three::scene()->mainSelectionIndex());
		Scene_facegraph_item* fg_item = qobject_cast<Scene_facegraph_item*>(item);
		if (fg_item)
			return true;
		Scene_polyhedron_selection_item* sel_item =
			qobject_cast<Scene_polyhedron_selection_item*>(item);
		if (sel_item)
			return true;
		return false;
	}

	CGAL::Three::Scene_item* load(QFileInfo fileinfo) {
		if (fileinfo.suffix().toLower() != "txt") return 0;
		// There will be no actual loading at this step.
		Scene_polyhedron_selection_item* item = new Scene_polyhedron_selection_item();
		if (!item->load(fileinfo.filePath().toStdString())) {
			delete item;
			return NULL;
		}
		item->setName(fileinfo.baseName());
		return item;
	}

	bool canSave(const CGAL::Three::Scene_item* scene_item) {
		return qobject_cast<const Scene_polyhedron_selection_item*>(scene_item);
	}
	bool save(const CGAL::Three::Scene_item* scene_item, QFileInfo fileinfo) {
		const Scene_polyhedron_selection_item* item = qobject_cast<const Scene_polyhedron_selection_item*>(scene_item);
		if (item == NULL) { return false; }

		//CGAL::Three::Three::warning("You are not saving your labeling work but the selection area. For saving the work, you need to select the first layer! ");

		return item->save(fileinfo.filePath().toStdString());
	}

	bool applicable(QAction*) const {
		return qobject_cast<Scene_face_graph_item*>(scene->item(scene->mainSelectionIndex()))
			|| qobject_cast<Scene_polyhedron_selection_item*>(scene->item(scene->mainSelectionIndex()));
	}
	void print_message(QString message) { CGAL::Three::Three::information(message); }
	QList<QAction*> actions() const { return QList<QAction*>() << actionSelection; }

	void init(QMainWindow* mainWindow, CGAL::Three::Scene_interface* scene_interface, Messages_interface* m) {
		mw = mainWindow;
		scene = scene_interface;
		messages = m;
		actionSelection = new QAction(
			QString("Surface Mesh Selection")
			, mw);
		actionSelection->setObjectName("actionSelection");
		connect(actionSelection, SIGNAL(triggered()), this, SLOT(selection_action()));
		last_mode = 0;
		dock_widget = new QDockWidget(
			"Surface Mesh Selection"
			, mw);
		dock_widget->setVisible(false);
		ui_widget.setupUi(dock_widget);
		dock_widget->setWindowTitle(tr(
			"Surface Mesh Selection"
		));
		connect(dock_widget, &QDockWidget::visibilityChanged,
			this, [this](bool b) {
				if (!b)
					this->set_operation_mode(-1);
			});

		addDockWidget(dock_widget);

		connect(ui_widget.Select_all_button, SIGNAL(clicked()), this, SLOT(on_Select_all_button_clicked()));
		//basic selection
		connect(ui_widget.Clear_all_button, SIGNAL(clicked()), this, SLOT(on_Clear_all_button_clicked()));
		connect(ui_widget.Inverse_selection_button, SIGNAL(clicked()), this, SLOT(on_Inverse_selection_button_clicked()));

		connect(ui_widget.SelectClasses, SIGNAL(clicked()), this, SLOT(on_Selected_class_clicked()));
		connect(ui_widget.select_classes_comboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(on_selected_label_combo_box_changed(int)));

		connect(ui_widget.Smart_type_combo_box, SIGNAL(currentIndexChanged(int)), this, SLOT(on_Smart_type_combo_box_changed(int)));
		connect(ui_widget.lassoCheckBox, &QCheckBox::toggled,
			this, &Polyhedron_demo_selection_plugin::on_LassoCheckBox_changed);
		connect(ui_widget.Brush_size_spin_box, SIGNAL(valueChanged(int)), this, SLOT(on_Brush_size_spin_box_changed(int)));
		connect(ui_widget.Expand_reduce_button, SIGNAL(clicked()), this, SLOT(on_Expand_reduce_button_clicked()));

		ui_widget.buttonGroup->setId(ui_widget.select_facet_radioButton, 0);
		ui_widget.buttonGroup->setId(ui_widget.select_segment_radioButton, 1);
		connect(ui_widget.select_facet_radioButton, SIGNAL(clicked(bool)), this, SLOT(group_selection_type_radio()));
		connect(ui_widget.select_segment_radioButton, SIGNAL(clicked(bool)), this, SLOT(group_selection_type_radio()));

		ui_widget.Smart_label->hide();
		ui_widget.Smart_type_combo_box->hide();

		QObject* scene = dynamic_cast<QObject*>(scene_interface);
		if (scene) {
			connect(scene, SIGNAL(itemAboutToBeDestroyed(CGAL::Three::Scene_item*)), this, SLOT(item_about_to_be_destroyed(CGAL::Three::Scene_item*)));
			connect(scene, SIGNAL(newItem(int)), this, SLOT(new_item_created(int)));
		}

		//Fill operations combo box.
		operations_strings = {
		  "Create Point Set from Selected Vertices"           ,
		  "Create Polyline from Selected Edges"               ,
		  "Create Facegraph from Selected Facets"            ,
		  "Erase Selected Facets"             ,
		  "Keep Connected Components of Selected Facets"           ,
		  "Expand Face Selection to Stay Manifold After Removal"   ,
		  "Convert from Edge Selection to Facets Selection"        ,
		  "Convert from Edge Selection to Point Selection"         ,
		  "Convert from Facet Selection to Boundary Edge Selection",
		  "Convert from Facet Selection to Point Selection",
		  "Convert selected facets into segment."
		};

		operations_map[operations_strings[0]] = 0;
		operations_map[operations_strings[1]] = 1;
		operations_map[operations_strings[2]] = 2;
		operations_map[operations_strings[3]] = 3;
		operations_map[operations_strings[4]] = 4;
		operations_map[operations_strings[5]] = 5;
		operations_map[operations_strings[6]] = 6;
		operations_map[operations_strings[7]] = 7;
		operations_map[operations_strings[8]] = 8;
		operations_map[operations_strings[9]] = 9;
		operations_map[operations_strings[10]] = 10;
	}
	virtual void closure()
	{
		dock_widget->hide();
	}
Q_SIGNALS:
	void save_handleType();
	void set_operation_mode(int);
	void set_selection_mode_index(int);
public Q_SLOTS:

	void connectItem(Scene_polyhedron_selection_item* new_item)
	{
		connect(this, SIGNAL(save_handleType()), new_item, SLOT(save_handleType()));
		connect(new_item, SIGNAL(updateInstructions(QString)), this, SLOT(setInstructions(QString)));
		connect(this, SIGNAL(set_operation_mode(int)), new_item, SLOT(set_operation_mode(int)));
		int item_id = scene->addItem(new_item);
		// QObject* scene_ptr = dynamic_cast<QObject*>(scene);
		// if (scene_ptr)
		//   connect(new_item,SIGNAL(simplicesSelected(CGAL::Three::Scene_item*)), scene_ptr, SLOT(setSelectedItem(CGAL::Three::Scene_item*)));
		connect(new_item, SIGNAL(isCurrentlySelected(Scene_facegraph_item_k_ring_selection*)), this, SLOT(isCurrentlySelected(Scene_facegraph_item_k_ring_selection*)));
		scene->setSelectedItem(item_id);
		//on_SelectionOrEuler_changed(ui_widget.selectionOrEuler->currentIndex());

		connect(this, SIGNAL(set_selection_mode_index(int)), new_item, SLOT(set_selection_mode_index(int)));
		on_selected_label_combo_box_changed(ui_widget.select_classes_comboBox->currentIndex());
		//connect(new_item->poly_item, SIGNAL(selection_done()), this, SLOT(reset_highlight_facets_if_selected()));

		if (last_mode == 0)
			on_Smart_type_combo_box_changed(ui_widget.Smart_type_combo_box->currentIndex());
	}
	// If the selection_item or the polyhedron_item associated to the k-ring_selector is currently selected,
	// set the k-ring_selector as currently selected. (A k-ring_selector tha tis not "currently selected" will
	// not process selection events)
	void isCurrentlySelected(Scene_facegraph_item_k_ring_selection* item)
	{
		if (scene->item_id(selection_item_map.find(item->poly_item)->second) == scene->mainSelectionIndex() ||
			scene->item_id(item->poly_item) == scene->mainSelectionIndex())
			item->setCurrentlySelected(true);
		else
			item->setCurrentlySelected(false);
	}

	void setInstructions(QString s)
	{
		//ui_widget.instructionsLabel->setText(s);
	}

	void printMessage(QString s)
	{
		print_message(s);
	}

	void selection_action() 
	{
		dock_widget->show();
		dock_widget->raise();
		Scene_face_graph_item* poly_item = getSelectedItem<Scene_face_graph_item>();
		if (!poly_item || selection_item_map.find(poly_item) != selection_item_map.end()) { return; }
		Scene_polyhedron_selection_item* new_item = new Scene_polyhedron_selection_item(poly_item, mw);
		new_item->setName(QString("%1 (selection)").arg(poly_item->name()));
		connectItem(new_item);

		new_item->setColor(QColor(255, 0, 0));
		poly_item->fill_classes_combo_box(ui_widget.select_classes_comboBox);
		poly_item->update_labels_for_selection();

		CGAL::Three::Three::SetdefaultSurfaceMeshRenderingMode(TextureModePlusFlatEdges);
		poly_item->setRenderingMode(TextureModePlusFlatEdges);
		//CGAL::Three::Three::information(QString("Reset the default rendering mode to TextureModePlusFlatEdges"));

		first_activate = false;
	}

	Scene_polyhedron_selection_item* onTheFlyItem() {
		Scene_face_graph_item* poly_item = qobject_cast<Scene_face_graph_item*>(scene->item(scene->mainSelectionIndex()));
		if (!poly_item)
			return NULL;
		Scene_polyhedron_selection_item* new_item = new Scene_polyhedron_selection_item(poly_item, mw);
		new_item->setName(QString("%1 (selection)").arg(poly_item->name()));
		connectItem(new_item);
		return new_item;
	}
	// Select all
	void on_Select_all_button_clicked() {
		Scene_polyhedron_selection_item* selection_item = getSelectedItem<Scene_polyhedron_selection_item>();
		if (!selection_item)
			selection_item = onTheFlyItem();
		if (!selection_item)
		{
			print_message("Error: there is no selected polyhedron selection item!");
			return;
		}

		if (ui_widget.Smart_type_combo_box->currentIndex() == 2 || 
			ui_widget.Smart_type_combo_box->currentIndex() == 3)
		{
			selection_item->select_all_facets_in_segment();
		}
		else
			selection_item->select_all();
	}

	void on_Selected_class_clicked()
	{
		Scene_polyhedron_selection_item* selection_item = getSelectedItem<Scene_polyhedron_selection_item>();
		int index = ui_widget.select_classes_comboBox->currentIndex();
		if (index == 0)
		{
			Q_EMIT selection_item->selected_HL(std::set<fg_face_descriptor>());
		}
		else
		{
			selection_item->clearHL();

			std::set<fg_face_descriptor> temp_segment_faces;
			int ind_new = index == 1 ? -1 : index - 2;
			if (!selection_item->poly_item->semantic_facet_map.empty() &&
				!selection_item->poly_item->semantic_facet_map[ind_new].empty())
			{
				BOOST_FOREACH(fg_face_descriptor fd, selection_item->poly_item->semantic_facet_map[ind_new])
				{
					if (fd.is_valid())
						temp_segment_faces.insert(fd);
				}
			}
			selection_item->set_is_insert(true);
			Q_EMIT selection_item->selected(temp_segment_faces);
			selection_item->poly_item->setRenderingMode(CGAL::Three::Three::defaultSurfaceMeshRenderingMode());
			//ui_widget.select_classes_comboBox->setCurrentIndex(0);
		}
	}

	void check_selection_validation() {
		Scene_polyhedron_selection_item* selection_item = getSelectedItem<Scene_polyhedron_selection_item>();
		if (!selection_item) {
			print_message("Error: there is no selected polyhedron selection item!");
			return;
		}
		selection_item->clear_all();
		selection_item->poly_item->unemphasize();
		selection_item->edited_segment = -1;
		//selection_item->segmentifySelection();
	}

	void group_selection_type_radio()
	{
		Scene_polyhedron_selection_item* selection_item = getSelectedItem<Scene_polyhedron_selection_item>();
		typedef Scene_polyhedron_selection_item::Active_handle Active_handle;

		switch (ui_widget.buttonGroup->checkedId())
		{
			case 0: //triangles
			{
				//rendering at first, adpat to later change
				CGAL::Three::Three::SetdefaultSurfaceMeshRenderingMode(TextureModePlusFlatEdges);
				selection_item->poly_item->setRenderingMode(TextureModePlusFlatEdges);
				selection_item->poly_item->showFacetEdges(true);

				//clear
				selection_item->clear_all();
				selection_item->set_active_handle_type(static_cast<Active_handle::Type>(1));//facet selection

				Q_EMIT save_handleType();
				Q_EMIT set_selection_mode_index(0);
				Q_EMIT set_operation_mode(-1);

				break;
			}

			case 1: //segments
			{
				//clear
				selection_item->clear_all();
				selection_item->set_active_handle_type(static_cast<Active_handle::Type>(0)); //0 segment selection

				//operations
				if (!first_activate)
					check_selection_validation();

				if (!original_fid_face_map.empty())
				{
					original_fid_face_map.clear();
					segment_fid_face_map.clear();
					face_center_point_set.clear();
					delete segment_mesh;
				}

				//enabled
				ui_widget.SelectClasses->setEnabled(true);
				ui_widget.select_classes_comboBox->setCurrentIndex(0);
				ui_widget.select_classes_comboBox->setEnabled(true);

				Q_EMIT save_handleType();
				Q_EMIT set_selection_mode_index(1);
				Q_EMIT set_operation_mode(-1);

				//rendering at last, use last as final rendering
				CGAL::Three::Three::SetdefaultSurfaceMeshRenderingMode(TextureModePlusFlatEdges);
				selection_item->poly_item->setRenderingMode(TextureModePlusFlatEdges);
				selection_item->poly_item->showFacetEdges(false);

				break;
			}
		}
	}

	void on_Clear_all_button_clicked() {
		Scene_polyhedron_selection_item* selection_item = getSelectedItem<Scene_polyhedron_selection_item>();
		if (!selection_item) {
			print_message("Error: there is no selected polyhedron selection item!");
			return;
		}
		selection_item->clearHL();
		selection_item->clear_all();
		reset_highlight_facets_if_selected();
	}

	void on_Inverse_selection_button_clicked()
	{
		Scene_polyhedron_selection_item* selection_item = getSelectedItem<Scene_polyhedron_selection_item>();
		if (!selection_item) {
			print_message("Error: there is no selected polyhedron selection item!");
			return;
		}

		selection_item->inverse_selection();
	}

	void reset_highlight_facets_if_selected()
	{
		Scene_polyhedron_selection_item* selection_item = getSelectedItem<Scene_polyhedron_selection_item>();
		if (ui_widget.select_classes_comboBox->currentIndex() != 0)
		{
			ui_widget.select_classes_comboBox->setCurrentIndex(0);
			on_Selected_class_clicked();
		}
	}

	void on_LassoCheckBox_changed(bool b)
	{
		for (Selection_item_map::iterator it = selection_item_map.begin(); it != selection_item_map.end(); ++it)
		{
			it->second->set_lasso_mode(b);
		}
	}
	
	void on_selected_label_combo_box_changed(int index)
	{
		Scene_polyhedron_selection_item* selection_item = getSelectedItem<Scene_polyhedron_selection_item>();
		std::set<fg_face_descriptor> temp_segment_faces;
		int ind_new = index == 1 ? -1 : index - 2;
		if (!selection_item->poly_item->semantic_facet_map.empty() && ind_new > -2)
		{
			BOOST_FOREACH(fg_face_descriptor fd, selection_item->poly_item->semantic_facet_map[ind_new])
			{
				if (fd.is_valid())
					temp_segment_faces.insert(fd);
			}
		}
		if (index != 0)
			selection_item->poly_item->setRenderingMode(Gouraud);
		else
			selection_item->poly_item->setRenderingMode(CGAL::Three::Three::defaultSurfaceMeshRenderingMode());

		if (!selection_item->selected_facets.empty())
		{
			selection_item->clear_all();
		}
		Q_EMIT selection_item->selected_HL(temp_segment_faces);
	}

	void on_Smart_type_combo_box_changed(int index) {
		
		//Scene_polyhedron_selection_item* selection_item = getSelectedItem<Scene_polyhedron_selection_item>();

		typedef Scene_polyhedron_selection_item::Active_handle Active_handle;
		for (Selection_item_map::iterator it = selection_item_map.begin(); it != selection_item_map.end(); ++it) {

			//reset
			ui_widget.Brush_size_spin_box->setValue(0);
			ui_widget.Expand_reduce_spin_box->setValue(0);
			ui_widget.select_classes_comboBox->setCurrentIndex(0);

			//enabled
			ui_widget.select_facet_radioButton->setEnabled(true);
			ui_widget.select_segment_radioButton->setEnabled(true);

			//disabled
			ui_widget.SelectClasses->setDisabled(true);
			ui_widget.select_classes_comboBox->setDisabled(true);
			ui_widget.lassoCheckBox->setChecked(false);

			if (index == 0) //no smart functions
			{
				//rendering
				CGAL::Three::Three::SetdefaultSurfaceMeshRenderingMode(TextureModePlusFlatEdges);
				it->second->poly_item->setRenderingMode(TextureModePlusFlatEdges);
				it->second->poly_item->showFacetEdges(false);

				//enabled
				ui_widget.SelectClasses->setEnabled(true);
				ui_widget.select_classes_comboBox->setCurrentIndex(0);
				ui_widget.select_classes_comboBox->setEnabled(true);

				ui_widget.selection_rings_label->setEnabled(true);
				ui_widget.Brush_size_spin_box->setEnabled(true);
				ui_widget.Expand_reduce_spin_box->setEnabled(true);
				ui_widget.Expand_reduce_button->setEnabled(true);
				ui_widget.lassoCheckBox->setEnabled(true);
				it->second->check_expand_reduce_enabled(true);

				//clear
				on_Clear_all_button_clicked();

				//set seleciton mode
				ui_widget.select_facet_radioButton->setChecked(false);
				ui_widget.select_segment_radioButton->setChecked(true);
				group_selection_type_radio();
			}
		}
	}

	void on_Insertion_radio_button_toggled(bool toggle) {
		for (Selection_item_map::iterator it = selection_item_map.begin(); it != selection_item_map.end(); ++it) {
			it->second->set_is_insert(toggle);
		}
	}

	void on_Brush_size_spin_box_changed(int value) {
		for (Selection_item_map::iterator it = selection_item_map.begin(); it != selection_item_map.end(); ++it) {
			it->second->set_k_ring(value);
		}
	}

	void on_Expand_reduce_button_clicked() {
		Scene_polyhedron_selection_item* selection_item = getSelectedItem<Scene_polyhedron_selection_item>();
		if (!selection_item) 
		{
			print_message("Error: there is no selected polyhedron selection item!");
			return;
		}

		int steps = ui_widget.Expand_reduce_spin_box->value();
		selection_item->segment_expand_or_reduce(steps);
	}
	
	// To handle empty selection items coming from loader
	void new_item_created(int item_id)
	{
		typedef Scene_polyhedron_selection_item::Active_handle Active_handle;
		Scene_polyhedron_selection_item* selection_item =
			qobject_cast<Scene_polyhedron_selection_item*>(scene->item(item_id));
		if (!selection_item) { return; }

		Scene_face_graph_item* poly_item = NULL;
		if (selection_item->polyhedron_item() == NULL) { //coming from selection_io loader
			bool found = false;
			for (int i = 0; i < scene->numberOfEntries(); ++i) {
				poly_item = qobject_cast<Scene_face_graph_item*>(scene->item(i));
				if (!poly_item)
					continue;
				if (!selection_item->actual_load(poly_item, mw)) {
					continue;
				}
				found = true;
				selection_item->invalidateOpenGLBuffers();
				scene->itemChanged(selection_item);
				break;
			}
			if (!found)
			{
				print_message("Error: loading selection item is not successful!");
				scene->erase(item_id);
				return;
			}
		}
		else 
		{
			poly_item = getSelectedItem<Scene_face_graph_item>();
		}
		if (!poly_item) {
			print_message("Error: please select corresponding polyhedron item from Geometric Objects list.");
			scene->erase(item_id);
			return;
		}

		// now set default params both for selection items coming from selection_io, or on_Create_selection_item_button_clicked
		Active_handle::Type aht = Active_handle::SEGMENT;//static_cast<Active_handle::Type>(ui_widget.Smart_type_combo_box->currentIndex());
		
		//bool is_insert = ui_widget.Insertion_radio_button->isChecked();
		bool is_insert = true;
		int k_ring = ui_widget.Brush_size_spin_box->value();

		selection_item->set_active_handle_type(aht);
		selection_item->set_is_insert(is_insert);
		selection_item->set_k_ring(k_ring);
		selection_item->setRenderingMode(Flat);
		if (selection_item->name() == "unamed") {
			selection_item->setName(tr("%1 (selection)").arg(poly_item->name()));
		}

		selection_item_map.insert(std::make_pair(poly_item, selection_item));
		connect(this, SIGNAL(save_handleType()), selection_item, SLOT(save_handleType()));
		connect(selection_item, SIGNAL(updateInstructions(QString)), this, SLOT(setInstructions(QString)));
		connect(selection_item, SIGNAL(printMessage(QString)), this, SLOT(printMessage(QString)));
		connect(this, SIGNAL(set_operation_mode(int)), selection_item, SLOT(set_operation_mode(int)));

		connect(this, SIGNAL(set_selection_mode_index(int)), selection_item, SLOT(set_selection_mode_index(int)));

		//QObject* scene_ptr = dynamic_cast<QObject*>(scene);
		//if (scene_ptr)
		//  connect(selection_item,SIGNAL(simplicesSelected(CGAL::Three::Scene_item*)), scene_ptr, SLOT(setSelectedItem(CGAL::Three::Scene_item*)));
		connect(selection_item, SIGNAL(isCurrentlySelected(Scene_facegraph_item_k_ring_selection*)), this, SLOT(isCurrentlySelected(Scene_facegraph_item_k_ring_selection*)));
		
		on_LassoCheckBox_changed(ui_widget.lassoCheckBox->isChecked());
		ui_widget.Brush_size_spin_box->setValue(0);
		ui_widget.Expand_reduce_spin_box->setValue(0);
		ui_widget.Smart_type_combo_box->setCurrentIndex(0);
		//if (!from_plugin) {
		//	ui_widget.selectionOrEuler->setCurrentIndex(0);
		//}
		//else
			from_plugin = false;
		//on_SelectionOrEuler_changed(ui_widget.selectionOrEuler->currentIndex());

		if (last_mode == 0)
			on_Smart_type_combo_box_changed(ui_widget.Smart_type_combo_box->currentIndex());
	}

	void item_about_to_be_destroyed(CGAL::Three::Scene_item* scene_item) {
		// if polyhedron item
		Scene_face_graph_item* poly_item = qobject_cast<Scene_face_graph_item*>(scene_item);
		if (poly_item) {
			std::pair<Selection_item_map::iterator, Selection_item_map::iterator> res =
				selection_item_map.equal_range(poly_item);

			for (Selection_item_map::iterator begin = res.first; begin != res.second; ) {
				Scene_polyhedron_selection_item* selection_item = begin->second;
				selection_item_map.erase(begin++); // first erase from map, because scene->erase will cause a call to this function
				scene->erase(scene->item_id(selection_item));
			}
		}
		// if polyhedron selection item
		Scene_polyhedron_selection_item* selection_item = qobject_cast<Scene_polyhedron_selection_item*>(scene_item);
		if (selection_item) {
			Scene_face_graph_item* poly_item = selection_item->polyhedron_item();
			std::pair<Selection_item_map::iterator, Selection_item_map::iterator> res =
				selection_item_map.equal_range(poly_item);
			for (Selection_item_map::iterator begin = res.first; begin != res.second; ++begin) {
				if (begin->second == selection_item) {
					selection_item_map.erase(begin); break;
				}
			}
		}
	}

private:
	Messages_interface* messages;
	QAction* actionSelection;

	QDockWidget* dock_widget;
	Ui::Selection ui_widget;
	std::map<QString, int> operations_map;
	std::vector<QString> operations_strings;
	typedef std::multimap<Scene_face_graph_item*, Scene_polyhedron_selection_item*> Selection_item_map;
	Selection_item_map selection_item_map;
	int last_mode;
	bool from_plugin;
	int current_label_index = 0;
	bool first_activate = true, is_split_button_clicked = false;
	SMesh* segment_mesh;
	std::map<int, face_descriptor> original_fid_face_map, segment_fid_face_map;
	Point_range_cgal face_center_point_set;
	float segment_area = 0.0f;
}; // end Polyhedron_demo_selection_plugin

//Q_EXPORT_PLUGIN2(Polyhedron_demo_selection_plugin, Polyhedron_demo_selection_plugin)

#include "Selection_plugin.moc"
