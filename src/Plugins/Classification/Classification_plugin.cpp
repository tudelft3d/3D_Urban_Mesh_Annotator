#include <QtCore/qglobal.h>
#include <QFileDialog>
#include <QColorDialog> 
#include <CGAL/Qt/manipulatedCameraFrame.h>
#include <CGAL/Qt/manipulatedFrame.h>

#include <fstream>


#include "Messages_interface.h"
#include "Item_classification_base.h"
#include "Scene_surface_mesh_item.h"
#include "Surface_mesh_item_classification.h"
#include "Scene_polylines_item.h"
#include "Scene_polygon_soup_item.h"

#include <CGAL/Three/Scene_interface.h> 
#include <CGAL/Three/Polyhedron_demo_plugin_helper.h>
#include <CGAL/Three/Three.h>

#include <CGAL/Random.h>
#include <CGAL/Real_timer.h>

#include <QMultipleInputDialog.h>

#include "ui_Classification_widget.h"

#include <QAction>
#include <QMainWindow>
#include <QApplication>
#include <QCheckBox>
#include <QRadioButton>
#include <QInputDialog>
#include <QMessageBox>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QSlider>

#include <map>

#include <boost/graph/adjacency_list.hpp>
#include <CGAL/boost/graph/split_graph_into_polylines.h>

using namespace CGAL::Three;
class Polyhedron_demo_classification_plugin :
	public QObject,
	public Polyhedron_demo_plugin_helper
{
	Q_OBJECT
		Q_INTERFACES(CGAL::Three::Polyhedron_demo_plugin_interface)
		Q_PLUGIN_METADATA(IID "com.geometryfactory.PolyhedronDemo.PluginInterface/1.0")

		struct LabelButton
	{
		QPushButton* color_button;
		QMenu* menu;
		char shortcut;

		QColor color;

		QLabel* label2;

		LabelButton(QWidget* parent,
			const char* name,
			const QColor& color,
			const char shortcut)
			: shortcut(shortcut), color(color)
		{
			color_button = new QPushButton(tr("%1 (%2)").arg(name).arg((char)(std::toupper(shortcut))), parent);

			menu = new QMenu("Label Menu", color_button);

			QColor text_color(255, 255, 255);
			if (color.red() * 0.299 + color.green() * 0.587 + color.blue() * 0.114 > 128)
				text_color = QColor(0, 0, 0);

			QString s("QPushButton { font-weight: bold; background: #"
				+ QString(color.red() < 16 ? "0" : "") + QString::number(color.red(), 16)
				+ QString(color.green() < 16 ? "0" : "") + QString::number(color.green(), 16)
				+ QString(color.blue() < 16 ? "0" : "") + QString::number(color.blue(), 16)
				+ "; color: #"
				+ QString(text_color.red() < 16 ? "0" : "") + QString::number(text_color.red(), 16)
				+ QString(text_color.green() < 16 ? "0" : "") + QString::number(text_color.green(), 16)
				+ QString(text_color.blue() < 16 ? "0" : "") + QString::number(text_color.blue(), 16)
				+ "; }");

			color_button->setStyleSheet(s);
			color_button->setMenu(menu);

			label2 = new QLabel(name, parent);
		}
		~LabelButton()
		{
		}
		void change_color(const QColor& color)
		{
			this->color = color;
			QColor text_color(255, 255, 255);
			if (color.red() * 0.299 + color.green() * 0.587 + color.blue() * 0.114 > 128)
				text_color = QColor(0, 0, 0);
			QString s("QPushButton { font-weight: bold; background: #"
				+ QString(color.red() < 16 ? "0" : "") + QString::number(color.red(), 16)
				+ QString(color.green() < 16 ? "0" : "") + QString::number(color.green(), 16)
				+ QString(color.blue() < 16 ? "0" : "") + QString::number(color.blue(), 16)
				+ "; color: #"
				+ QString(text_color.red() < 16 ? "0" : "") + QString::number(text_color.red(), 16)
				+ QString(text_color.green() < 16 ? "0" : "") + QString::number(text_color.green(), 16)
				+ QString(text_color.blue() < 16 ? "0" : "") + QString::number(text_color.blue(), 16)
				+ "; }");

			color_button->setStyleSheet(s);
			color_button->update();
		}

	};

public:
	bool applicable(QAction*) const {
		return
			qobject_cast<Scene_surface_mesh_item*>(scene->item(scene->mainSelectionIndex()));
	}
	void print_message(QString message) { CGAL::Three::Three::information(message); }
	QList<QAction*> actions() const { return QList<QAction*>() << actionClassification; }

	using Polyhedron_demo_plugin_helper::init;
	void init(QMainWindow* mainWindow, CGAL::Three::Scene_interface* scene_interface, Messages_interface* m) {

		mw = mainWindow;
		scene = scene_interface;
		messages = m;
		//***********************Weixiao Update menu name*******************************//
		//actionClassification = new QAction(tr("Classification"), mw);
		actionClassification = new QAction(tr("3D Annotation"), mw);
		//********************************************************************//
		connect(actionClassification, SIGNAL(triggered()), this, SLOT(classification_action()));
		//***********************Weixiao Update menu name*******************************//
		//dock_widget = new QDockWidget("Classification", mw);
		dock_widget = new QDockWidget("3D Annotation", mw);
		//********************************************************************//
		dock_widget->setVisible(false);

		label_button = new QPushButton(QIcon(QString(":/cgal/icons/plus")), "", dock_widget);
		//***********************Weixiao Update menu name*********************//
		label_button->setDisabled(true);
		label_button->setVisible(false);
		//********************************************************************//
		QMenu* label_menu = new QMenu("Label Menu", label_button);
		label_button->setMenu(label_menu);
		QAction* add_new_label = label_menu->addAction("Add new label(s)");
		connect(add_new_label, SIGNAL(triggered()), this,
			SLOT(on_add_new_label_clicked()));

		QAction* clear_labels = label_menu->addAction("Clear labels");
		//***********************Weixiao Update menu name*********************//
		clear_labels->setDisabled(true);
		clear_labels->setVisible(false);
		//********************************************************************//
		connect(clear_labels, SIGNAL(triggered()), this,
			SLOT(on_clear_labels_clicked()));

		ui_widget.setupUi(dock_widget);
		addDockWidget(dock_widget);

		color_att = QColor(75, 75, 77);
		
		connect(ui_widget.help, SIGNAL(clicked()), this,
			SLOT(on_help_clicked()));

		ui_widget.help->setVisible(false);
		connect(ui_widget.close, SIGNAL(clicked()), this,
			SLOT(ask_for_closing()));

		//***********************Ziqian && Weixiao*******************************//
		connect(ui_widget.display, SIGNAL(currentIndexChanged(int)), this,
			SLOT(on_display_button_clicked_with_probability(int)));

		connect(ui_widget.ProbSpin, SIGNAL(valueChanged(int)), ui_widget.ProbSlider, SLOT(setValue(int)));
		connect(ui_widget.ProbSlider, SIGNAL(valueChanged(int)), ui_widget.ProbSpin, SLOT(setValue(int)));

		connect(ui_widget.ProbSpin, SIGNAL(valueChanged(int)), this, SLOT(on_probability_threshold_changed(int)));
		//connect(ui_widget.ProbSlider, SIGNAL(valueChanged(int)), this, SLOT(on_probability_threshold_changed(int)));

		connect(ui_widget.ProbSwitcher, SIGNAL(currentIndexChanged(int)), this, SLOT(on_probability_switcher_changed(int)));
		//********************************************************************//

		//*******************************Weixiao **************************************//
		//connect(ui_widget.lineEdit, SIGNAL(triggered()), this, SLOT(show_total_lables(int))); //Total
		//connect(ui_widget.lineEdit_2, SIGNAL(on_add_selection_to_training_set_clicked()), this, SLOT(get_unlabelled_number_facets())); //Finished
		//connect(ui_widget.progressBar, SIGNAL(currentIndexChanged(int)), this, SLOT(show_progress_bar(int)));
		//********************************************************************//


		QObject* scene_obj = dynamic_cast<QObject*>(scene_interface);
		if (scene_obj)
		{
			connect(scene_obj, SIGNAL(itemAboutToBeDestroyed(CGAL::Three::Scene_item*)), this,
				SLOT(item_about_to_be_destroyed(CGAL::Three::Scene_item*)));

			//*************Weixiao*************//
			//connect(scene_obj, SIGNAL(itemIndexSelected(int)), this,
			//	SLOT(update_plugin(int)));
			//*********************************//
		}
	}
	virtual void closure()
	{
		dock_widget->hide();
		close_classification();
	}


public Q_SLOTS:

	void item_about_to_be_destroyed(CGAL::Three::Scene_item* scene_item) {
		Item_map::iterator it = item_map.find(scene_item);
		if (it != item_map.end())
		{
			Item_classification_base* classif = it->second;
			item_map.erase(it); // first erase from map, because scene->erase will cause a call to this function
			delete classif;
		}
	}

	void classification_action()
	{
		dock_widget->show();
		dock_widget->raise();
		if (Scene_surface_mesh_item * mesh_item
			= qobject_cast<Scene_surface_mesh_item*>(scene->item(scene->mainSelectionIndex())))
		{
			create_from_item(mesh_item);
			QAction* sm_selection = mw->findChild<QAction*>("actionSelection");
			if (sm_selection)
				sm_selection->trigger();
			else
				print_message("Warning: can't find Surface Mesh Selection plugin");

			//********************Weixiao Update************************//
			CGAL::Three::Three::SetdefaultSurfaceMeshRenderingMode(TextureModePlusFlatEdges);
			mesh_item->setRenderingMode(TextureModePlusFlatEdges);
			//CGAL::Three::Three::information(QString("Reset the default rendering mode to TextureModePlusFlatEdges"));
			//**********************************************************//
		}

		//on_help_clicked();
	}

	void ask_for_closing()
	{
		QMessageBox oknotok;
		oknotok.setWindowTitle("Closing classification");
		oknotok.setText("All computed data structures will be discarded.\nColored display will be reinitialized.\nLabels and training information will remain in the classified items.\n\nAre you sure you want to close?");
		oknotok.setStandardButtons(QMessageBox::Yes);
		oknotok.addButton(QMessageBox::No);
		oknotok.setDefaultButton(QMessageBox::Yes);

		if (oknotok.exec() == QMessageBox::Yes)
			close_classification();
	}

	void close_classification()
	{
		for (Item_map::iterator it = item_map.begin(); it != item_map.end(); ++it)
		{
			Item_classification_base* classif = it->second;
			item_changed(classif->item());
			delete classif;
		}
		item_map.clear();
		dock_widget->hide();
	}

	void item_changed(Scene_item* item)
	{
		scene->itemChanged(item);
		item->invalidateOpenGLBuffers();
	}

	void update_plugin(int i)
	{
		if (dock_widget->isVisible())
			update_plugin_from_item(get_classification(scene->item(i)));
	}

	void disable_everything()
	{

		ui_widget.view->setEnabled(false);
		ui_widget.frame->setEnabled(false);
	}

	void enable_computation()
	{
		ui_widget.view->setEnabled(true);
		ui_widget.frame->setEnabled(true);
	}

	void enable_classif()
	{
		ui_widget.frame->setEnabled(true);
	}

	void update_plugin_from_item(Item_classification_base* classif)
	{
		disable_everything();
		if (classif != NULL)
		{
			enable_computation();
			// Clear class labels
			for (std::size_t i = 0; i < label_buttons.size(); ++i)
			{
				ui_widget.labelGrid->removeWidget(label_buttons[i].color_button);
				label_buttons[i].color_button->deleteLater();
				label_buttons[i].menu->deleteLater();
				delete label_buttons[i].label2;
			}
			label_buttons.clear();

			// Add labels
			for (std::size_t i = 0; i < classif->number_of_labels(); ++i)
				add_new_label(LabelButton(dock_widget,
					classif->label(i)->name().c_str(),
					classif->label_color(i),
					get_shortcut(i, classif->label(i)->name().c_str())));
			add_label_button();

			int index = ui_widget.display->currentIndex();
			ui_widget.display->clear();
			ui_widget.display->addItem("Real colors");
			ui_widget.display->addItem("Editing");
			ui_widget.display->addItem("Unlabelled");
			ui_widget.display->setCurrentIndex(1);
			classif->fill_display_combo_box(ui_widget.display);

			//******************Weixiao**************************//
			if (!classif->can_show_probability())
			{
				int total_facet_num = classif->get_total_number_facets();
				ui_widget.lineEdit->setText(QString::number(total_facet_num));
				int unlabelled_num = classif->get_unlabelled_number_facets();
				ui_widget.lineEdit_2->setText(QString::number(unlabelled_num));
				float labeling_progress = 1.0f - float(unlabelled_num) / float(total_facet_num);
				labeling_progress = total_facet_num == unlabelled_num ? 0.0f : labeling_progress * 100.0f;
				ui_widget.progressBar->setValue(labeling_progress);
			}
			//**************************************************************//

			//********************Weixiao Update************************//
			Scene_polyhedron_selection_item* selection_item
				= qobject_cast<Scene_polyhedron_selection_item*>(scene->item(scene->mainSelectionIndex()));
			if (selection_item && selection_item->polyhedron_item()->label_selection_combox_tmp != NULL)
			{
				selection_item->polyhedron_item()->fill_classes_combo_box(selection_item->polyhedron_item()->label_selection_combox_tmp);
				selection_item->polyhedron_item()->update_labels_for_selection();
			}
			//**********************************************************//
		}
	}

	Item_classification_base* get_classification(Scene_item* item = NULL)
	{
		if (!item)
			item = scene->item(scene->mainSelectionIndex());

		if (!item)
			return NULL;

		Scene_polyhedron_selection_item* selection_item
			= qobject_cast<Scene_polyhedron_selection_item*>(item);
		if (selection_item)
			item = selection_item->polyhedron_item();

		Item_map::iterator it = item_map.find(item);

		if (it != item_map.end())
		{
			if (selection_item)
				dynamic_cast<Surface_mesh_item_classification*>(it->second)->set_selection_item(selection_item);
			return it->second;
		}

		return NULL;
	}

	Item_classification_base* create_from_item(Scene_surface_mesh_item* mesh_item)
	{
		if (item_map.find(mesh_item) != item_map.end())
			return item_map[mesh_item];

		QApplication::setOverrideCursor(Qt::WaitCursor);
		Item_classification_base* classif
			= new Surface_mesh_item_classification(mesh_item);
		item_map.insert(std::make_pair(mesh_item, classif));
		QApplication::restoreOverrideCursor();
		update_plugin_from_item(classif);

		/************Weixiao Update************/
		//if (!classif->can_show_probability())
		//	print_message("Input data do not have probability or with invalid probability, \
		//	so the probability slider will not work. The good news is the progress bar will work perfectly!!!");
		//else
		//	print_message("Input data have probability and predict labels, so the labeling progress bar will not work. \
		//	The good news is the probability slider will work perfectly!!!");
		/**************************************/

		return classif;
	}

	void on_help_clicked()
	{
		QMessageBox::information(dock_widget, QString("Urban Mesh Annotator"),
			QString("Urban Mesh Annotator\n"
				"\n"
				"Welcome to Urban Mesh Annotator! Please read carefully this notice\n"
				"before using the plugin.\n"
				"\n"
				"[QUICK INTRODUCTION]\n"
				"Firstly, you need to load the mesh data (*.ply) and start the [3D Annotation] in [Operations] menu.\n"
				"Then you need to click the 'selection' layer in 'Geometric Objects' to start annotation process. After you finished, do not forget to save your work.\n"
				"\n"
				"In order to annotate the data, you might need to use the following shortcuts:\n"
				"[Most used]\n"
				"Shift + Left Button: Selection\n"
				"Shift + Wheel(Forward/Backward): Expand/Reduce selected area\n"
				"Shift + D + Left Button: Deselection\n"
				"Shift + [Capital letter of labels in the bracket(?)] : Add selection to the label category\n"
				"Ctrl + 1: Switch on/off Texture rendering mode\n"
				"Ctrl + 2: Switch on/off Flat rendering mode\n"
				"Ctrl + Left Button: Update to the new view center\n"
				"Ctrl + S: Save data\n"
				"Z + Mouse Left Button: Zoom into the position\n"
				"[Additional used]\n"
				"Left Button Double Click: Set orthographic projection\n"
				"Shift + Right Button: Pop up context menu of current layer\n"
				"Ctrl + Space: Turn on/off current view\n"
				"Ctrl + R: Recenter the view center to original\n"
				"Ctrl + L: Load data\n"
				"Ctrl + F1: Save snapshot and camera parameters\n"
				"Ctrl + F2: Load camera parameter and move to the view\n"
				"D + Mouse Left Button 1st && 2nd: Measure euclidean distance between 1st and 2nd point\n"
			));
	}

	float display_button_value(QPushButton* button)
	{
		std::string text = button->text().toStdString();

		std::size_t pos1 = text.find('(');
		if (pos1 == std::string::npos)
			return std::numeric_limits<float>::infinity();
		std::size_t pos2 = text.find(')');
		if (pos2 == std::string::npos)
			return std::numeric_limits<float>::infinity();

		std::string fstring(text.begin() + pos1 + 1,
			text.begin() + pos2);

		return float(std::atof(fstring.c_str()));
	}

	void clear_labels(Item_classification_base* classif)
	{
		classif->clear_labels();
	}

	void add_new_label(Item_classification_base* classif, const std::string& name)
	{
		add_new_label(LabelButton(dock_widget,
			name.c_str(),
			QColor(64 + rand() % 192,
				64 + rand() % 192,
				64 + rand() % 192),
			get_shortcut(label_buttons.size(), name.c_str())));
		QColor color = classif->add_new_label(name.c_str());
		label_buttons.back().change_color(color);
	}

	void on_add_new_label_clicked()
	{
		Item_classification_base* classif
			= get_classification();
		if (!classif)
		{
			print_message("Error: there is no point set classification item!");
			return;
		}

		bool ok;
		QString name =
			QInputDialog::getText((QWidget*)mw,
				tr("Add new labels"), // dialog title
				tr("Names (separated by spaces):"), // field label
				QLineEdit::Normal,
				tr("label_%1").arg(label_buttons.size()),
				&ok);
		if (!ok)
			return;

		std::istringstream iss(name.toStdString());

		std::string n;
		while (iss >> n)
			add_new_label(classif, n);

		add_label_button();
		update_plugin_from_item(classif);
	}

	void on_clear_labels_clicked()
	{
		Item_classification_base* classif
			= get_classification();
		if (!classif)
		{
			print_message("Error: there is no point set classification item!");
			return;
		}

		if (classif->number_of_labels() != 0)
		{
			QMessageBox::StandardButton reply
				= QMessageBox::question(NULL, "Classification",
					"Current labels will be discarded. Continue?",
					QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);

			if (reply == QMessageBox::No)
				return;
		}
		clear_labels(classif);
		update_plugin_from_item(classif);
	}

	char get_shortcut(std::size_t position, const char* name)
	{
		std::set<char> used_letters;

		used_letters.insert('t'); // used for "train"
		used_letters.insert('s'); // used for "random select"
		for (std::size_t i = 0; i < label_buttons.size(); ++i)
			if (i != position)
				used_letters.insert(label_buttons[i].shortcut);

		std::size_t idx = 0;
		while (name[idx] != '\0')
		{
			if (std::isalpha(name[idx]) &&
				used_letters.find(std::tolower(name[idx])) == used_letters.end())
				return std::tolower(name[idx]);
			++idx;
		}

		char fallback = 'a';
		while (used_letters.find(fallback) != used_letters.end())
			++fallback;

		return fallback;
	}

	void add_new_label(const LabelButton& label_button)
	{
		label_buttons.push_back(label_button);
		int position = static_cast<int>(label_buttons.size()) - 1;

		int x = position / 3;
		int y = position % 3;

		ui_widget.labelGrid->addWidget(label_buttons.back().color_button, x, y);

		QAction* add_selection = label_buttons.back().menu->addAction("Add selection to training set");

		add_selection->setShortcut(Qt::SHIFT | (Qt::Key_A + (label_button.shortcut - 'a')));
		//    add_selection->setShortcut(Qt::Key_0 + label_buttons.size() - 1);

		connect(add_selection, SIGNAL(triggered()), this,
			SLOT(on_add_selection_to_training_set_clicked()));
		
		label_buttons.back().menu->addSeparator();

		QAction* change_color = label_buttons.back().menu->addAction("Change color");
		//***********************Weixiao Update menu name*********************//
		change_color->setDisabled(true);
		change_color->setVisible(false);
		//********************************************************************//
		connect(change_color, SIGNAL(triggered()), this,
			SLOT(on_color_changed_clicked()));

		QAction* change_name = label_buttons.back().menu->addAction("Change name");
		//***********************Weixiao Update menu name*********************//
		change_name->setDisabled(true);
		change_name->setVisible(false);
		//********************************************************************//
		connect(change_name, SIGNAL(triggered()), this,
			SLOT(on_name_changed_clicked()));
		
		label_buttons.back().menu->addSeparator();

		QAction* remove_label = label_buttons.back().menu->addAction("Remove label");
		//***********************Weixiao Update menu name*********************//
		remove_label->setDisabled(true);
		remove_label->setVisible(false);
		//********************************************************************//
		connect(remove_label, SIGNAL(triggered()), this,
			SLOT(on_remove_label_clicked()));
	}

	void add_label_button()
	{
		int position = static_cast<int>(label_buttons.size());
		int x = position / 3;
		int y = position % 3;

		label_button->setVisible(true);
		ui_widget.labelGrid->addWidget(label_button, x, y);
	}

	void on_remove_label_clicked()
	{
		Item_classification_base* classif
			= get_classification();
		if (!classif)
		{
			print_message("Error: there is no point set classification item!");
			return;
		}

		QPushButton* label_clicked = qobject_cast<QPushButton*>(QObject::sender()->parent()->parent());
		if (label_clicked == NULL)
			std::cerr << "Error" << std::endl;
		else
		{
			int index = ui_widget.labelGrid->indexOf(label_clicked);
			int row_index, column_index, row_span, column_span;
			ui_widget.labelGrid->getItemPosition(index, &row_index, &column_index, &row_span, &column_span);

			int position = row_index * 3 + column_index;

			classif->remove_label(position);

			ui_widget.labelGrid->removeWidget(label_buttons[position].color_button);
			label_buttons[position].color_button->deleteLater();
			label_buttons[position].menu->deleteLater();

			delete label_buttons[position].label2;

			if (label_buttons.size() > 1)
				for (int i = position + 1; i < static_cast<int>(label_buttons.size()); ++i)
				{
					int position = i - 1;
					int x = position / 3;
					int y = position % 3;

					ui_widget.labelGrid->addWidget(label_buttons[i].color_button, x, y);
				}

			label_buttons.erase(label_buttons.begin() + position);
			add_label_button();
		}
		update_plugin_from_item(classif);
		item_changed(classif->item());
	}

	void on_color_changed_clicked()
	{
		Item_classification_base* classif
			= get_classification();
		if (!classif)
		{
			print_message("Error: there is no point set classification item!");
			return;
		}

		QPushButton* label_clicked = qobject_cast<QPushButton*>(QObject::sender()->parent()->parent());
		if (label_clicked == NULL)
			std::cerr << "Error" << std::endl;
		else
		{
			int index = ui_widget.labelGrid->indexOf(label_clicked);
			int row_index, column_index, row_span, column_span;
			ui_widget.labelGrid->getItemPosition(index, &row_index, &column_index, &row_span, &column_span);

			int position = row_index * 3 + column_index;

			QColor color = label_buttons[position].color;
			color = QColorDialog::getColor(color, (QWidget*)mw, "Change color of label");

			if (!color.isValid())
				return;

			label_buttons[position].change_color(color);
			classif->change_label_color(position,
				color);
		}
		classif->update_color();
		item_changed(classif->item());
	}

	void on_name_changed_clicked()
	{
		Item_classification_base* classif
			= get_classification();
		if (!classif)
		{
			print_message("Error: there is no point set classification item!");
			return;
		}

		QPushButton* label_clicked = qobject_cast<QPushButton*>(QObject::sender()->parent()->parent());
		if (label_clicked == NULL)
			std::cerr << "Error" << std::endl;
		else
		{
			int index = ui_widget.labelGrid->indexOf(label_clicked);
			int row_index, column_index, row_span, column_span;
			ui_widget.labelGrid->getItemPosition(index, &row_index, &column_index, &row_span, &column_span);

			int position = row_index * 3 + column_index;

			bool ok;
			QString name =
				QInputDialog::getText((QWidget*)mw,
					tr("Change name of label"), // dialog title
					tr("New name:"), // field label
					QLineEdit::Normal,
					classif->label(position)->name().c_str(),
					&ok);

			if (!ok)
				return;

			classif->change_label_name(position, name.toStdString());
		}

		update_plugin_from_item(classif);
		classif->update_color();
		item_changed(classif->item());
	}

	void on_add_selection_to_training_set_clicked()
	{
		Item_classification_base* classif
			= get_classification();
		if (!classif)
		{
			print_message("Error: there is no point set classification item!");
			return;
		}

		QPushButton* label_clicked = qobject_cast<QPushButton*>(QObject::sender()->parent()->parent());
		if (label_clicked == NULL)
			std::cerr << "Error" << std::endl;
		else
		{
			int index = ui_widget.labelGrid->indexOf(label_clicked);
			int row_index, column_index, row_span, column_span;
			ui_widget.labelGrid->getItemPosition(index, &row_index, &column_index, &row_span, &column_span);

			int position = row_index * 3 + column_index;
			/*****************************Ziqian*******************************/
			if (!classif->segment_form()) {
				print_message("Error: can't put faces from different segment into one segment.");
				//return;
			}
			/******************************************************************/

			classif->add_selection_to_training_set(position);
			//*****************************Weixiao**********************//
			int total_facet_num = classif->get_total_number_facets();
			ui_widget.lineEdit->setText(QString::number(total_facet_num));
			int unlabelled_num = classif->get_unlabelled_number_facets();
			ui_widget.lineEdit_2->setText(QString::number(unlabelled_num));
			float labeling_progress = 1.0f - float(unlabelled_num) / float(total_facet_num);
			labeling_progress = total_facet_num == unlabelled_num ? 0.0f : labeling_progress * 100.0f;
			ui_widget.progressBar->setValue(labeling_progress);

			Scene_polyhedron_selection_item* selection_item
				= qobject_cast<Scene_polyhedron_selection_item*>(scene->item(scene->mainSelectionIndex()));
			if (selection_item)
				selection_item->polyhedron_item()->update_labels_for_selection();
			//**********************************************************//
			//**********************************************************//
		}
		item_changed(classif->item());
	}

	//***********************Ziqian*******************************//
	bool threshold_based_change_color(Item_classification_base* classif, int index, int threshold, bool below)
	{
		float vmin = std::numeric_limits<float>::infinity();
		float vmax = std::numeric_limits<float>::infinity();

		classif->threshold_based_change_color(index, threshold, below, &vmin, &vmax);

		//if (vmin == std::numeric_limits<float>::infinity() || vmax == std::numeric_limits<float>::infinity())
		//{
		//	ui_widget.minDisplay->setEnabled(false);
		//	ui_widget.minDisplay->setText("Min");
		//	ui_widget.maxDisplay->setEnabled(false);
		//	ui_widget.maxDisplay->setText("Max");
		//}
		//else
		//{
		//	ui_widget.minDisplay->setEnabled(true);
		//	ui_widget.minDisplay->setText(tr("Min (%1)").arg(vmin));
		//	ui_widget.maxDisplay->setEnabled(true);
		//	ui_widget.maxDisplay->setText(tr("Max (%1)").arg(vmax));
		//}

		item_changed(classif->item());
		return true;
	}

	void disable_probability_enable_progressbar()
	{
		//probability slider
		ui_widget.ProbSlider->setEnabled(false);
		ui_widget.ProbSpin->setEnabled(false);
		ui_widget.ProbSwitcher->setEnabled(false);
		ui_widget.label_2->setEnabled(false);
		ui_widget.label->setEnabled(false);

		ui_widget.ProbSlider->setVisible(false);
		ui_widget.ProbSpin->setVisible(false);
		ui_widget.ProbSwitcher->setVisible(false);
		ui_widget.label_2->setVisible(false);
		ui_widget.label->setVisible(false);
		//progress bar
		ui_widget.label_3->setEnabled(true);
		ui_widget.label_4->setEnabled(true);
		ui_widget.label_5->setEnabled(true);
		ui_widget.lineEdit->setEnabled(true);
		ui_widget.lineEdit_2->setEnabled(true);
		ui_widget.progressBar->setEnabled(true);

		ui_widget.label_3->setVisible(true);
		ui_widget.label_4->setVisible(true);
		ui_widget.label_5->setVisible(true);
		ui_widget.lineEdit->setVisible(true);
		ui_widget.lineEdit_2->setVisible(true);
		ui_widget.progressBar->setVisible(true);
	}

	void enable_probability_disable_progressbar()
	{
		//probability slider
		ui_widget.ProbSlider->setEnabled(true);
		ui_widget.ProbSpin->setEnabled(true);
		ui_widget.ProbSwitcher->setEnabled(true);
		ui_widget.label_2->setEnabled(true);
		ui_widget.label->setEnabled(true);

		ui_widget.ProbSlider->setVisible(true);
		ui_widget.ProbSpin->setVisible(true);
		ui_widget.ProbSwitcher->setVisible(true);
		ui_widget.label_2->setVisible(true);
		ui_widget.label->setVisible(true);
		//progress bar
		ui_widget.label_3->setEnabled(false);
		ui_widget.label_4->setEnabled(false);
		ui_widget.label_5->setEnabled(false);
		ui_widget.lineEdit->setEnabled(false);
		ui_widget.lineEdit_2->setEnabled(false);
		ui_widget.progressBar->setEnabled(false);

		ui_widget.label_3->setVisible(false);
		ui_widget.label_4->setVisible(false);
		ui_widget.label_5->setVisible(false);
		ui_widget.lineEdit->setVisible(false);
		ui_widget.lineEdit_2->setVisible(false);
		ui_widget.progressBar->setVisible(false);
	}

	void disable_probability_disable_progressbar()
	{
		//probability slider
		ui_widget.ProbSlider->setEnabled(false);
		ui_widget.ProbSpin->setEnabled(false);
		ui_widget.ProbSwitcher->setEnabled(false);
		ui_widget.label_2->setEnabled(false);
		ui_widget.label->setEnabled(false);
		//progress bar
		ui_widget.label_3->setEnabled(false);
		ui_widget.label_4->setEnabled(false);
		ui_widget.label_5->setEnabled(false);
		ui_widget.lineEdit->setEnabled(false);
		ui_widget.lineEdit_2->setEnabled(false);
		ui_widget.progressBar->setEnabled(false);
	}

	void on_display_button_clicked_with_probability(int index)
	{
		Item_classification_base* classif = get_classification();
		if (!classif) return;

		if (index == 0)
			disable_probability_disable_progressbar();
		else
		{
			if (!classif->can_show_probability())
				disable_probability_enable_progressbar();
			else
				enable_probability_disable_progressbar();
		}

		int threshold = ui_widget.ProbSlider->value();
		bool below;
		if (ui_widget.ProbSwitcher->currentIndex() == 0)
			below = true;
		else
			below = false;

		threshold_based_change_color(classif, index, threshold, below);
	}


	void on_probability_threshold_changed(int value) {
		Item_classification_base* classif = get_classification();
		if (!classif) return;
		int threshold = value;
		bool below;
		if (ui_widget.ProbSwitcher->currentIndex() == 0) below = true;
		else below = false;

		int index = ui_widget.display->currentIndex();

		threshold_based_change_color(classif, index, threshold, below);
		//print_message("probability_state_changed!");
	}

	void on_probability_switcher_changed(int value) {
		Item_classification_base* classif = get_classification();
		if (!classif) return;
		int threshold = ui_widget.ProbSlider->value();
		bool below;
		if (value == 0) below = true;
		else below = false;

		int index = ui_widget.display->currentIndex();

		threshold_based_change_color(classif, index, threshold, below);
		//print_message("probability_state_changed!");
	}
	//********************************************************************//
private:
	Messages_interface* messages;
	QAction* actionClassification;

	QDockWidget* dock_widget;

	std::vector<LabelButton> label_buttons;
	QPushButton* label_button;

	Ui::Classification ui_widget;

	QColor color_att;

	typedef std::map<Scene_item*, Item_classification_base*> Item_map;
	Item_map item_map;


}; // end Polyhedron_demo_classification_plugin

#include "Classification_plugin.moc"
