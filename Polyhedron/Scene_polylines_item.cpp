#include "Scene_polylines_item.h"

#include <CGAL/bounding_box.h>
#include <CGAL/Three/Three.h>
#include <QMenu>
#include <QSlider>
#include <QWidgetAction>
#include <QAction>
#include <QInputDialog>
#include <QApplication>

struct Scene_polylines_item_private {
    typedef Scene_polylines_item::K K;
    typedef K::Point_3 Point_3;

    Scene_polylines_item_private(Scene_polylines_item *parent) :
        draw_extremities(false)
    {
      line_Slider = new QSlider(Qt::Horizontal);
      line_Slider->setValue(CGAL::Three::Three::getDefaultLinesWidth());
      line_Slider->setMaximum(2000);
      line_Slider->setMinimum(1);
      item = parent;
      invalidate_stats();
    }

    void invalidate_stats()
    {
      nb_vertices = 0;
      nb_edges = 0;
      min_length = std::numeric_limits<double>::max();
      max_length = 0;
      mean_length = 0;
      computed_stats = false;
    }

    enum VAOs {
        Edges=0,
        NbOfVaos
    };
    enum VBOs {
        Edges_Vertices = 0,
        NbOfVbos
    };

    mutable std::vector<float> positions_lines;
    mutable std::size_t nb_lines;
    typedef std::map<Point_3, int> Point_to_int_map;
    typedef Point_to_int_map::iterator iterator;
    void initializeBuffers(CGAL::Three::Viewer_interface *viewer) const;
    void computeElements() const;
    bool draw_extremities;
    Scene_polylines_item *item;
    mutable std::size_t nb_vertices;
    mutable std::size_t nb_edges;
    mutable double min_length;
    mutable double max_length;
    mutable double mean_length;
    mutable bool computed_stats;
    QSlider* line_Slider;
};


void
Scene_polylines_item_private::initializeBuffers(CGAL::Three::Viewer_interface *viewer = 0) const
{
  float lineWidth[2];
  if(!viewer->isOpenGL_4_3())
    viewer->glGetFloatv(GL_LINE_WIDTH_RANGE, lineWidth);
  else
  {
    lineWidth[0] = 0;
    lineWidth[1] = 10;
  }
  line_Slider->setMaximum(lineWidth[1]);
    QOpenGLShaderProgram *program;
   //vao for the lines
    {
      if(viewer->isOpenGL_4_3())
        program = item->getShaderProgram(Scene_polylines_item::PROGRAM_SOLID_WIREFRAME, viewer);
      else
        program = item->getShaderProgram(Scene_polylines_item::PROGRAM_NO_SELECTION, viewer);
      program->bind();
      
      item->vaos[Edges]->bind();
      item->buffers[Edges_Vertices].bind();
      item->buffers[Edges_Vertices].allocate(positions_lines.data(),
                                             static_cast<int>(positions_lines.size()*sizeof(float)));
      program->enableAttributeArray("vertex");
      program->setAttributeBuffer("vertex",GL_FLOAT,0,4);
      item->buffers[Edges_Vertices].release();
      item->vaos[Edges]->release();
      program->release();
      
      nb_lines = positions_lines.size();
      positions_lines.clear();
      positions_lines.swap(positions_lines);
    }
    item->are_buffers_filled = true;
}
void
Scene_polylines_item_private::computeElements() const
{
    const CGAL::qglviewer::Vec offset = static_cast<CGAL::Three::Viewer_interface*>(CGAL::QGLViewer::QGLViewerPool().first())->offset();
    QApplication::setOverrideCursor(Qt::WaitCursor);
    positions_lines.resize(0);
    double mean = 0;
    bool all_equal=true;
    //Fills the VBO with the lines
    for(std::list<std::vector<Point_3> >::const_iterator it = item->polylines.begin();
        it != item->polylines.end();
        ++it)
    {
      if(it->empty()) continue;
      if(it->front() == it->back())
        nb_vertices += it->size() - 1;
      else 
        nb_vertices += it->size();
      
      for(size_t i = 0, end = it->size()-1;
          i < end; ++i)
      {
            const Point_3& a = (*it)[i];
            const Point_3& b = (*it)[i+1];
            if(a!=b)
              all_equal = false;
            if(!computed_stats)
            {
              ++nb_edges;
                double length = CGAL::sqrt(
                      (a.x()-b.x()) * (a.x()-b.x()) +
                      (a.y()-b.y()) * (a.y()-b.y()) +
                      (a.z()-b.z()) * (a.z()-b.z()) );
                if(max_length < length)
                  max_length = length;
                if(min_length > length)
                  min_length = length;
                mean += length;
            }

            positions_lines.push_back(a.x()+offset.x);
            positions_lines.push_back(a.y()+offset.y);
            positions_lines.push_back(a.z()+offset.z);
            positions_lines.push_back(1.0);

            positions_lines.push_back(b.x()+offset.x);
            positions_lines.push_back(b.y()+offset.y);
            positions_lines.push_back(b.z()+offset.z);
            positions_lines.push_back(1.0);
        }

    }
    if(all_equal)
      item->setPointsMode();
    if(!computed_stats)
      mean_length = mean/double(nb_edges);
    computed_stats = true;
    QApplication::restoreOverrideCursor();
}

Scene_polylines_item::Scene_polylines_item() 
    :CGAL::Three::Scene_group_item("unnamed",Scene_polylines_item_private::NbOfVbos,Scene_polylines_item_private::NbOfVaos)
    ,d(new Scene_polylines_item_private(this))
{
    setRenderingMode(FlatPlusEdges);
    d->nb_lines = 0;
    invalidateOpenGLBuffers();

}

Scene_polylines_item::~Scene_polylines_item()
{
  delete d->line_Slider;
  delete d;

}

bool
Scene_polylines_item::isEmpty() const {
    return polylines.empty();
}

void
Scene_polylines_item::compute_bbox() const {
    typedef K::Iso_cuboid_3 Iso_cuboid_3;

    if(isEmpty())
    {
        _bbox =Bbox();
        return;
    }
    std::list<Point_3> boxes;
    for(std::list<std::vector<Point_3> >::const_iterator it = polylines.begin();
        it != polylines.end();
        ++it){
        if(it->begin() != it->end()) {
            Iso_cuboid_3 cub = CGAL::bounding_box(it->begin(), it->end());
            boxes.push_back((cub.min)());
            boxes.push_back((cub.max)());
        }
    }
    Iso_cuboid_3 bbox =
            boxes.begin() != boxes.end() ?
                CGAL::bounding_box(boxes.begin(), boxes.end()) :
                Iso_cuboid_3();

    _bbox = Bbox(bbox.xmin(),
                bbox.ymin(),
                bbox.zmin(),
                bbox.xmax(),
                bbox.ymax(),
                bbox.zmax());
}

Scene_item::Bbox Scene_polylines_item::bbox() const
{
  if(!is_bbox_computed)
      compute_bbox();
  is_bbox_computed = true;
  return _bbox;
}
Scene_polylines_item* 
Scene_polylines_item::clone() const {
    Scene_polylines_item* item = new Scene_polylines_item;
    item->polylines = polylines;
    QVariant metadata_variant = property("polylines metadata");
    if(metadata_variant.type() == QVariant::StringList)
    {
        item->setProperty("polylines metadata", metadata_variant);
    }
    return item;
}

QString
Scene_polylines_item::toolTip() const {
    QString s =
            tr("<p><b>%1</b> (mode: %2, color: %3)<br />"
               "<i>Polylines</i></p>"
               "<p>Number of polylines: %4</p>")
            .arg(this->name())
            .arg(this->renderingModeName())
            .arg(this->color().name())
            .arg(polylines.size());
    if(polylines.size() == 1 )
    {
      double length = 0;
      for(std::size_t i=1; i<polylines.front().size(); ++i)
      {
        K::Vector_3 vec(polylines.front()[i-1], polylines.front()[i]);
        length += CGAL::sqrt(vec.squared_length());
      }
      QString vertices_info = tr("<p>Number of vertices: %1<br />"
                                 "Polyline's length: %2</p>")
          .arg(polylines.front().size())
          .arg(length);
     s.append(vertices_info);

    }
    return s;
}

bool
Scene_polylines_item::supportsRenderingMode(RenderingMode m) const {
    return (m == Wireframe ||
            m == FlatPlusEdges ||
            m == Points);
}

// Shaded OpenGL drawing: only draw spheres
void
Scene_polylines_item::draw(CGAL::Three::Viewer_interface* viewer) const {
  if(!visible())
    return;
  if(!are_buffers_filled)
  {
    d->computeElements();
    d->initializeBuffers(viewer);
  }
  if(d->draw_extremities)
  {
    Scene_group_item::draw(viewer);
  }
}

// Wireframe OpenGL drawing
void 
Scene_polylines_item::drawEdges(CGAL::Three::Viewer_interface* viewer) const {
  if(!visible())
    return;
  if(renderingMode() == Wireframe
     || renderingMode() == FlatPlusEdges)
  {
    if(!are_buffers_filled)
    {
      d->computeElements();
      d->initializeBuffers(viewer);
    }
    
    vaos[Scene_polylines_item_private::Edges]->bind();
    if(!viewer->isOpenGL_4_3())
    {
      attribBuffers(viewer, PROGRAM_NO_SELECTION);
    }
    else
    {
      attribBuffers(viewer, PROGRAM_SOLID_WIREFRAME);
    }
    QOpenGLShaderProgram *program = viewer->isOpenGL_4_3() ? getShaderProgram(PROGRAM_SOLID_WIREFRAME)
                                                           : getShaderProgram(PROGRAM_NO_SELECTION);
    program->bind();
    if(viewer->isOpenGL_4_3())
    {
      QVector2D vp(viewer->width(), viewer->height());
      program->setUniformValue("viewport", vp);
      program->setUniformValue("near",(GLfloat)viewer->camera()->zNear());
      program->setUniformValue("far",(GLfloat)viewer->camera()->zFar());
      program->setUniformValue("width", GLfloat(d->line_Slider->value()));
    }
    program->setAttributeValue("colors", this->color());
    viewer->glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(d->nb_lines/4));
    program->release();
    vaos[Scene_polylines_item_private::Edges]->release();
  }
    if(d->draw_extremities)
    {
       Scene_group_item::drawEdges(viewer);
    }
}

void 
Scene_polylines_item::drawPoints(CGAL::Three::Viewer_interface* viewer) const {
  if(!visible())
    return;
  if(renderingMode() == Points)
  {
    if(!are_buffers_filled)
    {
      d->computeElements();
      d->initializeBuffers(viewer);
    }
    GLfloat point_size;
    viewer->glGetFloatv(GL_POINT_SIZE, &point_size);
    viewer->setGlPointSize(GLfloat(5));
    vaos[Scene_polylines_item_private::Edges]->bind();
    attribBuffers(viewer, PROGRAM_NO_SELECTION);
    QOpenGLShaderProgram *program = getShaderProgram(PROGRAM_NO_SELECTION);
    program->bind();
    QColor temp = this->color();
    program->setAttributeValue("colors", temp);
    viewer->glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(d->nb_lines/4));
    // Clean-up
    vaos[Scene_polylines_item_private::Edges]->release();
    program->release();
    viewer->setGlPointSize(point_size);
  }
   if(d->draw_extremities)
   {
      Scene_group_item::drawPoints(viewer);
   }
}

QMenu* Scene_polylines_item::contextMenu() 
{
    const char* prop_name = "Menu modified by Scene_polylines_item.";

    QMenu* menu = Scene_item::contextMenu();

    // Use dynamic properties:
    // http://doc.qt.io/qt-5/qobject.html#property
    bool menuChanged = menu->property(prop_name).toBool();

    if(!menuChanged) {
        menu->addSeparator();
        // TODO: add actions to display corners
        QAction* action = menu->addAction(tr("Display corners with radius..."));
        connect(action, SIGNAL(triggered()),
                this, SLOT(change_corner_radii()));

        QAction* actionSmoothPolylines =
                menu->addAction(tr("Smooth polylines"));
        actionSmoothPolylines->setObjectName("actionSmoothPolylines");
        connect(actionSmoothPolylines, SIGNAL(triggered()),this, SLOT(smooth()));

        QMenu *container = new QMenu(tr("Line Width"));
        QWidgetAction *sliderAction = new QWidgetAction(0);
        connect(d->line_Slider, &QSlider::valueChanged, this, &Scene_polylines_item::itemChanged);

        sliderAction->setDefaultWidget(d->line_Slider);

        container->addAction(sliderAction);
        menu->addMenu(container);

        menu->setProperty(prop_name, true);
    }
    return menu;
}

void Scene_polylines_item::invalidateOpenGLBuffers()
{
    are_buffers_filled = false;
    d->invalidate_stats();
    compute_bbox();


}

QString Scene_polylines_item::computeStats(int type)
{
  switch (type)
  {
  case NB_VERTICES:
    return QString::number(d->nb_vertices);
  case NB_EDGES:
    return QString::number(d->nb_edges);
  case MIN_LENGTH:
    return QString::number(d->min_length);
  case MAX_LENGTH:
    return QString::number(d->max_length);
  case MEAN_LENGTH:
    return QString::number(d->mean_length);
  default:
    return QString();
  }
}

CGAL::Three::Scene_item::Header_data Scene_polylines_item::header() const
{
  CGAL::Three::Scene_item::Header_data data;
  //categories
  data.categories.append(std::pair<QString,int>(QString("Properties"),5));


  //titles
  data.titles.append(QString("#Vertices"));
  data.titles.append(QString("#Segment Edges"));
  data.titles.append(QString("Shortest Segment Edge Length"));
  data.titles.append(QString("Longest Segment Edge Length"));
  data.titles.append(QString("Average Segment Edge Length"));
  return data;
}

void Scene_polylines_item::setWidth( int i)
{
  d->line_Slider->setValue(i);
  redraw();
}
