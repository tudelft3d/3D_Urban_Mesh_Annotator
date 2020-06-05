#ifndef POLYHEDRON_DEMO_H
#define POLYHEDRON_DEMO_H

#include <QtCore/qglobal.h>

#include "Polyhedron_demo_config.h"

#include <QApplication>
#include <QScopedPointer>
#include <QStringList>

struct Polyhedron_demo_impl;

class POLYHEDRON_DEMO_EXPORT UrbanMeshAnnotator : public QApplication
{
  bool d_ptr_is_initialized; /// can be false during a call to `notify()`
  QScopedPointer<Polyhedron_demo_impl> d_ptr;
public:
  /*!
   * Constructor : calls the constructor of QApplication
   */
  UrbanMeshAnnotator(int& argc, char **argv,
                  QString application_name = "UrbanMeshAnnotator",
                  QString main_window_title = "UrbanMeshAnnotator",
                  QStringList input_keywords = QStringList());

  ~UrbanMeshAnnotator();

  /*!
   * Catches unhandled exceptions from all the widgets
   */
  bool notify(QObject* receiver, QEvent* event);

  /*! After a call to `do_not_catch_exceptions()`, unhandled exceptions are
      no longer caught
   */
  void do_not_catch_exceptions();

  /*! Call `QApplication::exec()` unless the main window is already closed
   */
  int try_exec();
}; // end class UrbanMeshAnnotator

#endif // POLYHEDRON_DEMO_H
