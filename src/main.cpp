#include "UrbanMeshAnnotator.h"
#include <QSurfaceFormat>
#include <QOpenGLContext>
#include <clocale>


/*!
 * \brief Defines the entry point of the demo.
 * Creates the application and sets a main window.
 */
int main(int argc, char **argv) {
    QApplication::setAttribute(Qt::AA_ShareOpenGLContexts);
    QApplication::setAttribute(Qt::AA_UseHighDpiPixmaps);
#if (QT_VERSION >= QT_VERSION_CHECK(5, 6, 0))
    QApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
#endif

    QSurfaceFormat fmt;
    //fmt.setSamples(4); //the recenter will not work if setSamples
    fmt.setVersion(4, 3);
    fmt.setRenderableType(QSurfaceFormat::OpenGL);
    fmt.setProfile(QSurfaceFormat::CoreProfile);
    fmt.setOption(QSurfaceFormat::DebugContext);
    QSurfaceFormat::setDefaultFormat(fmt);

    //We set the locale to avoid any trouble with VTK
    std::setlocale(LC_ALL, "C");

    UrbanMeshAnnotator app(argc, argv,
                      "UrbanMeshAnnotator",
                      "UrbanMeshAnnotator");
    return app.try_exec();
}
