
#include <QtCore/QCoreApplication>

#include <subm_libs/qt/QRosNodeSingleton.hpp>

#include "test.h"

using namespace subm_libs::qt;

static const char* node_name="subm_visu";

int main(int argc, char** argv) {
    QCoreApplication app(argc, argv);
    QCoreApplication::setApplicationName(node_name);
    QCoreApplication::setApplicationVersion("1.0");

    QRosNodeSingleton::instance(&app)->init(argc, argv, node_name);
    
    QObject::connect(
        QRosNodeSingleton::instance(), SIGNAL(rosShutdown()),
        &app, SLOT(quit())
        );
    
    subm_visu::TestRudder test(&app);
    
    test.init();
    test.start();        
    
    return app.exec();
}
  
