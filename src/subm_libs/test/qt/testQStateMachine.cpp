
#include <iostream>

#include <QtCore/QCoreApplication>
#include <QtCore/QTimer>

#include <subm_libs/qt/QStateMachine.hpp>


using namespace subm_libs::qt;

const char *node_name = "mytopic_subscriber2";

QStateMachine m;
int i=0;
    
void onStateChanged(QString name) {
    std::cerr << __FUNCTION__ << ": " << name.toStdString() << std::endl;
}

void onTimeout() {
    std::cerr << __FUNCTION__ << std::endl;
    if(!i) {
        m.postEvent("Hello");
        QTimer::singleShot(500, &onTimeout);
        i++;
    }
    else
        m.postEvent("world");
}

int main(int ac, char** av) {
    QCoreApplication app(ac, av);
    QCoreApplication::setApplicationName(node_name);
    QCoreApplication::setApplicationVersion("1.0");
    
    m.init();
    m.start();
    
    QObject::connect(&m, &QStateMachine::stateChanged, &onStateChanged);
    QTimer::singleShot(500, &onTimeout);
    
    return app.exec();
}
