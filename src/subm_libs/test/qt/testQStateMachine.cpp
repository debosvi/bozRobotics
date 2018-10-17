
#include <iostream>

#include <QtCore/QCoreApplication>
#include <QtCore/QTimer>

#include <subm_libs/qt/QStateMachine.hpp>


using namespace subm_libs::qt;

const char *node_name = "mytopic_subscriber2";

QStateMachine m;
int i=0;
    
void onStateEntered(QString name) {
    std::cerr << __PRETTY_FUNCTION__ << ": " << name.toStdString() << std::endl;
}

void onStateExited(QString name) {
    std::cerr << __PRETTY_FUNCTION__ << ": " << name.toStdString() << std::endl;
}

void onTimeout() {
    std::cerr << __PRETTY_FUNCTION__ << std::endl;
    if(!i) {
        std::cerr << "post tos2" << std::endl;
        m.postTransition("toS2");
        QTimer::singleShot(500, &onTimeout);
        i++;
    }
    else {
        std::cerr << "post toDone" << std::endl;
        m.postTransition("toDone");
    }
}

int main(int ac, char** av) {
    QCoreApplication app(ac, av);
    QCoreApplication::setApplicationName(node_name);
    QCoreApplication::setApplicationVersion("1.0");
    
    m.init();
    
    if(m.addState("s1", true)==false) {
        std::cerr << "unable to add state 1" << std::endl;
    }
    
    if(m.addState("s2")==false) {
        std::cerr << "unable to add state 2" << std::endl;
    }
    
    if(m.addTransition("toS2", "s1", "s2")==false) {
        std::cerr << "unable to add transition to state 2" << std::endl;
    }
    
    if(m.addTransitionToFinal("toDone", "s2")==false) {
        std::cerr << "unable to add transition to state done" << std::endl;
    }    
    
    QObject::connect(&m, SIGNAL(finished()), &app, SLOT(quit()));
    
    m.start();
    
    QObject::connect(&m, &QStateMachine::enterState, &onStateEntered);
    QObject::connect(&m, &QStateMachine::exitState, &onStateExited);
    QTimer::singleShot(500, &onTimeout);
    
    return app.exec();
}
