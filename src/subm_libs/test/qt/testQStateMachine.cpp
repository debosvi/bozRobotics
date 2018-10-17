
#include <iostream>

#include <QtCore/QCoreApplication>
#include <QtCore/QTimer>

#include <subm_libs/qt/QStateMachine.hpp>


using namespace subm_libs::qt;

const char *node_name = "mytopic_subscriber2";

QStateMachine m;
int i=0;

const QStateIdentifier STATE_1 = 1;
const QStateIdentifier STATE_2 = 2;

const QTransitionIdentifier TRANSIT_1_2 = 0;
const QTransitionIdentifier TRANSIT_2_FINAL = 1;

void onStateEntered(QStateIdentifier state) {
    std::cerr << __PRETTY_FUNCTION__ << ": " << state << std::endl;
}

void onStateExited(QStateIdentifier state) {
    std::cerr << __PRETTY_FUNCTION__ << ": " << state << std::endl;
}

void onStarted() {
    std::cerr << __PRETTY_FUNCTION__ << std::endl;
}

void onFinished() {
    std::cerr << __PRETTY_FUNCTION__ << std::endl;
}

void onTimeout() {
    std::cerr << __PRETTY_FUNCTION__ << std::endl;
    if(!i) {
        std::cerr << "post tos2" << std::endl;
        m.postTransition(TRANSIT_1_2);
        QTimer::singleShot(500, &onTimeout);
        i++;
    }
    else {
        std::cerr << "post toDone" << std::endl;
        m.postTransition(TRANSIT_2_FINAL);
    }
}

int main(int ac, char** av) {
    QCoreApplication app(ac, av);
    QCoreApplication::setApplicationName(node_name);
    QCoreApplication::setApplicationVersion("1.0");
    
    m.init();
    
    if(m.addState(STATE_1, true)==false) {
        std::cerr << "unable to add state 1" << std::endl;
    }
    
    if(m.addState(STATE_2)==false) {
        std::cerr << "unable to add state 2" << std::endl;
    }
    
    if(m.addTransition(TRANSIT_1_2, STATE_1, STATE_2)==false) {
        std::cerr << "unable to add transition to state 2" << std::endl;
    }
    
    if(m.addTransitionToFinal(TRANSIT_2_FINAL, STATE_2)==false) {
        std::cerr << "unable to add transition to state done" << std::endl;
    }    
    
    QObject::connect(&m, &QStateMachine::started, &onStarted);
    QObject::connect(&m, &QStateMachine::finished, &onFinished);
    QObject::connect(&m, SIGNAL(finished()), &app, SLOT(quit()));
    
    m.start();
    
    QObject::connect(&m, &QStateMachine::enterState, &onStateEntered);
    QObject::connect(&m, &QStateMachine::exitState, &onStateExited);
    QTimer::singleShot(500, &onTimeout);
    
    return app.exec();
}
