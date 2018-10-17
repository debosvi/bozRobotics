
#include <iostream>

#include <QtCore/QCoreApplication>
#include <QtCore/QTimer>

#include <subm_libs/qt/QStateMachine.hpp>
#include "testQStateMachine.hpp"

using namespace subm_libs::qt;

const char *node_name = "testQStateMachine";

MyComplexStateMachine::MyComplexStateMachine(QObject *parent) : QObject(parent), current_m(0) {}
MyComplexStateMachine::~MyComplexStateMachine() {}
    
void MyComplexStateMachine::init() {
    m1.init();
    
    if(m1.addState(STATE_1, true)==false) {
        std::cerr << "unable to add state 1" << std::endl;
    }
    
    if(m1.addState(STATE_2)==false) {
        std::cerr << "unable to add state 2" << std::endl;
    }
    
    if(m1.addTransition(TRANSIT_1_2, STATE_1, STATE_2)==false) {
        std::cerr << "unable to add transition to state 1" << std::endl;
    }
    
    if(m1.addTransitionToFinal(TRANSIT_2_FINAL, STATE_2)==false) {
        std::cerr << "unable to add transition to state 2" << std::endl;
    }    
    
    m2.init();
    
    if(m2.addState(STATE_3, true)==false) {
        std::cerr << "unable to add state 3" << std::endl;
    }
    
    if(m2.addState(STATE_4)==false) {
        std::cerr << "unable to add state 4" << std::endl;
    }
    
    if(m2.addTransition(TRANSIT_3_4, STATE_3, STATE_4)==false) {
        std::cerr << "unable to add transition to state 3" << std::endl;
    }
    
    if(m2.addTransitionToFinal(TRANSIT_4_FINAL, STATE_4)==false) {
        std::cerr << "unable to add transition to state 4" << std::endl;
    }    
    
    QObject::connect(&m1, SIGNAL(started()), this, SLOT(onStarted()));
    QObject::connect(&m2, SIGNAL(started()), this, SLOT(onStarted()));
    QObject::connect(&m1, SIGNAL(finished()), this, SLOT(onFinished()));
    QObject::connect(&m2, SIGNAL(finished()), this, SLOT(onFinished()));

    QObject::connect(&m1, SIGNAL(finished()), this, SLOT(onNextMachine()));
    QObject::connect(&m2, SIGNAL(finished()), this, SIGNAL(finished()));

    QObject::connect(&m1, SIGNAL(enterState(const QStateIdentifier)), this, SLOT(onStateEntered(const QStateIdentifier)));
    QObject::connect(&m2, SIGNAL(enterState(const QStateIdentifier)), this, SLOT(onStateEntered(const QStateIdentifier)));
    QObject::connect(&m1, SIGNAL(exitState(const QStateIdentifier)), this, SLOT(onStateExited(const QStateIdentifier)));
    QObject::connect(&m2, SIGNAL(exitState(const QStateIdentifier)), this, SLOT(onStateExited(const QStateIdentifier)));
    
}

void MyComplexStateMachine::start() {
    onNextMachine();
}

void MyComplexStateMachine::nextTransition(const int val) {
    std::cerr << __PRETTY_FUNCTION__ << std::endl;
    switch(val) {
    case 0:            
        std::cerr << "post event '1 to 2'" << std::endl;
        current_m->postTransition(TRANSIT_1_2);
        break;
    case 1:            
        std::cerr << "post event '2 to Final'" << std::endl;
        current_m->postTransition(TRANSIT_2_FINAL);
        break;
    case 2:            
        std::cerr << "post event '3 to 4'" << std::endl;
        current_m->postTransition(TRANSIT_3_4);
        break;
    case 3:            
        std::cerr << "post event '4 to Final'" << std::endl;
        current_m->postTransition(TRANSIT_4_FINAL);
        break;
        
    default: break;
    }
}

void MyComplexStateMachine::onStateEntered(QStateIdentifier state) {
    QObject* sender = QObject::sender();
    QString name="unknown";
    if(sender==&m1) name = "m1";
    else if(sender==&m2) name = "m2";
    std::cerr << __PRETTY_FUNCTION__ << ": " << name.toStdString() << ": " << state << std::endl;
}

void MyComplexStateMachine::onStateExited(QStateIdentifier state) {
    QObject* sender = QObject::sender();
    QString name="unknown";
    if(sender==&m1) name = "m1";
    else if(sender==&m2) name = "m2";
    std::cerr << __PRETTY_FUNCTION__ << ": " << name.toStdString() << ": " << state << std::endl;
}

void MyComplexStateMachine::onStarted() {
    std::cerr << __PRETTY_FUNCTION__ << std::endl;
}

void MyComplexStateMachine::onFinished() {
    std::cerr << __PRETTY_FUNCTION__ << std::endl;
}

void MyComplexStateMachine::onNextMachine() {
    std::cerr << __PRETTY_FUNCTION__ << std::endl;
    if(!current_m)
        current_m = &m1;
    else
        current_m = &m2;
        
    current_m->start();
}

static void onMainStarted() {
    std::cerr << __PRETTY_FUNCTION__ << std::endl;
}

static MyComplexStateMachine mcst;
static int i=0;

void onTimeout() {
    std::cerr << __PRETTY_FUNCTION__ << std::endl;
    mcst.nextTransition(i++);
    if(i<4)
        QTimer::singleShot(500, &onTimeout);
}

int main(int ac, char** av) {
    QCoreApplication app(ac, av);
    QCoreApplication::setApplicationName(node_name);
    QCoreApplication::setApplicationVersion("1.0");
        
    mcst.init();
    mcst.start();
    
    QObject::connect(&mcst, &MyComplexStateMachine::started, &onMainStarted);
    QObject::connect(&mcst, SIGNAL(finished()), &app, SLOT(quit()));
    QTimer::singleShot(500, &onTimeout);
    
    return app.exec();
}
