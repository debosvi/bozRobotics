/**
 * \file        QStateMachinePrivate.cpp
 * \author      Vincent de RIBOU <v.debossoreillederibou@akka.eu>
 * \version     0.1
 * \date        17th October 2018
 * \copyright   BozRobotics.
 * \brief       Implementation of public QStateMachinePrivate class.
 */

#include <unistd.h> // read and write
#include <iostream> // std::cerr

#include <QtCore/QStateMachine>

#include "priv/QStateMachinePrivate.hpp"

namespace subm_libs {
namespace qt {
namespace priv {

QStateMachinePrivate::QStateMachinePrivate(QStateMachine *q, QObject* parent) : 
    QObject(parent), 
    q_ptr(q), _done(0) {
    _machine.reset(new ::QStateMachine());
}

QStateMachinePrivate::~QStateMachinePrivate() {
}

void QStateMachinePrivate::addNewState(const QString& state, QAbstractState* p, const bool final) {
    // add to _machine
    _machine->addState(p);

    if(!final) {
        // store in map
        _states[state] = (QState*)p;
    }

    // connects to signals
    connect(p, SIGNAL(entered()), this, SLOT(onStateEntered()));
    connect(p, SIGNAL(exited()), this, SLOT(onStateExited()));        
}

void QStateMachinePrivate::addNewTransition(const QString& trans, QState* from, QAbstractState* to) {
    // creates _transitions
    QStringTransition *t = new QStringTransition(trans);
    t->setTargetState(to);
    from->addTransition(t);            
    
    _transitions.push_back(trans);
}
    
void QStateMachinePrivate::onStateGeneric(const bool enter) {
    Q_Q(QStateMachine);
    
    bool emitted=false;
    QObject *obj = this->sender();
    QState *sobj=(QState*)obj;
    
    QMap<QString, QState*>::iterator it;
    
    for (it = _states.begin(); it != _states.end(); ++it) {
//         std::cerr << it.key().toStdString() << std::endl;
        if(it.value()==sobj) {
            if(enter) Q_EMIT q->enterState(it.key());            
            else Q_EMIT q->exitState(it.key());
            emitted=true;
            break;
        }
    }
    
    if(!emitted && (obj==_done) ) {
        if(enter) Q_EMIT q->enterState("done");            
        else Q_EMIT q->exitState("done");
    }
    
}

void QStateMachinePrivate::onStateEntered() {
    onStateGeneric(true);    
}

void QStateMachinePrivate::onStateExited() {
    onStateGeneric(false);    
}

}  // namespace priv
}  // namespace qt
}  // namespace subm_libs
