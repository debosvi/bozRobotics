/**
 * \file        QStateMachine.cpp
 * \author      Vincent de RIBOU <v.debossoreillederibou@akka.eu>
 * \version     0.1
 * \date        17th October 2018
 * \copyright   BozRobotics.
 * \brief       Implementation of public QStateMachine class.
 */

#include "priv/QStateMachinePrivate.hpp"

namespace subm_libs {
namespace qt {

QStateMachine::QStateMachine(QObject* parent) : QObject(parent), d_ptr(new QStateMachinePrivate(this, parent)) {}


QStateMachine::QStateMachine(QStateMachinePrivate&dd, QObject *parent) : QObject(parent), d_ptr(&dd) {}

QStateMachine::~QStateMachine() {}

void QStateMachine::init() {
    Q_D(QStateMachine);
    
    d->_states.clear();
    d->_transitions.clear();
    
    
#if 0
    d->s1 = new QState();
    d->s2 = new QState();
    d->done = new QFinalState();

    QStringTransition *t1 = new QStringTransition("Hello");
    t1->setTargetState(d->s2);
    d->s1->addTransition(t1);
    
    QStringTransition *t2 = new QStringTransition("world");
    t2->setTargetState(d->done);
    d->s2->addTransition(t2);

    d->_machine->addState(d->s1);
    d->_machine->addState(d->s2);
    d->_machine->addState(d->done);
    d->_machine->setInitialState(d->s1);

    connect(d->s1, SIGNAL(entered()), d, SLOT(onStateEntered()));
    connect(d->s2, SIGNAL(entered()), d, SLOT(onStateEntered()));
    connect(d->done, SIGNAL(entered()), d, SLOT(onStateEntered()));    
    connect(d->s1, SIGNAL(exited()), d, SLOT(onStateExited()));
    connect(d->s2, SIGNAL(exited()), d, SLOT(onStateExited()));
    connect(d->done, SIGNAL(exited()), d, SLOT(onStateExited()));    
#endif
}

bool QStateMachine::addState(const QString& state, const bool first) {
    Q_D(QStateMachine);

    if(!d->_states.contains(state)) {
        // not already there, add it
        QState* s = new QState();

        d->addNewState(state, s);
        
        if(!d->_machine->initialState() && first) {
            d->_machine->setInitialState(s);
        }
        return true;
    }
    return false;
}

bool QStateMachine::addTransition(const QString& trans, const QString& from, const QString& to) {
    Q_D(QStateMachine);

    if(!d->_transitions.contains(trans)) {
        if( d->_states.contains(from) && d->_states.contains(to) ) {
            QState* sfrom = d->_states[from];
            QState* sto = d->_states[to];
            
            d->addNewTransition(trans, sfrom, sto);
            return true;
        }
        // at least one of the 2 states are missing
        return false;
    }
    
    // transition already there
    return false;    
}

bool QStateMachine::addTransitionToFinal(const QString& trans, const QString& from) {
    Q_D(QStateMachine);

    if(!d->_transitions.contains(trans)) {
        if(d->_states.contains(from) && !d->_done) {
            QState* sfrom = d->_states[from];
            d->_done = new QFinalState();
        
            d->addNewState("", d->_done, true);            
            d->addNewTransition(trans, sfrom, d->_done);
            return true;
        }
        // 'from' and/or 'done' states are missing
        return false;
    }
    
    // transition already there
    return false;    
}

void QStateMachine::start() {
    Q_D(QStateMachine);
    d->_machine->start();
}

void QStateMachine::postTransition(const QString& event) {
    Q_D(QStateMachine);
    d->_machine->postEvent(new QStringEvent(event));
}

}  // namespace qt
}  // namespace subm_libs
