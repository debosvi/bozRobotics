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

    QObject::connect(d->_machine.data(), SIGNAL(started()), this, SIGNAL(started()));
    QObject::connect(d->_machine.data(), SIGNAL(finished()), this, SIGNAL(finished()));
}

bool QStateMachine::addState(const QStateIdentifier state, const bool first) {
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

bool QStateMachine::addTransition(const QTransitionIdentifier trans, const QStateIdentifier from, const QStateIdentifier to) {
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

bool QStateMachine::addTransitionToFinal(const QTransitionIdentifier trans, const QStateIdentifier from) {
    Q_D(QStateMachine);

    if(!d->_transitions.contains(trans)) {
        if(d->_states.contains(from) && !d->_done) {
            QState* sfrom = d->_states[from];
            d->_done = new QFinalState();
        
            d->addNewState(0, d->_done, true);            
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

void QStateMachine::postTransition(const QTransitionIdentifier event) {
    Q_D(QStateMachine);
    d->_machine->postEvent(new QLocalEvent(event));
}

}  // namespace qt
}  // namespace subm_libs
