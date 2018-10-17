/**
 * \file        QStateMachine.cpp
 * \author      Vincent de RIBOU <v.debossoreillederibou@akka.eu>
 * \version     0.1
 * \date        17th October 2018
 * \copyright   BozRobotics.
 * \brief       Implementation of public QStateMachine class.
 */

#include "priv/QStateMachinePrivate.hpp"
#include "priv/QStringTransition.hpp"

namespace subm_libs {
namespace qt {

QStateMachine::QStateMachine(QObject* parent) : QObject(parent), d_ptr(new QStateMachinePrivate(this, parent)) {}


QStateMachine::QStateMachine(QStateMachinePrivate&dd, QObject *parent) : QObject(parent), d_ptr(&dd) {}

QStateMachine::~QStateMachine() {}

void QStateMachine::init() {
    Q_D(QStateMachine);
    
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
}


void QStateMachine::start() {
    Q_D(QStateMachine);
    d->_machine->start();
}

void QStateMachine::postEvent(const QString& event) {
    Q_D(QStateMachine);
    d->_machine->postEvent(new QStringEvent(event));
}

}  // namespace qt
}  // namespace subm_libs
