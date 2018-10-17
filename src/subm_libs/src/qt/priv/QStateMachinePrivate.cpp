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

QStateMachinePrivate::QStateMachinePrivate(QStateMachine *q, QObject* parent) : QObject(parent), q_ptr(q) {
    _machine.reset(new ::QStateMachine());
}

QStateMachinePrivate::~QStateMachinePrivate() {
}

void QStateMachinePrivate::onStateEntered() {
    Q_Q(QStateMachine);
    
    QObject *obj = this->sender();
    QState *sobj=(QState*)obj;
    
    std::cerr << __FUNCTION__ << std::endl;
    if(obj==s1)
        Q_EMIT q->stateChanged("Entering s1");
    if(obj==s2)
        Q_EMIT q->stateChanged("Entering s2");
    if(obj==done)
        Q_EMIT q->stateChanged("Entering done");
    
    
}

}  // namespace priv
}  // namespace qt
}  // namespace subm_libs
