/**
 * \file        QStateMachinePrivate.hpp
 * \author      Vincent de RIBOU <v.debossoreillederibou@akka.eu>
 * \version     0.1
 * \date        17th October 2018
 * \copyright   BozRobotics.
 * \brief       Definition of public QStateMachinePrivate class.
 */

#pragma once

#include <QtCore/QSharedPointer> 
#include <QtCore/QStateMachine> 
#include <QtCore/QState>
#include <QtCore/QFinalState>

#include <subm_libs/qt/QStateMachine.hpp> 

namespace subm_libs {
namespace qt {
namespace priv {
    
class QStateMachinePrivate : public QObject {
    Q_OBJECT
    Q_DECLARE_PUBLIC(QStateMachine);
    friend subm_libs::qt::QStateMachine;
    
public:
    explicit QStateMachinePrivate(QStateMachine *q, QObject* parent = 0);
    virtual ~QStateMachinePrivate();
    
private Q_SLOTS:
    void onStateEntered();
    
    
private:
    QStateMachine *q_ptr;
    QSharedPointer<::QStateMachine> _machine;
    QState *s1;
    QState *s2;
    QFinalState *done;
};

} // namespace priv
} // namespace qt
} // namespace subm_libs
