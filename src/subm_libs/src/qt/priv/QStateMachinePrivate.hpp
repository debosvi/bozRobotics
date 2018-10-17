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
#include "priv/QStringTransition.hpp"

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
    void onStateExited();    
    
private:
    void addNewState(const QString& state, QAbstractState* p, const bool final=false);
    void addNewTransition(const QString& trans, QState* from, QAbstractState* to);    
    void onStateGeneric(const bool enter=true);    

private:
    QStateMachine *q_ptr;
    QSharedPointer<::QStateMachine> _machine;
    QMap<QString, QState*> _states;
    QList<QString> _transitions;
    QFinalState *_done;
};

} // namespace priv
} // namespace qt
} // namespace subm_libs
