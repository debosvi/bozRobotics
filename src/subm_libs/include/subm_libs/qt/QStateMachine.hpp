/**
 * \file        QStateMachine.hpp
 * \author      Vincent de RIBOU <v.debossoreillederibou@akka.eu>
 * \version     0.1
 * \date        17th October 2018
 * \copyright   BozRobotics.
 * \brief       Definition of public QStateMachine class.
 */

#pragma once

#include <QtCore/QObject>
#include <QtCore/QScopedPointer>

namespace subm_libs {
namespace qt {
    
namespace priv { 
class QStateMachinePrivate;
} // ns priv
 
using namespace priv;

typedef unsigned short QStateIdentifier;
typedef unsigned short QTransitionIdentifier;

class QStateMachine : public QObject {
    Q_OBJECT
    Q_DECLARE_PRIVATE(QStateMachine)
    Q_DISABLE_COPY(QStateMachine)

public:
    explicit QStateMachine(QObject* parent = 0);
    virtual ~QStateMachine();
    
    void init();
    void start();
    
    bool addState(const QStateIdentifier state, const bool first=false);
    bool addTransition(const QTransitionIdentifier trans, const QStateIdentifier from, const QStateIdentifier to);    
    bool addTransitionToFinal(const QTransitionIdentifier trans, const QStateIdentifier from);    
    
    void postTransition(const QTransitionIdentifier trans);
    
Q_SIGNALS:
    void enterState(const QStateIdentifier state);
    void exitState(const QStateIdentifier state);
    void started();
    void finished();
    
private:
    explicit QStateMachine(priv::QStateMachinePrivate&, QObject *parent);
    QScopedPointer<priv::QStateMachinePrivate> d_ptr;
};

} // namespace qt
} // namespace subm_libs
