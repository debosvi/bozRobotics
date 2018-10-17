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

class QStateMachine : public QObject {
    Q_OBJECT
    Q_DECLARE_PRIVATE(QStateMachine)
    Q_DISABLE_COPY(QStateMachine)

public:
    explicit QStateMachine(QObject* parent = 0);
    virtual ~QStateMachine();
    
    void init();
    void start();
    
    void postEvent(const QString& event);
    
Q_SIGNALS:
    void stateChanged(QString name);
    
private:
    explicit QStateMachine(priv::QStateMachinePrivate&, QObject *parent);
    QScopedPointer<priv::QStateMachinePrivate> d_ptr;
};

} // namespace qt
} // namespace subm_libs
