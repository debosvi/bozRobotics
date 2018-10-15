/**
 * \file        QSignalHandlerPrivate.hpp
 * \author      Vincent de RIBOU <v.debossoreillederibou@akka.eu>
 * \version     0.1
 * \date        30th July 2018
 * \copyright   BozRobotics.
 * \brief       Definition of public QSignalHandlerPrivate class.
 */

#pragma once

#include <QtCore/QSharedPointer> 
#include <subm_libs/qt/QSignalHandler.hpp> 

class QSocketNotifier;

namespace subm_libs {
namespace qt {
namespace priv {
    
class QSignalHandlerPrivate : public QObject {
    Q_OBJECT
    Q_DECLARE_PUBLIC(QSignalHandler);
    
public:
    explicit QSignalHandlerPrivate(QSignalHandler *q, QObject* parent = 0);
    virtual ~QSignalHandlerPrivate();
    static void sigHandler(int sig_num);
    
private Q_SLOTS:
    void handleSignal();

private:
    QSignalHandler *q_ptr;
    /* Signal handling members */
    static int signalsFds[2];
    QSharedPointer<QSocketNotifier> signalsNotifier;

};

} // namespace priv
} // namespace qt
} // namespace subm_libs
