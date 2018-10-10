/**
 * \file        QSignalHandler.hpp
 * \author      Vincent de RIBOU <v.debossoreillederibou@akka.eu>
 * \version     0.1
 * \date        30th July 2018
 * \copyright   BozRobotics.
 * \brief       Definition of public QSignalHandler class.
 */

#pragma once

#include <QtCore/QMetaObject> 
#include <QtCore/QMetaType> 
#include <QtCore/QMetaMethod> 

class QSocketNotifier;

namespace subm_libs {
namespace qt {
    
class QSignalHandler : public QObject {
    Q_OBJECT

public:
    explicit QSignalHandler(QObject* parent = 0);

    virtual ~QSignalHandler();

Q_SIGNALS:
    void sigINT();
    void sigHUP();
    void sigTERM();

private:
  /* Signal handling members */
  static int sigFds[2];
  QSocketNotifier *sigIntNotifier;

  static void sigIntHandler(int sig_num);
};

} // namespace qt
} // namespace subm_libs
