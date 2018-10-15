/**
 * \file        QSignalHandler.hpp
 * \author      Vincent de RIBOU <v.debossoreillederibou@akka.eu>
 * \version     0.1
 * \date        30th July 2018
 * \copyright   BozRobotics.
 * \brief       Definition of public QSignalHandler class.
 */

#pragma once

#include <QtCore/QObject>
#include <QtCore/QScopedPointer>

namespace subm_libs {
namespace qt {
    
namespace priv { 
class QSignalHandlerPrivate;
} // ns priv
 
using namespace priv;

class QSignalHandler : public QObject {
    Q_OBJECT
    Q_DECLARE_PRIVATE(QSignalHandler)
    Q_DISABLE_COPY(QSignalHandler)

public:
    explicit QSignalHandler(QObject* parent = 0);
    virtual ~QSignalHandler();
    
    void init(void);
    
Q_SIGNALS:
    void sigINT();
    void sigHUP();
    void sigTERM();

private:
    explicit QSignalHandler(priv::QSignalHandlerPrivate&, QObject *parent);
    QScopedPointer<priv::QSignalHandlerPrivate> d_ptr;
};

} // namespace qt
} // namespace subm_libs
