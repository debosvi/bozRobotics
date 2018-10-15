/**
 * \file        QSignalHandler.cpp
 * \author      Vincent de RIBOU <v.debossoreillederibou@akka.eu>
 * \version     0.1
 * \date        30th July 2018
 * \copyright   BozRobotics.
 * \brief       Implementation of public QSignalHandler class.
 */

#include <unistd.h> // read and write
#include <iostream> // std::cerr

#include <QtCore/QSocketNotifier>

#include "priv/QSignalHandlerPrivate.hpp"

#include <signal.h>
#include <sys/types.h>
#include <sys/socket.h>

namespace subm_libs {
namespace qt {

QSignalHandler::QSignalHandler(QObject* parent) : QObject(parent), d_ptr(new QSignalHandlerPrivate(this, parent)) {}


QSignalHandler::QSignalHandler(QSignalHandlerPrivate&dd, QObject *parent) : QObject(parent), d_ptr(&dd) {}

QSignalHandler::~QSignalHandler() {}

void QSignalHandler::init() {
//     Q_D(QSignalHandler);
    
    if(signal(SIGINT, QSignalHandlerPrivate::sigHandler) == SIG_ERR)
        std::cerr << "can't catch SIGINT" << std::endl;
    if(signal(SIGHUP, QSignalHandlerPrivate::sigHandler) == SIG_ERR)
        std::cerr << "can't catch SIGHUP" << std::endl;
    if(signal(SIGTERM, QSignalHandlerPrivate::sigHandler) == SIG_ERR)
        std::cerr << "can't catch SIGTERM" << std::endl;   
}

}  // namespace qt
}  // namespace subm_libs
