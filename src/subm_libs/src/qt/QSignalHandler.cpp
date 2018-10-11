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

#include <subm_libs/qt/QSignalHandler.hpp>

#include <signal.h>
#include <sys/types.h>
#include <sys/socket.h>

namespace subm_libs {
namespace qt {

/* Static members definitions */
int QSignalHandler::signalsFds[2];

QSignalHandler::QSignalHandler(QObject* parent) : QObject(parent) {
    /* Build a socket pair and a notifier to allow the SIGINT signal to be handled in the Qt context */
    socketpair(AF_UNIX, SOCK_STREAM, 0, signalsFds);
    signalsNotifier.reset(new QSocketNotifier(signalsFds[1], QSocketNotifier::Read, this));
    connect(signalsNotifier.data(), SIGNAL(activated(int)), this, SLOT(handleSignal()));
}

QSignalHandler::~QSignalHandler() {
}

void QSignalHandler::init() {
    if(signal(SIGINT, QSignalHandler::sigHandler) == SIG_ERR)
        std::cerr << "can't catch SIGINT" << std::endl;
    if(signal(SIGHUP, QSignalHandler::sigHandler) == SIG_ERR)
        std::cerr << "can't catch SIGHUP" << std::endl;
    if(signal(SIGTERM, QSignalHandler::sigHandler) == SIG_ERR)
        std::cerr << "can't catch SIGTERM" << std::endl;
   
}

void QSignalHandler::handleSignal() {
    // disable notifier
    signalsNotifier->setEnabled(false);

    char tmp;
    ssize_t r = ::read(signalsFds[1], &tmp, sizeof(tmp));
    
    switch(tmp) {
        case 2: Q_EMIT sigINT(); break;
        case 15: Q_EMIT sigTERM(); break;
        case 1: Q_EMIT sigHUP(); break;
        default: std::cerr << __PRETTY_FUNCTION__ << "not managed signal number (" << tmp << ")" << std::endl; break;        
    }

    // re enable notifier
    signalsNotifier->setEnabled(true);
}

void QSignalHandler::sigHandler(int sig_num) {
    char a = sig_num;
    ssize_t r = ::write(signalsFds[0], &a, sizeof(a));
}

}  // namespace qt
}  // namespace subm_libs
