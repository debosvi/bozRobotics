/**
 * \file        QSignalHandlerPrivate.cpp
 * \author      Vincent de RIBOU <v.debossoreillederibou@akka.eu>
 * \version     0.1
 * \date        30th July 2018
 * \copyright   BozRobotics.
 * \brief       Implementation of public QSignalHandlerPrivate class.
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
namespace priv {

/* Static members definitions */
int QSignalHandlerPrivate::signalsFds[2];

QSignalHandlerPrivate::QSignalHandlerPrivate(QSignalHandler *q, QObject* parent) : QObject(parent), q_ptr(q) {
    /* Build a socket pair and a notifier to allow the SIGINT signal to be handled in the Qt context */
    socketpair(AF_UNIX, SOCK_STREAM, 0, signalsFds);
    signalsNotifier.reset(new QSocketNotifier(signalsFds[1], QSocketNotifier::Read, this));
    connect(signalsNotifier.data(), SIGNAL(activated(int)), this, SLOT(handleSignal()));
}

QSignalHandlerPrivate::~QSignalHandlerPrivate() {
}

void QSignalHandlerPrivate::handleSignal() {
    Q_Q(QSignalHandler);
    
    // disable notifier
    signalsNotifier->setEnabled(false);

    char tmp;
    ssize_t r = ::read(signalsFds[1], &tmp, sizeof(tmp));
    
    switch(tmp) {
        case 2: Q_EMIT q->sigINT(); break;
        case 15: Q_EMIT q->sigTERM(); break;
        case 1: Q_EMIT q->sigHUP(); break;
        default: std::cerr << __PRETTY_FUNCTION__ << "not managed signal number (" << tmp << ")" << std::endl; break;        
    }

    // re enable notifier
    signalsNotifier->setEnabled(true);
}

void QSignalHandlerPrivate::sigHandler(int sig_num) {
    char a = sig_num;
    ssize_t r = ::write(signalsFds[0], &a, sizeof(a));
}

}  // namespace priv
}  // namespace qt
}  // namespace subm_libs
