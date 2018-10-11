/**
 * \file        QRosNodeSingleton.cpp
 * \author      Vincent de RIBOU <v.debossoreillederibou@akka.eu>
 * \version     0.1
 * \date        30th July 2018
 * \copyright   BozRobotics.
 * \brief       Implementation of public QRosNodeSingleton class.
 */

#include <subm_libs/qt/QRosNodeSingleton.hpp>

#include <signal.h>
#include <sys/types.h>
#include <sys/socket.h>

namespace subm_libs {
namespace qt {

QRosNodeSingleton::QRosNodeSingleton(QObject* parent) : QRosNode(parent) {}

QRosNodeSingleton::~QRosNodeSingleton() {}

}  // namespace qt
}  // namespace subm_libs
