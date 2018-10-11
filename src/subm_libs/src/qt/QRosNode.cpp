/**
 * \file        QRosNode.cpp
 * \author      Vincent de RIBOU <v.debossoreillederibou@akka.eu>
 * \version     0.1
 * \date        30th July 2018
 * \copyright   BozRobotics.
 * \brief       Implementation of public QRosNode class.
 */

#include <QtCore/QSocketNotifier>

#include <subm_libs/qt/QRosNode.hpp>

#include <signal.h>
#include <sys/types.h>
#include <sys/socket.h>

namespace subm_libs {
namespace qt {

QRosNode::QRosNode(QObject* parent) : spinner_(0), QObject(parent) {
}

QRosNode::~QRosNode() {
    shutdown();
}

void QRosNode::init(int argc, char* argv[], const std::string& name) {
    /* Make the ROS shutdown message also handle shutting down the Qt application in addition tp the node */
    ros::init(argc, argv, name);
    ros::XMLRPCManager::instance()->unbind("shutdown");
    ros::XMLRPCManager::instance()->bind("shutdown", boost::bind(&QRosNode::shutdownCallback, this, _1, _2));

    /* Start the node at this point */
    ros::start();
    
    _sigs.init();
    QObject::connect(&_sigs, SIGNAL(sigINT()), this, SLOT(shutdown()));

    /* Create and start an asynchronous ROS spinner (in its own thread) */
    spinner_.reset(new ros::AsyncSpinner(1));
    spinner_->start();
}


void QRosNode::unsubscribe(QObject* receiver) {
    subscribers_.erase(receiver);
}


const std::string& QRosNode::nodeName() const {
    return ros::this_node::getName();
}


void QRosNode::shutdown() {
    if (ros::isStarted()) {
        spinner_->stop();
        ros::shutdown();
        ros::waitForShutdown();
    }
    Q_EMIT(rosShutdown());
}

void QRosNode::shutdownCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result) {
    int num_params = 0;
    if (params.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        num_params = params.size();
    }

    if (num_params > 1) {
        std::string reason = params[1];
        ROS_WARN("Shutdown request received. Reason: [%s]", reason.c_str());
        Q_EMIT rosShutdown();
    }

    result = ros::xmlrpc::responseInt(1, "", 0);
}

}  // namespace qt
}  // namespace subm_libs
