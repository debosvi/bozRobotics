/*
 * qros_node.cpp
 *
 *  Created on: 20 juin 2016
 *      Author: bonnetst
 */

#include <QtCore/QSocketNotifier>

#include <subm_libs/qt/qros_node.h>

#include <signal.h>
#include <sys/types.h>
#include <sys/socket.h>

namespace adcc_libs {
namespace qt {

/* Static members definitions */
int QRosNode::sigIntFd[2];

QRosNode::QRosNode(QObject* parent) : spinner_(0), QObject(parent) {
    /* Build a socket pair and a notifier to allow the SIGINT signal to be handled in the Qt context */
    socketpair(AF_UNIX, SOCK_STREAM, 0, sigIntFd);
    sigIntNotifier = new QSocketNotifier(sigIntFd[1], QSocketNotifier::Read, this);
    connect(sigIntNotifier, SIGNAL(activated(int)), this, SLOT(handleSigInt()));
}

QRosNode::~QRosNode() {
    shutdown();
    if (spinner_) {
        delete spinner_;
    }
}

void QRosNode::init(int argc, char* argv[], const std::string& name) {
  /* Make the ROS shutdown message also handle shutting down the Qt application in addition tp the node */
    ros::init(argc, argv, name);
    ros::XMLRPCManager::instance()->unbind("shutdown");
    ros::XMLRPCManager::instance()->bind("shutdown", boost::bind(&QRosNode::shutdownCallback, this, _1, _2));

    /* Start the node at this point */
    ros::start();

    /* Enable Qt SIGINT signal handling. By default, this is handled by ROS to shutdown the node but
    * here we also need to get the Qt application part to shutdown properly for the node to exit.
    */
    struct sigaction sigint;
    sigint.sa_handler = QRosNode::sigIntHandler;
    sigemptyset(&sigint.sa_mask);
    sigint.sa_flags |= SA_RESTART;
    sigaction(SIGINT, &sigint, 0);

    /* Create and start an asynchronous ROS spinner (in its own thread) */
    spinner_ = new ros::AsyncSpinner(1);
    spinner_->start();
}


void QRosNode::unsubscribe(QObject* receiver)
{
    subscribers_.erase(receiver);
}


const std::string& QRosNode::nodeName() const
{
    return ros::this_node::getName();
}


void QRosNode::shutdown()
{
    if (ros::isStarted()) {
        spinner_->stop();
        ros::shutdown();
        ros::waitForShutdown();
    }

    Q_EMIT(rosShutdown());
}


void QRosNode::handleSigInt()
{
    /* A SIGINT signal was caught, we are now in the Qt context: Qt signals can be emitted from here. */
    sigIntNotifier->setEnabled(false);

    char tmp;
    ssize_t r = ::read(sigIntFd[1], &tmp, sizeof(tmp));

    shutdown();

    sigIntNotifier->setEnabled(true);
}

void QRosNode::shutdownCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
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


void QRosNode::sigIntHandler(int sig_num)
{
  /* We are in the Unix signal context: no Qt calls are permitted.
   * Just send a character on the socket pair, this will wake-up the QSocketNotifier object which will then
   * emit a Qt signal to notify a new character has been received. The Qt signal handlers execute in the Qt
   * context where other Qt functions can be invoked.
   */
      char a = 1;
      ssize_t r = ::write(sigIntFd[0], &a, sizeof(a));
}

}  // namespace qt
}  // namespace adcc_libs
