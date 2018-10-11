
#include <iostream>

#include <QtCore/QCoreApplication>

#include <std_msgs/UInt16.h>
#include <subm_libs/qt/QRosNode.hpp>

#include "testQRosSubscriber.hpp"

void testQRosSubscriber::onReceive(const std_msgs::UInt16ConstPtr& ui16) {
    std::cerr << __FUNCTION__ << std::endl;
}

using namespace subm_libs::qt;

QROS_SUBSCRIBER_DECLARE_METATYPE(std_msgs::UInt16);

int main(int ac, char** av) {
    const char *topic = "/mytopic";
    const char *node_name = "mytopic_subscriber2";
    const uint32_t size=10;
    testQRosSubscriber test;

    QCoreApplication app(ac, av);
    QCoreApplication::setApplicationName(node_name);
    QCoreApplication::setApplicationVersion("1.0");
    
    QRosNode qrn;

    qrn.init(ac, av, node_name);
    
    qrn.subscribe<std_msgs::UInt16>(topic, size, &test, SLOT(onReceive(const std_msgs::UInt16ConstPtr&)));
    QObject::connect(&qrn, SIGNAL(rosShutdown()), &app, SLOT(quit()));
    
    return app.exec();
}
