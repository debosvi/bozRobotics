
#include <iostream>

#include <QtCore/QCoreApplication>

#include <ros/ros.h>
#include <std_msgs/UInt16.h>
#include <subm_libs/qt/QRosSubscriber.hpp>

#include "testQRosSubscriber.hpp"

using namespace subm_libs::qt;

void testQRosSubscriber::onReceive(const std_msgs::UInt16ConstPtr& ui16) {
    std::cerr << __FUNCTION__ << std::endl;
}

QROS_SUBSCRIBER_DECLARE_METATYPE(std_msgs::UInt16);

int main(int ac, char** av) {
    const char *topic = "/mytopic";
    const char *node_name = "mytopic_subscriber";
    const uint32_t size=10;
    testQRosSubscriber test;

    QCoreApplication app(ac, av);
    QCoreApplication::setApplicationName(node_name);
    QCoreApplication::setApplicationVersion("1.0");
    

    ros::init(ac, av, node_name);
    ros::start();
    ros::AsyncSpinner spinner(1);
    
    QRosSubscriber<std_msgs::UInt16> sub(topic, size, &test, SLOT(onReceive(const std_msgs::UInt16ConstPtr&)));
    
    spinner.start();
    
    return app.exec();
}
