
#include <iostream>
#include <std_msgs/UInt16.h>
#include <subm_libs/qt/QRosSubscriber.hpp>

#include "testQRosSubscriber.hpp"

using namespace subm_libs::qt;

void testQRosSubscriber::onReceive(const std_msgs::UInt16ConstPtr& ui16) {
    std::cerr << __FUNCTION__ << ui16->data << std::endl;
};

QROS_SUBSCRIBER_DECLARE_METATYPE(std_msgs::UInt16);

int main(int ac, char** av) {
    const char *topic = "/mytopic";
    const char *node = "mytopic_subscriber";
    const uint32_t size=10;
    testQRosSubscriber test;
     
    ros::init(ac, av, node);
    ros::start();
    
    QRosSubscriber<std_msgs::UInt16> sub(topic, size, &test, SLOT(onReceive(const std_msgs::UInt16ConstPtr&)));
    
    while(ros::ok()) {
        
        ros::spin();
    }    
    
    return 0;
}
