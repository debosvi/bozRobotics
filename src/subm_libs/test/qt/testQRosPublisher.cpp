
#include <ros/ros.h>
#include <std_msgs/UInt16.h>
#include "testQRosPublisher.hpp"

using namespace subm_libs::qt;

int main(int ac, char**av) {
    const char *topic = "/mytopic";
    const char *node = "mytopic_publisher";
    const uint32_t size=10;
    std_msgs::UInt16 ui16;
    
    
    ros::init(ac, av, node);
    ros::start();
    
    QRosPublisher<std_msgs::UInt16> pub(topic, size);
    ros::Publisher pubh = pub.publisher();;
    testQRosPublisher test(&pubh);
    
    ros::Rate rate(10);
    
    while(ros::ok()) {
        test.publish(ui16);
        ui16.data++;
        
        ros::spinOnce();
        rate.sleep();
    }    
    
    return 0;
}
