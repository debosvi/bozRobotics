
#pragma once 

#include <QtCore/QObject>
#include <std_msgs/UInt16.h>

#include <subm_libs/qt/QRosPublisher.hpp>

class testQRosPublisher : public QObject {
    Q_OBJECT
    
public:
    explicit testQRosPublisher(ros::Publisher* pub) : _pub(pub) {}
    
public Q_SLOTS:
    void publish(const std_msgs::UInt16& ui16) {
        _pub->publish(ui16);
    };
    
private:
    ros::Publisher* _pub;
};
