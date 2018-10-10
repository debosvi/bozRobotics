
#pragma once 

#include <QtCore/QObject>
#include <std_msgs/UInt16.h>

class testQRosSubscriber : public QObject {
    Q_OBJECT
    
public Q_SLOTS:
    void onReceive(const std_msgs::UInt16ConstPtr& ui16);
};
