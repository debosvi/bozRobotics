
#include <QtCore/QTimer>
#include <QtCore/QDebug>

#include <ros/ros.h>

#include <tf/transform_datatypes.h>

#include <visualization_msgs/Marker.h>

#include "test.h"

namespace subm_visu {
    
const int TestRudder::_upt_ms=50;

TestRudder::TestRudder(QObject* parent) : _parent(parent), _x(0.0f), _p(0.0f), _r(0.0f) {}

TestRudder::~TestRudder() {}

void TestRudder::init() {
    qDebug() << __PRETTY_FUNCTION__;
    
    ros::NodeHandle nh;
    _pub = nh.advertise<visualization_msgs::Marker>("/visu/rudder", 10);    
}
    
void TestRudder::start() {
    QTimer::singleShot(_upt_ms, this, SLOT(onTimer()));
}

void TestRudder::onTimer() {
//     qDebug() << __PRETTY_FUNCTION__;
    
    visualization_msgs::Marker marker = _rudder.marker();
    _pub.publish(marker);
    
    start();
    _x+=0.10f/_upt_ms;
    _p+=2.0f/_upt_ms;
    _r+=2.0f/_upt_ms;
    
}
    
}  // namespace subm_visu
