
#include <QtCore/QTimer>
#include <QtCore/QDebug>

#include <ros/ros.h>

#include <tf/transform_datatypes.h>

#include <visualization_msgs/Marker.h>

#include "test.h"

namespace subm {
    
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
    
    visualization_msgs::Marker marker;
    marker.header.frame_id = "rudder";
    marker.header.stamp = ros::Time();
    marker.ns = "subm";
    marker.id = 0;
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::CUBE;
//         marker.mesh_resource = model_path;
    marker.mesh_use_embedded_materials = false;
    marker.pose.position.x = _x;
    marker.pose.position.y = 0.0f;
    marker.pose.position.z = 0.0f;
    double roll = _r * (M_PI / 180.0);
    double yaw = 0.0f * (M_PI / 180.0);
    double pitch = _p * (M_PI / 180.0);
    marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
    marker.color.r = 255.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.frame_locked = true;

    _pub.publish(marker);
    
    start();
    _x+=0.10f/_upt_ms;
    _p+=2.0f/_upt_ms;
    _r+=2.0f/_upt_ms;
    
}
    
}  // namespace subm
