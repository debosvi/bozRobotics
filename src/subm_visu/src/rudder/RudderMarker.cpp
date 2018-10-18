/**
 * \file        RudderMarkerMarker.cpp
 * \author      Vincent de RIBOU <v.debossoreillederibou@akka.eu>
 * \version     0.1
 * \date        30th July 2018
 * \copyright   BozRobotics.
 * \brief       Implementation of public RudderMarker class.
 */

#include <tf/transform_datatypes.h>

#include "subm_visu/RudderMarker.hpp"

namespace subm_visu {
    
RudderMarker::RudderMarker() {
        
}

RudderMarker::~RudderMarker() {}

const visualization_msgs::Marker RudderMarker::marker() const {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "rudder";
    marker.header.stamp = ros::Time();
    marker.ns = "subm";
    marker.id = 0;
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::CUBE;
//         marker.mesh_resource = model_path;
    marker.mesh_use_embedded_materials = false;
    marker.pose.position.x = 0.0f;
    marker.pose.position.y = 0.0f;
    marker.pose.position.z = 0.0f;
    double roll = 0.0f * (M_PI / 180.0);
    double yaw = 0.0f * (M_PI / 180.0);
    double pitch = 0.0f * (M_PI / 180.0);
    marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
    marker.color.r = 255.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.scale.x = width();
    marker.scale.y = height();
    marker.scale.z = depth();
    marker.frame_locked = true;
    
    return marker;
}

}  // namespace subm_visu
