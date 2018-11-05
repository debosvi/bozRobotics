 
#include <ros/node_handle.h>
#include <pluginlib/class_list_macros.h>
#include <equipments/RudderNodelet.h>


PLUGINLIB_EXPORT_CLASS(equipments::RudderNodelet, nodelet::Nodelet)

namespace equipments {

void RudderNodelet::onInit() {
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
        ros::console::notifyLoggerLevelsChanged();
    }
    NODELET_DEBUG("Initializing nodelet...");

    _current = subm_nodelets::RudderReport::RDR_VALUE_INIT;
    _target = subm_nodelets::RudderReport::RDR_VALUE_INIT;
    _done = true;
    
    ros::NodeHandle pnh = getPrivateNodeHandle();
    _timer = pnh.createTimer(ros::Duration(0.01f), boost::bind(&RudderNodelet::timerCb, this, _1));
    _pub = pnh.advertise<subm_nodelets::RudderReport>("report", 10);
    _sub = pnh.subscribe<subm_nodelets::RudderCommand>("command", 10, boost::bind(&RudderNodelet::messageCb, this, _1));    
}

void RudderNodelet::messageCb(const subm_nodelets::RudderCommandConstPtr& message) {
    NODELET_DEBUG(__PRETTY_FUNCTION__);
    if(message->target<subm_nodelets::RudderReport::RDR_VALUE_MIN) {
        NODELET_ERROR("%s: target not reachable (%d>%d)", __PRETTY_FUNCTION__, message->target, subm_nodelets::RudderReport::RDR_VALUE_MIN);   
        return;
    }
    if(message->target>subm_nodelets::RudderReport::RDR_VALUE_MAX) {
        NODELET_ERROR("%s: target not reachable (%d>%d)", __PRETTY_FUNCTION__, message->target, subm_nodelets::RudderReport::RDR_VALUE_MAX);    
        return;
    }
    _target = (float)message->target;
    _t0 = ros::Time::now();
    _done = false;
    return;    
}

void RudderNodelet::timerCb(const ros::TimerEvent& event) {
//     NODELET_DEBUG("%s: values (c:%0.02f, t:%0.02f)", __PRETTY_FUNCTION__, _current, _target);
    
    // update current
    const float max = 360.0f/100.0f;
    float step = (float)(_target-_current);
    if(fabs(step)<0.1f)
        step=0.0f;
//     NODELET_DEBUG("%s: step brut(%0.02f)", __PRETTY_FUNCTION__, step);
    
    if(step<(-1.0f*max))
        step = (-1.0f*max);
    if(step>(1.0f*max))
        step = (1.0f*max);
    
//     NODELET_DEBUG("%s: step corr(%0.02f)", __PRETTY_FUNCTION__, step);
    {
        _current += step;
        if((fabs(step)<0.1f) && !_done) {
            _t1 = ros::Time::now();
            ros::Duration spent = _t1-_t0;
            NODELET_DEBUG("%s: time spent (%ds %03dms)", __PRETTY_FUNCTION__, (int)spent.sec, (int)(spent.nsec/1e6));
            _done = true;
        }
    }
    
    // send report
    subm_nodelets::RudderReport msg;
    msg.header.frame_id = "rudder";
    msg.header.stamp = ros::Time::now();
    msg.status = subm_nodelets::RudderReport::RDR_STATUS_OK;
    msg.current = (uint16_t)_current;
    msg.target = (uint16_t)_target;
    
    _pub.publish(msg);
}

} // namespace equipments
