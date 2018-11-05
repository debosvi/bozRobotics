
#include <nodelet/nodelet.h>
#include <ros/timer.h>

#include <subm_nodelets/RudderCommand.h>
#include <subm_nodelets/RudderReport.h>

namespace equipments {

class RudderNodelet : public nodelet::Nodelet {
public:
    virtual void onInit();

private:
    void timerCb(const ros::TimerEvent& event);
    void messageCb(const subm_nodelets::RudderCommandConstPtr& message);
    
private:
    ros::Timer _timer;
    ros::Publisher _pub;
    ros::Subscriber _sub;
    
    float _current;
    float _target;
    
    ros::Time _t0;
    ros::Time _t1;
    bool _done;
    
};

} // namespace equipments
