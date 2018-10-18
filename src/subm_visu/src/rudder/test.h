
#pragma once

#include <QtCore/QObject>

#include <ros/publisher.h>
#include "subm_visu/RudderMarker.hpp"

namespace subm_visu {

class TestRudder : public QObject {
    Q_OBJECT
    
public:
    explicit TestRudder(QObject* parent=0);
    virtual ~TestRudder();

    void init();    
    void start();
    
private Q_SLOTS:
    void onTimer();
    
private:
    QObject* _parent;
    ros::Publisher _pub;  
    subm_visu::RudderMarker _rudder;
    double _x;
    double _p;
    double _r;
    static const int _upt_ms;
};
    
} // ns subm_visu
