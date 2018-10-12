
#pragma once

#include <QtCore/QObject>

#include <ros/publisher.h>

namespace subm {

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
    double _x;
    double _p;
    double _r;
    static const int _upt_ms;
};
    
} 
