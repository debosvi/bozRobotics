/**
 * \file        qros_publisher.h
 * \author      Vincent de RIBOU <v.debossoreillederibou@akka.eu>
 * \version     0.1
 * \date        30th July 2018
 * \copyright   Renault SAS.
 * \brief       Definition of public qros_publisher class.
 */

#pragma once

class QObject;

namespace adcc_libs {
namespace qt {
    
class QRosPublisherBase
{
public:
    virtual ~QRosPublisherBase() {}
};

template <typename T>
class QRosPublisher : public QRosPublisherBase {

public:
    QRosPublisher(const std::string topic, uint32_t queue_size) {
        _pub = nh.advertise<T>(topic, queue_size);
    }

    virtual ~QRosPublisher() {}
    
    ros::Publisher& publisher() {
        return _pub;
    }

private:
    ros::NodeHandle nh;
    ros::Publisher _pub;
};

} // namespace qt
} // namespace adcc_libs
