/**
 * \file        QRosNode.hpp
 * \author      Vincent de RIBOU <v.debossoreillederibou@akka.eu>
 * \version     0.1
 * \date        30th July 2018
 * \copyright   BozRobotics.
 * \brief       Definition of public QRosNode class.
 */

#pragma once

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <ros/xmlrpc_manager.h>
#endif

#include <QtCore/QSharedPointer> 

#include <subm_libs/qt/QRosSubscriber.hpp>
#include <subm_libs/qt/QRosPublisher.hpp>
#include <subm_libs/qt/QSignalHandler.hpp>


namespace subm_libs {
namespace qt {
    
class QRosNode : public QObject {
    Q_OBJECT

public:
    QRosNode(QObject* parent = 0);

    virtual ~QRosNode();

    const std::string& nodeName() const;

    void init(int argc, char* argv[], const std::string& name);

    template<typename MessageType>
    void subscribe(const char* topic, uint32_t queue_size, QObject* receiver, const char* method);

    template<typename MessageType>
    void advertise(const std::string topic, uint32_t queue_size);
    
    template<typename MessageType>
    void publish(MessageType& msg);

public Q_SLOTS:
    void unsubscribe(QObject* receiver);
    void shutdown();

Q_SIGNALS:
    void rosShutdown();

private:
    QSignalHandler _sigs;
    QSharedPointer<ros::AsyncSpinner> spinner_;
    
    std::multimap<QObject*, boost::shared_ptr<QRosSubscriberBase> > subscribers_;
    std::multimap<std::string , boost::shared_ptr<QRosPublisherBase> > publishers_;

    void shutdownCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result);
};


} // namespace qt
} // namespace subm_libs

#include <subm_libs/qt/QRosNode.tpp>
