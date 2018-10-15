/**
 * \file        QRosNode.tpp
 * \author      Vincent de RIBOU <v.debossoreillederibou@akka.eu>
 * \version     0.1
 * \date        30th July 2018
 * \copyright   BozRobotics.
 * \brief       implementation of public QRosNode template class.
 */

#pragma once

namespace subm_libs {
namespace qt {

// template functions implementation
template<typename MessageType>
void QRosNode::subscribe(const char* topic, uint32_t queue_size, QObject* receiver, const char* method)
{
    subscribers_.insert(std::make_pair(receiver, boost::make_shared<QRosSubscriber<MessageType>>(topic, queue_size, receiver, method)));
    if (subscribers_.count(receiver) == 1) {
        connect(receiver, SIGNAL(destroyed(QObject*)), SLOT(unsubscribe(QObject*)), Qt::ConnectionType::DirectConnection);
    }
}

template<typename MessageType>
void QRosNode::advertise(const std::string topic, uint32_t queue_size) {
    publishers_.insert(std::make_pair(::ros::message_traits::DataType<MessageType>::value(), boost::make_shared<QRosPublisher<MessageType>>(topic, queue_size)));
}

template<typename MessageType>
void QRosNode::publish(MessageType& msg) {
    std::multimap<std::string, boost::shared_ptr<QRosPublisherBase>>::iterator it = publishers_.find(::ros::message_traits::DataType<MessageType>::value());        
    if(it == publishers_.end()) {
        ROS_ERROR("You must advertise message before using publish method\n");
        return;
    }
    boost::shared_ptr<QRosPublisherBase> base =it->second;
    QRosPublisher<MessageType>* pub = (QRosPublisher<MessageType>*)(base.get());
    pub->publisher().publish(msg);
}

} // namespace qt
} // namespace subm_libs
