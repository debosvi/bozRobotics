/**
 * \file        QRosSubscriber.hpp
 * \author      Vincent de RIBOU <v.debossoreillederibou@akka.eu>
 * \version     0.1
 * \date        30th July 2018
 * \copyright   BozRobotics.
 * \brief       Definition of public QRosSubscriber class.
 */

#pragma once

#include <QtCore/QMetaObject> 
#include <QtCore/QMetaType> 
#include <QtCore/QMetaMethod> 

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <ros/xmlrpc_manager.h>
#endif

class QObject;

namespace subm_libs {
namespace qt {

template <typename T>
struct TypeString
{
    constexpr static const char* name = "";
};

#define QROS_SUBSCRIBER_DECLARE_METATYPE(Type) \
namespace subm_libs { \
namespace qt { \
template<> \
struct TypeString<Type> \
{ \
    constexpr static const char* name = #Type; \
}; \
} \
}

class QRosSubscriberBase
{
public:
    virtual ~QRosSubscriberBase() {}
};

template <typename T>
class QRosSubscriber : public QRosSubscriberBase
{
public:
  QRosSubscriber(const char* topic, uint32_t queue_size, QObject* receiver, const char* method) : receiver_(receiver)
  {
    qRegisterMetaType<T>(TypeString<T>::name);

    int method_index;
    switch(method[0]) {
      case '1':
        method_index = receiver->metaObject()->indexOfSlot(QMetaObject::normalizedSignature(++method));
        break;
      case '2':
        method_index = receiver->metaObject()->indexOfSignal(QMetaObject::normalizedSignature(++method));
        break;
      default:
        method_index = receiver->metaObject()->indexOfMethod(QMetaObject::normalizedSignature(method));
        break;
    }

    if ((method_index) != -1) {
      qt_slot_ = receiver->metaObject()->method(method_index);
      ros::NodeHandle nh;
      sub_ = nh.subscribe(topic, queue_size, &QRosSubscriber<T>::invoke, this);
    } else {
      ROS_WARN("QSubscriber: no such method '%s::%s'.\n", receiver->metaObject()->className(), QMetaObject::normalizedSignature(method).data());
    }
  }

  virtual ~QRosSubscriber()
  {
  }

  void invoke(T msg)
  {
    qt_slot_.invoke(receiver_, Qt::QueuedConnection, QArgument<T>(TypeString<T>::name, msg));
  }

private:
  QObject* receiver_;
  ros::Subscriber sub_;
  QMetaMethod qt_slot_;
};

} // namespace qt
} // namespace subm_libs
