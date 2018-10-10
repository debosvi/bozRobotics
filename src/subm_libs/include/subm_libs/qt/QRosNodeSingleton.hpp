/**
 * \file        QRosNodeSingleton.hpp
 * \author      Vincent de RIBOU <v.debossoreillederibou@akka.eu>
 * \version     0.1
 * \date        30th July 2018
 * \copyright   BozRobotics.
 * \brief       Definition of public QRosNodeSingleton class.
 */

#pragma once

#include <subm_libs/qt/QRosNode.hpp>
#include <subm_libs/util/Singleton.hpp>

namespace subm_libs {
namespace qt {
    
class QRosNodeSingleton : public QRosNode, public Singleton<QRosNodeSingleton> {
Q_OBJECT

public:
    QRosNodeSingleton(QObject* parent = 0);

    virtual ~QRosNodeSingleton();
};

} // namespace qt
} // namespace subm_libs
