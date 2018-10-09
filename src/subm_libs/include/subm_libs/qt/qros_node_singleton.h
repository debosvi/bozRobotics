
#pragma once

#include <subm_libs/qt/qros_node.h>
#include <subm_libs/util/Singleton.h>

namespace adcc_libs {
namespace qt {
    
class QRosNodeSingleton : public QRosNode, public Singleton<QRosNodeSingleton> {
Q_OBJECT

public:
    QRosNodeSingleton(QObject* parent = 0);

    virtual ~QRosNodeSingleton();
};

} // namespace qt
} // namespace adcc_libs
