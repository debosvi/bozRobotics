
#include <subm_libs/qt/qros_node_singleton.h>

#include <signal.h>
#include <sys/types.h>
#include <sys/socket.h>

namespace adcc_libs {
namespace qt {

QRosNodeSingleton::QRosNodeSingleton(QObject* parent) : QRosNode(parent) {}

QRosNodeSingleton::~QRosNodeSingleton() {}

}  // namespace qt
}  // namespace adcc_libs
