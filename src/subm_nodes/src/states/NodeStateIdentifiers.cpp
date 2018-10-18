/**
 * \file        NodeStateIdentifiers.hpp
 * \author      Vincent de RIBOU <v.debossoreillederibou@akka.eu>
 * \version     0.1
 * \date        18th October 2018
 * \copyright   BozRobotics.
 * \brief       Implementation of public NodeStateIdentifiers class.
 */

#include <subm_nodes/states/NodeStateIdentifiers.hpp>

namespace subm_nodes {
namespace states {

const QStateIdentifier NodeStateIdentifiers::STATE_SLEEPING = 1;
const QStateIdentifier NodeStateIdentifiers::STATE_STANDBY = 2;
const QStateIdentifier NodeStateIdentifiers::STATE_READY = 3;
const QStateIdentifier NodeStateIdentifiers::STATE_WORKING = 4;
const QStateIdentifier NodeStateIdentifiers::STATE_EXCEPTION = 5;
const QStateIdentifier NodeStateIdentifiers::STATE_SHUTDOWN = 6;

} // namespace qt
} // namespace subm_libs

// end of file
