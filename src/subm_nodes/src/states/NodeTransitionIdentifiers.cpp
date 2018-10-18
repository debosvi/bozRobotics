/**
 * \file        NodeTransitionIdentifiers.hpp
 * \author      Vincent de RIBOU <v.debossoreillederibou@akka.eu>
 * \version     0.1
 * \date        18th October 2018
 * \copyright   BozRobotics.
 * \brief       Implementation of public NodeTransitionIdentifiers class.
 */

#include <subm_nodes/states/NodeTransitionIdentifiers.hpp>

namespace subm_nodes {
namespace states {

const QTransitionIdentifier NodeTransitionIdentifiers::GO_TO_SLEEP = 1;
const QTransitionIdentifier NodeTransitionIdentifiers::GO_TO_STANDBY = 2;
const QTransitionIdentifier NodeTransitionIdentifiers::GO_TO_WORKING = 3;
const QTransitionIdentifier NodeTransitionIdentifiers::SWITCH_OFF = 4;
const QTransitionIdentifier NodeTransitionIdentifiers::RESET = 5;

} // namespace qt
} // namespace subm_libs

// end of file
