/**
 * \file        NodeTransitionIdentifiers.hpp
 * \author      Vincent de RIBOU <v.debossoreillederibou@akka.eu>
 * \version     0.1
 * \date        18th October 2018
 * \copyright   BozRobotics.
 * \brief       Definition of public NodeTransitionIdentifiers class.
 */


#pragma once

#include <subm_libs/qt/QStateMachine.hpp>

using namespace subm_libs::qt;

namespace subm_nodes {
namespace states {

class NodeTransitionIdentifiers { 

public:
    static const QTransitionIdentifier GO_TO_SLEEP;
    static const QTransitionIdentifier GO_TO_STANDBY;
    static const QTransitionIdentifier GO_TO_WORKING;
    static const QTransitionIdentifier SWITCH_OFF;
    static const QTransitionIdentifier RESET;
 };

} // namespace qt
} // namespace subm_libs

// end of file
