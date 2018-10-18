/**
 * \file        NodeStateIdentifiers.hpp
 * \author      Vincent de RIBOU <v.debossoreillederibou@akka.eu>
 * \version     0.1
 * \date        18th October 2018
 * \copyright   BozRobotics.
 * \brief       Definition of public NodeStateIdentifiers class.
 */


#pragma once

#include <subm_libs/qt/QStateMachine.hpp>

using namespace subm_libs::qt;

namespace subm_nodes {
namespace states {

class NodeStateIdentifiers { 

public:
    static const QStateIdentifier STATE_SLEEPING;
    static const QStateIdentifier STATE_STANDBY;
    static const QStateIdentifier STATE_READY;
    static const QStateIdentifier STATE_WORKING;
    static const QStateIdentifier STATE_EXCEPTION;
    static const QStateIdentifier STATE_SHUTDOWN;
};

} // namespace qt
} // namespace subm_libs

// end of file
