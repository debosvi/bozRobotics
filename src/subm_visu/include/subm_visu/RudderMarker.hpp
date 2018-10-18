/**
 * \file        RudderMarker.hpp
 * \author      Vincent de RIBOU <v.debossoreillederibou@akka.eu>
 * \version     0.1
 * \date        30th July 2018
 * \copyright   BozRobotics.
 * \brief       Definition of public Rudder class.
 */

#pragma once

#include <visualization_msgs/Marker.h>
#include "subm_visu/Rudder.hpp"

namespace subm_visu {

class RudderMarker : public Rudder {
    
public:
    explicit RudderMarker();
    virtual ~RudderMarker();
    
    const visualization_msgs::Marker marker() const;

private:
};
    
} // namespace subm_visu;
