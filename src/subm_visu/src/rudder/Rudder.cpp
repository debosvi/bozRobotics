/**
 * \file        Rudder.cpp
 * \author      Vincent de RIBOU <v.debossoreillederibou@akka.eu>
 * \version     0.1
 * \date        30th July 2018
 * \copyright   BozRobotics.
 * \brief       Implementation of public Rudder class.
 */

#include "subm_visu/Rudder.hpp"

namespace subm_visu {
    
Rudder::Rudder(const double width, const double height, const double depth) : 
    _w(width), _h(height), _d(depth) {
        
}

Rudder::~Rudder() {}

const double Rudder::width() const {
    return _w;
}

const double Rudder::height() const {
    return _h;
}

const double Rudder::depth() const {
    return _d;
}

}  // namespace subm_visu
