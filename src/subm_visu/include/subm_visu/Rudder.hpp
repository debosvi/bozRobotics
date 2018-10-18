/**
 * \file        Rudder.hpp
 * \author      Vincent de RIBOU <v.debossoreillederibou@akka.eu>
 * \version     0.1
 * \date        30th July 2018
 * \copyright   BozRobotics.
 * \brief       Definition of public Rudder class.
 */

#pragma once

namespace subm_visu {

class Rudder {
    
public:
    explicit Rudder(const double width=0.08f, const double height=0.08f, const double depth=0.01f);
    virtual ~Rudder();
    
    const double width() const;
    const double height() const;
    const double depth() const;

private:
    double _w, _h, _d;
};
    
} // namespace subm_visu;
