/*
 * Copyright (c) 2020-2021 Queen's Knights Robotics Team <qkrt@engsoc.queensu.ca>
 *
 * This file is part of qkrt-mcb.
 *
 * qkrt-mcb is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * qkrt-mcb is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with qkrt-mcb.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef DRIVERS_HPP_
#define DRIVERS_HPP_

#include <tap/drivers.hpp>

#include "control/control_operator_interface.hpp"
#include "communication/vision_coprocessor.hpp"
#include "communication/logger/logger.hpp"

class Drivers : public tap::Drivers
{
    Drivers()
        : tap::Drivers()
        , controlOperatorInterface(remote)
        , logger(this)
        , visionCoprocessor(this)
       
    {
    }
    
    friend class DriversSingleton;
public:
    control::ControlOperatorInterface controlOperatorInterface;
    communication::logger::Logger logger;
    communication::VisionCoprocessor visionCoprocessor;

    bool m_kill_switch() {
        return true;
    } 
};

#endif  // DRIVERS_HPP_
