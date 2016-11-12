//==================================================
// DjiRosSubscriber.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Nov 11, 2016
//==================================================

#pragma once

#include <ros/ros.h>

#include <ait/common.h>
#include <ait/dji/dji_drone.h>

namespace ait
{
namespace dji
{

class RosDjiDrone
{
public:
    RosDjiDrone() {
        dji_drone_ = new DJIDrone(node_handle_);
    }

    ~RosDjiDrone() {
        SAFE_DELETE(dji_drone_);
    }

    DJIDrone& operator()() {
        return *dji_drone_;
    }

    DJIDrone* operator->() {
        return dji_drone_;
    }

private:
    ros::NodeHandle node_handle_;
    DJIDrone* dji_drone_;
};

}
}
