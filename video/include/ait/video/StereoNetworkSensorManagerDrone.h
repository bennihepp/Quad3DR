//==================================================
// StereoNetworkSensorManagerDrone.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Nov 11, 2016
//==================================================

#pragma once

#include <ait/dji/RosDjiDrone.h>
#include <ait/video/StereoNetworkSensorManager.h>

namespace ait
{
namespace video
{

template <typename TNetworkClient>
class StereoNetworkSensorManagerDrone : public StereoNetworkSensorManager<TNetworkClient, StereoFrameDroneUserData>
{
    using Base = StereoNetworkSensorManager<TNetworkClient, StereoFrameDroneUserData>;

public:
    StereoNetworkSensorManagerDrone(const StereoCalibration& stereo_calibration, const StereoClientType& client_type, const std::string& remote_ip, unsigned int remote_port)
        : Base(stereo_calibration, client_type, remote_ip, remote_port) {
    }

    ~StereoNetworkSensorManagerDrone() override {
    }

    void updateUserData(StereoFrameDroneUserData& user_data) override {
        user_data.local_position = drone_->local_position;
        std::cout << "local position: " << drone_->local_position << std::endl;
    }

protected:
    ait::dji::RosDjiDrone drone_;
};

}
}
