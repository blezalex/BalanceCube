#pragma once

#include "global.h"
#include "guard.hpp"
#include "imu/imu.hpp"

class AngleGuard : public Guard {
public:
	AngleGuard(const IMU& imu, const Config_BalancingConfig* balance_settings)
		: imu_(imu), balance_settings_(balance_settings) {
	}

	bool CanStart() {
		return abs(imu_.angles[ANGLE_STEER]) < balance_settings_->max_start_angle && abs(imu_.angles[ANGLE_DRIVE]) < balance_settings_->max_start_angle;
	}

	bool MustStop() {
		return abs(imu_.angles[ANGLE_DRIVE]) > balance_settings_->shutoff_angle
						|| abs(imu_.angles[ANGLE_STEER]) > balance_settings_->shutoff_angle;
	}

private:
	const IMU& imu_;
	const Config_BalancingConfig* balance_settings_;

	DISALLOW_COPY_AND_ASSIGN(AngleGuard);
};
