#pragma once


#include "global.h"
#include "pid.hpp"
#include "lpf.hpp"


class BalanceController  {
public:
	BalanceController(const Config* settings, Config_PidConfig* pid_config) :
		settings_(settings), d_lpf_(&settings->balance_settings.balance_d_param_lpf_rc), angle_pid_(pid_config), rate_pid_(&settings->rate_pid) {
		reset();
	}

	void reset() {
		angle_pid_.reset();
		rate_pid_.reset();
		d_lpf_.reset();
		prev_error_ = 0;
	}



	float calcRatePid(float rateRequest,  float rate) {
		float error = rateRequest * 400 - rate;
		float d_term  = error - prev_error_;
		d_term = constrain(d_term, -settings_->balance_settings.balance_d_param_limiter, settings_->balance_settings.balance_d_param_limiter);
		prev_error_ = error;
		d_term = d_lpf_.compute(d_term);
		float result = rate_pid_.compute(error, d_term);

		return constrain(result, -1, 1);
	}

	// Compute torque needed while board in normal mode.
	// Returns torque request based on current imu and gyro readings. Range is [-1:1].
	float compute(float angle, float rate) {
		float rateRequest = angle_pid_.compute(angle);
		return calcRatePid(rateRequest, rate);
	}

	// Compute torque needed while board in starting up phase (coming from one side to balanced state).
	// Returns torque request based on current imu and gyro readings. Range is [-1:1].
	int16_t computeStarting(float angle, float rate, float pid_P_multiplier) {
		rate_pid_.resetI();
		angle_pid_.resetI();
		float rateRequest = angle_pid_.compute(angle);
		rateRequest *= pid_P_multiplier;
		return calcRatePid(rateRequest, rate);
	}

private:
	const Config* settings_;
	BiQuadLpf d_lpf_;
	PidController angle_pid_;
	PidController rate_pid_;
	float prev_error_;

};
