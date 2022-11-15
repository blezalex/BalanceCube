#pragma once
#include <math.h>
#include <limits>

#include "balanceController.hpp"
#include "global.h"
#include "imu/imu.hpp"
#include "io/genericOut.hpp"
#include "io/pwm_out.hpp"
#include "lpf.hpp"
#include "pid.hpp"
#include "stateTracker.hpp"

#include "cmsis_boot/stm32f10x.h"
#include "io/rx.h"
#include "stm_lib/inc/stm32f10x_gpio.h"
#include "stm_lib/inc/stm32f10x_rcc.h"
#include "stm_lib/inc/stm32f10x_tim.h"

#define MAX_DUTY 500

class MotorController {
 public:
  MotorController(PwmOut* pwm, GenericOut* dir) : pwm_(pwm), dir_(dir) {}

  void Set(float cmd) {
    dir_->setState(cmd > 0);

    int16_t duty = fabsf(cmd) * 500;
    if (duty > 500) {
      duty = 500;
    }

    pwm_->set(500 - duty);
  }

 private:
  PwmOut* pwm_;
  GenericOut* dir_;
};

class BoardController : public UpdateListener {
 public:
  BoardController(Config* settings, IMU& imu, GenericOut& status_led,
                  GenericOut& beeper, Guard** guards, int guards_count,
                  GenericOut& green_led, PwmOut* pwm1, GenericOut* dir1,
                  PwmOut* pwm2, GenericOut* dir2, PwmOut* pwm3, GenericOut* dir3)
      : settings_(settings),
        imu_(imu),
        state_(guards, guards_count),
        pitch_balancer_(settings_, &(settings_->angle_pid)),
        roll_balancer_(settings_, &(settings_->angle_pid)),
        yaw_pid_controler_(&(settings_->yaw_pid)),
        status_led_(status_led),
        beeper_(beeper),
        green_led_(green_led),
        m1_speed_lpf_(&settings->misc.throttle_rc),
        m2_speed_lpf_(&settings->misc.throttle_rc),
        m3_speed_lpf_(&settings->misc.throttle_rc),
        motor1_(pwm1, dir1),
        motor2_(pwm2, dir2),
				motor3_(pwm3, dir3),
        motor1_out_lpf_(&(settings_->balance_settings.output_lpf_rc)),
        motor2_out_lpf_(&(settings_->balance_settings.output_lpf_rc)),
				motor3_out_lpf_(&(settings_->balance_settings.output_lpf_rc)) {}


  // Main control loop. Runs at 1000hz Must finish in less than 1ms otherwise
  // controller will freeze.
  void processUpdate(const MpuUpdate& update);

  void Reset();

 public:
  float fwd;
  float right;

  Config* settings_;
  IMU& imu_;
  StateTracker state_;
  BalanceController pitch_balancer_;
  BalanceController roll_balancer_;
  PidController yaw_pid_controler_;

  GenericOut& status_led_;
  GenericOut& beeper_;

  GenericOut& green_led_;

  // These are used to estimate motor actual speed
  BiQuadLpf m1_speed_lpf_;
  BiQuadLpf m2_speed_lpf_;
  BiQuadLpf m3_speed_lpf_;

  MotorController motor1_;
  BiQuadLpf motor1_out_lpf_;

  MotorController motor2_;
  BiQuadLpf motor2_out_lpf_;

	MotorController motor3_;
  BiQuadLpf motor3_out_lpf_;

  float fwdTargetAngle_;
  float rightTargetAngle_;
};
