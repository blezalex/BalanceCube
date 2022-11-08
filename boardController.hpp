#pragma once
#include <math.h>
#include <limits>

#include "drv/vesc/vesc.hpp"
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

class BoardController : public UpdateListener {
 public:
  BoardController(Config* settings, IMU& imu, GenericOut& status_led,
                  GenericOut& beeper, Guard** guards, int guards_count,
                  GenericOut& green_led, PwmOut* pwm1, GenericOut* dir1)
      : settings_(settings),
        imu_(imu),
        state_(guards, guards_count),
        pitch_balancer_(settings_, &(settings_->angle_pid)),
        roll_balancer_(settings_, &(settings_->angle_pid)),
        yaw_pid_controler_(&(settings_->yaw_pid)),
        status_led_(status_led),
        beeper_(beeper),
        green_led_(green_led),
        fwd_lpf_(&settings->misc.throttle_rc),
        right_lpf_(&settings->misc.throttle_rc),
        pwm1_(pwm1),
        dir1_(dir1),
        motor1_out_lpf_(&(settings_->balance_settings.output_lpf_rc)) {}

  float mapRcInput(uint16_t input) {
    if (input < MIN_MOTOR_CMD || input > MAX_MOTOR_CMD) {
      return 0;
    }

    return fmap(input, MIN_MOTOR_CMD, MAX_MOTOR_CMD, -1, 1);
  }

  // Main control loop. Runs at 1000hz Must finish in less than 1ms otherwise
  // controller will freeze.
  void processUpdate(const MpuUpdate& update) {
    imu_.compute(update);
    State current_state = state_.update();

    switch (current_state) {
      case State::Stopped:
        pwm1_->set(MAX_DUTY);
        motor1_out_lpf_.reset();

        status_led_.setState(0);
        beeper_.setState(0);
        break;

      case State::FirstIteration:
        pwm1_->set(MAX_DUTY);
        motor1_out_lpf_.reset();

        fwd_lpf_.reset();
        right_lpf_.reset();

        pitch_balancer_.reset();
        roll_balancer_.reset();
        yaw_pid_controler_.reset();

        status_led_.setState(1);
        // intentional fall through
      case State::Starting:
      case State::Running:

        // float fwdTargetAngle = mapRcInput(rxVals[1]) * 5;
        // float rightTargetAngle = mapRcInput(rxVals[0]) * 5;

        float avg_duty1 = fwd_lpf_.getVal();
        float rightTargetAngle = 0;
        float fwdTargetAngle = avg_duty1 * settings_->misc.stop_wheel_signal_p;

        if (current_state == State::Starting) {
          fwd = pitch_balancer_.computeStarting(imu_.angles[1] - fwdTargetAngle,
                                                update.gyro[1],
                                                state_.start_progress());
          right = roll_balancer_.computeStarting(
              imu_.angles[0] - rightTargetAngle, -update.gyro[0],
              state_.start_progress());
        } else {
          fwd = pitch_balancer_.compute(imu_.angles[1] - fwdTargetAngle,
                                        update.gyro[1]);
          right = roll_balancer_.compute(imu_.angles[0] - rightTargetAngle,
                                         -update.gyro[0]);
        }

        fwd_lpf_.compute(fwd);
        float duty1 = motor1_out_lpf_.compute(fwd);

        bool dyty1_fwd = duty1 > 0;
        dir1_->setState(dyty1_fwd);

        int16_t duty = fabsf(duty1) * 500;
        if (duty > 500) {
          duty = 500;
        }

        pwm1_->set(500 - duty);

        break;
    }
  }

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

  // These lpfs compensate for body inertia.
  BiQuadLpf fwd_lpf_;
  BiQuadLpf right_lpf_;

  PwmOut* pwm1_;
  GenericOut* dir1_;

  BiQuadLpf motor1_out_lpf_;

  int vesc_update_cycle_ctr_ = 0;
};
