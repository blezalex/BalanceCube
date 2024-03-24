#include "boardController.hpp"

float mapRcInput(uint16_t input) {
  if (input < MIN_MOTOR_CMD || input > MAX_MOTOR_CMD) {
    return 0;
  }

  return fmap(input, MIN_MOTOR_CMD, MAX_MOTOR_CMD, -1, 1);
}

void BoardController::Reset() {
  fwd_filter_.reset();
  right_filter_.reset();
  fwd_target_pid_.reset();
  right_target_pid_.reset();

  motor1_.Reset();
  motor2_.Reset();
  motor3_.Reset();
}

void BoardController::updateDecoders() {
  for (int i = 0; i < 3; ++i) {
    decoders_[i].Update();
  }
}

void BoardController::processUpdate(const MpuUpdate& update) {
  imu_.compute(update);
  State current_state = state_.update();

  // max speed observd 1683
  m1_speed_lpf_.compute(decoders_[0].GetPositionAndReset());
  m2_speed_lpf_.compute(decoders_[1].GetPositionAndReset());
  m3_speed_lpf_.compute(decoders_[2].GetPositionAndReset());

  // motor3_.Set(-100);

  switch (current_state) {
    case State::Stopped:
      Reset();
      status_led_.setState(0);
      beeper_.setState(0);
      break;

    case State::FirstIteration:
      Reset();

      pitch_balancer_.reset();
      roll_balancer_.reset();
      yaw_pid_controler_.reset();

      status_led_.setState(1);
      // intentional fall through
    case State::Starting:
    case State::Running:
      float fwdTargetAngle = 0;
      fwdTargetAngle += (1.0 / - sin(deg_to_rad(120))) * m1_speed_lpf_.getVal();
      fwdTargetAngle += (1.0 /   sin(deg_to_rad(120))) * m2_speed_lpf_.getVal();

      float rightTargetAngle = m3_speed_lpf_.getVal();
      rightTargetAngle += (1.0 / cos(deg_to_rad(120))) * (m1_speed_lpf_.getVal() + m2_speed_lpf_.getVal());

      fwdTargetAngle = fwd_filter_.compute(fwdTargetAngle);
      rightTargetAngle = right_filter_.compute(rightTargetAngle);

      fwdTargetAngle = -fwd_target_pid_.compute(fwdTargetAngle);
      rightTargetAngle = -right_target_pid_.compute(rightTargetAngle);

      fwdTargetAngle = constrain(fwdTargetAngle, -5, 5);
      rightTargetAngle = constrain(rightTargetAngle, -5, 5);

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

      float yaw_target = settings_->misc.yaw_target;
			float yaw = yaw_pid_controler_.compute(update.gyro[2] - yaw_target)  * state_.start_progress();
    
      // blue side of the cube is facing fwd
			const float pwm1 = yaw + cos(deg_to_rad(120)) * right - sin(deg_to_rad(120)) * fwd;
		  const float pwm2 = yaw + cos(deg_to_rad(120)) * right + sin(deg_to_rad(120)) * fwd;
      const float pwm3 = yaw + right;

      motor1_.Set(pwm1, m1_speed_lpf_.getVal());
      motor2_.Set(pwm2, m2_speed_lpf_.getVal());
      motor3_.Set(pwm3, m3_speed_lpf_.getVal());
      break;
  }
}