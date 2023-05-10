// robp_phidgets
#include <robp_phidgets/motors.h>

namespace robp::phidgets {
Motors::Motors(ros::NodeHandle& nh, ros::NodeHandle& nh_priv)
    : server_(nh_priv) {
  int port_left, port_right;
  if (!nh_priv.getParam("left_port", port_left)) {
    ROS_FATAL("Must specify 'left_port'");
    exit(1);
  }
  if (!nh_priv.getParam("right_port", port_right)) {
    ROS_FATAL("Must specify 'right_port'");
    exit(1);
  }

  uint32_t attach_timeout_ms =
      nh_priv.param("timeout", PHIDGET_TIMEOUT_DEFAULT);

  left_ = std::make_unique<Motor>(port_left, attach_timeout_ms,
                                  std::bind(&Motors::publish, this));
  right_ = std::make_unique<Motor>(port_right, attach_timeout_ms,
                                   std::bind(&Motors::publish, this));

  sub_ = nh_priv.subscribe("duty_cycles", 1, &Motors::dutyCyclesCallback, this);

  pub_ = nh_priv.advertise<robp_msgs::DutyCycles>("current_duty_cycles", 1);

  reset_failsafe_srv_ =
      nh_priv.advertiseService("reset_failsafe", &Motors::resetFailsafe, this);

  // Dynamic reconfigure
  f_ = boost::bind(&Motors::configCallback, this, _1, _2);
  server_.setCallback(f_);
}

Motors::~Motors() {}

void Motors::dutyCyclesCallback(robp_msgs::DutyCycles::ConstPtr const& msg) {
  if (!left_ || !right_) {
    return;
  }

  if (!failsafe_enabled_) {
    failsafe_enabled_ = true;
    left_->setFailsafe(failsafe_time_);
    right_->setFailsafe(failsafe_time_);
  }

  if (1 >= std::abs(msg->duty_cycle_left) &&
      1 >= std::abs(msg->duty_cycle_right)) {
    left_->setTargetVelocity(msg->duty_cycle_left);
    right_->setTargetVelocity(-msg->duty_cycle_right);
  } else {
    ROS_WARN(
        "Duty cycles (%f, %f) is out out of range ([-1, 1], [-1, 1]). Stopping "
        "motors!",
        msg->duty_cycle_left, msg->duty_cycle_right);
    left_->setTargetVelocity(0);
    right_->setTargetVelocity(0);
  }

  left_->resetFailsafe();
  right_->resetFailsafe();
}

void Motors::publish() {
  if (!left_ || !right_ || !left_->hasUpdate() || !right_->hasUpdate()) {
    return;
  }

  robp_msgs::DutyCycles::Ptr msg(new robp_msgs::DutyCycles);
  msg->header.stamp = ros::Time::now();
  msg->header.frame_id = "";

  msg->duty_cycle_left = left_->velocityUpdate();
  msg->duty_cycle_right = -right_->velocityUpdate();

  pub_.publish(msg);
}

bool Motors::resetFailsafe(std_srvs::Empty::Request&,
                           std_srvs::Empty::Response&) {
  if (left_) {
    left_->resetFailsafe();
  }
  if (right_) {
    right_->resetFailsafe();
  }
  return true;
}

void Motors::configCallback(robp_phidgets::MotorConfig& config,
                            uint32_t level) {
  left_->setAcceleration(config.acceleration);
  right_->setAcceleration(config.acceleration);

  left_->setTargetBrakingStrength(config.braking_strength);
  right_->setTargetBrakingStrength(config.braking_strength);

  left_->setCurrentLimit(config.current_limit);
  right_->setCurrentLimit(config.current_limit);

  left_->setDataRate(config.data_rate);
  right_->setDataRate(config.data_rate);

  failsafe_time_ = config.failsafe_timeout;
}
}  // namespace robp::phidgets