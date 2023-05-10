// robp_phidgets
#include <robp_phidgets/encoders.h>

// robp_msgs
#include <robp_msgs/Encoders.h>

namespace robp::phidgets {
Encoders::Encoders(ros::NodeHandle& nh, ros::NodeHandle& nh_priv)
    : nh_(nh), server_(nh_priv) {
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

  pub_ = nh_priv.advertise<robp_msgs::Encoders>("encoders", 1);

  left_ = std::make_unique<Encoder>(port_left, attach_timeout_ms,
                                    std::bind(&Encoders::publish, this));
  right_ = std::make_unique<Encoder>(port_right, attach_timeout_ms,
                                     std::bind(&Encoders::publish, this));

  // Dynamic reconfigure
  f_ = boost::bind(&Encoders::configCallback, this, _1, _2);
  server_.setCallback(f_);
}

Encoders::~Encoders() {}

void Encoders::publish() {
  if (!left_ || !right_ || !left_->hasChange() || !right_->hasChange()) {
    return;
  }

  robp_msgs::Encoders::Ptr msg(new robp_msgs::Encoders);
  msg->header.stamp = ros::Time::now();
  msg->header.frame_id = "";

  std::tie(msg->delta_encoder_left, msg->delta_time_left) = left_->change();
  std::tie(msg->delta_encoder_right, msg->delta_time_right) = right_->change();
  msg->encoder_left = left_->position();
  msg->delta_encoder_right = -msg->delta_encoder_right;
  msg->encoder_right = -right_->position();

  pub_.publish(msg);
}

void Encoders::configCallback(robp_phidgets::EncoderConfig& config,
                              uint32_t level) {
  left_->setDataRate(config.data_rate);
  right_->setDataRate(config.data_rate);

  left_->setPositionChangeTrigger(config.position_change_trigger);
  right_->setPositionChangeTrigger(config.position_change_trigger);
}
}  // namespace robp::phidgets