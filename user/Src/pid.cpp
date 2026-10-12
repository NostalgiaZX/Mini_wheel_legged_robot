
#include "../Inc/pid.h"

PID::PID(float kp, float ki, float kd, float kf,
         float output_max, float integral_max,
         float d_filter_alpha)
    : kp_(kp), ki_(ki), kd_(kd), kf_(kf),
      output_max_(output_max),
      integral_(0.0f), integral_max_(integral_max),
      d_filter_alpha_(d_filter_alpha), d_filter_prev_(0.0f),
      prev_error_(0.0f), first_update_(true),
      output_(0.0f) {
}

void PID::setGains(float kp, float ki, float kd, float kf) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    kf_ = kf;
}

void PID::reset() {
    integral_ = 0.0f;
    prev_error_ = 0.0f;
    d_filter_prev_ = 0.0f;
    first_update_ = true;
    output_ = 0.0f;
}

float PID::update(float target, float measured, float dt) {
    float error = target - measured;

    integral_ += error * dt;
    if (integral_ > integral_max_) integral_ = integral_max_;
    if (integral_ < -integral_max_) integral_ = -integral_max_;

    float derivative = 0.0f;
    if (!first_update_ && dt > 0.0f) {
        derivative = (error - prev_error_) / dt;
    }
    prev_error_ = error;
    first_update_ = false;

    derivative = d_filter_alpha_ * derivative + (1.0f - d_filter_alpha_) * d_filter_prev_;
    d_filter_prev_ = derivative;

    output_ = kp_ * error + ki_ * integral_ + kd_ * derivative + kf_ * target;

    if (output_ > output_max_) output_ = output_max_;
    if (output_ < -output_max_) output_ = -output_max_;

    return output_;
}
