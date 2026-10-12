
#ifndef MINI_WHEEL_LEGGED_ROBOT_PID_H
#define MINI_WHEEL_LEGGED_ROBOT_PID_H

class PID {
public:
    PID(float kp, float ki, float kd, float kf,
        float output_max, float integral_max,
        float d_filter_alpha);

    void setGains(float kp, float ki, float kd, float kf = 0.0f);

    float update(float target, float measured, float dt);

    void reset();

    float output() const { return output_; }

private:
    float kp_;
    float ki_;
    float kd_;
    float kf_;

    float output_max_;

    float integral_;
    float integral_max_;

    float d_filter_alpha_;
    float d_filter_prev_;

    float prev_error_;
    bool first_update_;

    float output_;
};

#endif //MINI_WHEEL_LEGGED_ROBOT_PID_H
