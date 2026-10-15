#include "pid.hpp"

namespace {
float clampf(float value, float lo, float hi)
{
    if (value < lo) return lo;
    if (value > hi) return hi;
    return value;
}
} // namespace

PID::PID(float kp, float ki, float kd, float outMin, float outMax)
    : kp_(kp), ki_(ki), kd_(kd), outMin_(outMin), outMax_(outMax)
{
}

void PID::reset()
{
    iTerm_ = 0.0f;
    lastOutput_ = 0.0f;
}

void PID::setGains(float kp, float ki, float kd)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void PID::setOutputLimits(float outMin, float outMax)
{
    outMin_ = outMin;
    outMax_ = outMax;
    iTerm_ = clampf(iTerm_, outMin_, outMax_);
    lastOutput_ = clampf(lastOutput_, outMin_, outMax_);
}

float PID::update(float setpoint, float measurement, float rate, float dt)
{
    if (dt <= 0.0f) {
        return lastOutput_;     // bad timing: keep the previous output
    }

    float error = setpoint - measurement;
    float p = kp_ * error;

    float d = -kd_ * rate;

    // Calculate output before updating the integral.
    float output = p + iTerm_ + d;

    // Integrate only if not saturated, or if the error
    // would help move the output away from saturation.
    if ((output > outMin_ && output < outMax_) ||
        (output >= outMax_ && ki_ * error < 0.0f) ||
        (output <= outMin_ && ki_ * error > 0.0f)) {
        iTerm_ += ki_ * error * dt;
    }

    iTerm_ = clampf(iTerm_, outMin_, outMax_);

    output = clampf(p + iTerm_ + d, outMin_, outMax_);

    lastOutput_ = output;
    return output;
}