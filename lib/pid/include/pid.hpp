#ifndef PID_HPP
#define PID_HPP

/*
 * Small PID controller class.
 *
 * Features:
 *  - output clamping to [outMin, outMax]
 *  - anti-windup (the integral term is clamped to the output range)
 *  - derivative taken on the measurement (no "kick" when the setpoint changes)
 *
 * Typical use:
 *      PID pid(2.0f, 0.5f, 1.0f, -100.0f, 100.0f);
 *      ...
 *      float out = pid.update(setpoint, measurement, dt);   // dt in seconds
 */
class PID {
public:
    // Gains must be >= 0 and outMin < outMax.
    PID(float kp, float ki, float kd, float outMin, float outMax);

    // Compute the next output. dt = seconds since the previous call (must be > 0).
    float update(float setpoint, float measurement, float dt);

    // Clear the internal state (integral and derivative memory). Call this when
    // the controller has been off/paused, or after a large jump in the setpoint.
    void reset();

    // Change settings while running.
    void setGains(float kp, float ki, float kd);
    void setOutputLimits(float outMin, float outMax);

    float lastOutput() const { return lastOutput_; }

private:
    float kp_, ki_, kd_;
    float outMin_, outMax_;

    float iTerm_ = 0.0f;            // accumulated integral contribution (already multiplied by ki)
    float prevMeasurement_ = 0.0f;  // for the derivative
    float lastOutput_ = 0.0f;
    bool  firstRun_ = true;         // true until the first update() call
};

#endif // PID_HPP