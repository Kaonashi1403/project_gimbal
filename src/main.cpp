#include <stdio.h>
#include "pico/stdlib.h"
#include <math.h>
#include <stdint.h>
#include "hardware/pwm.h"
#include "hardware/clocks.h"

extern "C" {
#include "mpu.h"
}
#include "pid.hpp"

// ============================================================
//  CONFIGURATION (all tunable numbers live here)
// ============================================================

// ---- SPI wiring ----
#define SPI      spi0
#define SCK      18
#define MOSI     19
#define MISO     16
#define CS       17
#define BAUDRATE (1000 * 1000)

// ---- Servo ----
#define SERVO_PIN        15
#define SERVO_MIN_US     500       // pulse width at 0 deg
#define SERVO_MAX_US     2400      // pulse width at 180 deg
#define SERVO_CENTER     90.0f     // neutral angle (deg)
#define SERVO_MAX_DEG    180.0f
#define SERVO_PERIOD_US  20000     // 50 Hz servo frame
#define SERVO_TICK_HZ    1000000   // PWM counter ticks at 1 MHz -> 1 tick = 1 us
#define SERVO_SETTLE_MS  3000      // wait for servo to reach center at boot

// ---- PID (output limited so servo stays within 0..180) ----
#define KP 1.9f
#define KI 1.0f
#define KD 0.0f
#define PID_OUT_LIMIT 90.0f        // = SERVO_MAX_DEG / 2

// ---- Math ----
static constexpr float RAD_TO_DEG = 180.0f / (float)M_PI;

// ---- Startup calibration ----
static constexpr bool  CALIBRATE_ON_START   = false; // true: define "level" at boot
static constexpr int   CALIB_SAMPLES        = 100;
static constexpr int   CALIB_DELAY_MS       = 50;
static constexpr int   GYRO_BIAS_SAMPLES    = 500;
static constexpr int   GYRO_BIAS_DELAY_MS   = 2;

// ---- Main loop timing ----
static constexpr int   LOOP_PERIOD_MS       = 5;
static constexpr float DT_MAX_S             = 0.1f;   // clamp dt if the loop stalls
static constexpr int   PRINT_EVERY_N_LOOPS  = 10;

// ---- Complementary filter ----
// Time constant: for changes faster than this the gyro is trusted,
// for slower changes the accelerometer slowly pulls the angle back.
static constexpr float FILTER_TAU_S         = 0.25f;
// Accelerometer is only used when it measures ~1 g (i.e. only gravity)...
static constexpr float ACC_MAG_TOLERANCE_G  = 0.05f;
// ...and the board is not rotating fast (fast motion adds linear accel).
static constexpr float ACC_MAX_GYRO_DPS     = 20.0f;

// ---- Gyro bias tracking (runs only while almost stationary) ----
static constexpr float BIAS_TRACK_MAX_DPS   = 2.0f;
static constexpr float BIAS_TRACK_GAIN      = 0.001f;

// ============================================================
//  AXIS MAPPING
//  Lets you remount the sensor in any orientation by editing
//  g_orient only. "Logical X" is the axis we balance around.
// ============================================================
enum class Axis : uint8_t { X = 0, Y = 1, Z = 2 };

struct AxisMap {
    Axis   src;    // which physical axis feeds this logical axis
    int8_t sign;   // +1 or -1
};

struct Orientation { AxisMap x, y, z; };

static inline void remap3(const Orientation &o,
                          float px, float py, float pz,
                          float *lx, float *ly, float *lz)
{
    const float p[3] = { px, py, pz };
    if (lx) *lx = o.x.sign * p[static_cast<int>(o.x.src)];
    if (ly) *ly = o.y.sign * p[static_cast<int>(o.y.src)];
    if (lz) *lz = o.z.sign * p[static_cast<int>(o.z.src)];
}

/// THIS IS THE ONLY LINE YOU CHANGE TO RE-ORIENT THE SENSOR
constexpr Orientation ORIENT_DEFAULT = {
    { Axis::X, +1 }, { Axis::Y, +1 }, { Axis::Z, +1 }
};

constexpr Orientation ORIENT_RL = {
    { Axis::Y, +1 }, { Axis::Z, -1 }, { Axis::X, -1 }
};

// >>>>>>>>>>>> THE ONLY LINE YOU CHANGE TO RE-ORIENT <<<<<<<<<<<<
static Orientation g_orient = ORIENT_RL;

static void read_accel_mapped(const mpu_t *m, float *ax, float *ay, float *az)
{
    float px, py, pz;
    mpu_read_accel_g(m, &px, &py, &pz);
    remap3(g_orient, px, py, pz, ax, ay, az);
}

static void read_gyro_mapped(const mpu_t *m, float *gx, float *gy, float *gz)
{
    float px, py, pz;
    mpu_read_gyro_dps(m, &px, &py, &pz);
    remap3(g_orient, px, py, pz, gx, gy, gz);
}

// ============================================================
//  SERVO
// ============================================================
static void servo_init(uint pin)
{
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(pin);
    pwm_config cfg = pwm_get_default_config();
    // Divide system clock so one PWM tick = 1 us, wrap = one 20 ms frame.
    pwm_config_set_clkdiv(&cfg, (float)clock_get_hz(clk_sys) / SERVO_TICK_HZ);
    pwm_config_set_wrap(&cfg, SERVO_PERIOD_US - 1);
    pwm_init(slice, &cfg, true);
}

static void servo_write_angle(uint pin, float deg)
{
    if (deg < 0.0f)           deg = 0.0f;
    if (deg > SERVO_MAX_DEG)  deg = SERVO_MAX_DEG;
    // Linear map: 0..180 deg -> SERVO_MIN_US..SERVO_MAX_US (1 tick = 1 us)
    float pulse_us = SERVO_MIN_US + (deg / SERVO_MAX_DEG) * (SERVO_MAX_US - SERVO_MIN_US);
    pwm_set_gpio_level(pin, (uint16_t)pulse_us);
}

// ============================================================
//  ANGLE HELPERS
// ============================================================

// Wrap an angle difference into [-180, +180] so the filter always
// takes the short way around the circle.
static inline float wrap_deg(float a)
{
    while (a >  180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

// Physical frame: X = left->right, Y = front->back, Z = bottom->top.
// Tilting about X moves gravity within the Y-Z plane.
// Level => gravity on Z => (ay, az) = (0, 1) => angle 0 deg.
// Rotating by +theta about X (right-hand rule) gives ay = sin(theta),
// az = cos(theta), so atan2(ay, az) = +theta, the same sign as the gyro.
static inline float accel_angle_deg(float ay, float az)
{
    return atan2f(ay, az) * RAD_TO_DEG;
}

// Average offset so that (accel angle + offset) == 0 in the current pose.
// Uses the SAME formula as the main loop (the old version used atan2(-ax, ay),
// which measures a different axis and gave a wrong offset).
static float calibrate(const mpu_t *mpu)
{
    printf("Calibrating level offset...\n");
    float sum = 0.0f;
    for (int i = 0; i < CALIB_SAMPLES; i++) {
        float ax, ay, az;
        read_accel_mapped(mpu, &ax, &ay, &az);
        sum -= accel_angle_deg(ay, az);
        sleep_ms(CALIB_DELAY_MS);
    }
    return sum / CALIB_SAMPLES;
}

// Average gyro X reading while the board is held still = constant bias.
static float measure_gyro_bias(const mpu_t *mpu)
{
    printf("Finding gyro bias (keep board still)...\n");
    float sum = 0.0f;
    for (int i = 0; i < GYRO_BIAS_SAMPLES; i++) {
        float gx, gy, gz;
        read_gyro_mapped(mpu, &gx, &gy, &gz);
        sum += gx;
        sleep_ms(GYRO_BIAS_DELAY_MS);
    }
    return sum / GYRO_BIAS_SAMPLES;
}

// ============================================================
//  MAIN
// ============================================================
int main()
{
    stdio_init_all();

    // ---- 1. Servo to neutral first so the mechanism starts centered ----
    servo_init(SERVO_PIN);
    servo_write_angle(SERVO_PIN, SERVO_CENTER);
    printf("Setting servo default angle...\n");
    sleep_ms(SERVO_SETTLE_MS);

    // ---- 2. Sensor init ----
    spi_init(SPI, BAUDRATE);
    mpu_t mpu;
    mpu_config_t cfg = { SPI, SCK, MOSI, MISO, CS };
    if (!mpu_init_spi(&mpu, &cfg)) {
        // Without a sensor the loop would feed garbage to the servo: stop here.
        while (true) {
            printf("MPU not found!\n");
            sleep_ms(1000);
        }
    }

    float ax, ay, az, gx, gy, gz;

    // ---- 3. Optional "this pose is level" calibration ----
    float offset = CALIBRATE_ON_START ? calibrate(&mpu) : 0.0f;

    // ---- 4. Gyro bias (board must be still) ----
    float gyroBias = measure_gyro_bias(&mpu);
    printf("Gyro bias: %f\n", gyroBias);

    // ---- 5. Initial angle comes from the accelerometer only ----
    read_accel_mapped(&mpu, &ax, &ay, &az);
    float angle = accel_angle_deg(ay, az) + offset;

    PID pid(KP, KI, KD, -PID_OUT_LIMIT, PID_OUT_LIMIT);

    uint64_t last_us = time_us_64();
    int   print_count = 0;
    float gyroOnly = 0.0f;      // debug: pure gyro integration (drifts)

    while (true) {
        // ---- A. Time step ----
        uint64_t now_us = time_us_64();
        float dt = (now_us - last_us) / 1e6f;     // us -> s
        last_us = now_us;
        if (dt > DT_MAX_S) dt = DT_MAX_S;         // protect against stalls

        // ---- B. Read sensors ----
        read_accel_mapped(&mpu, &ax, &ay, &az);
        read_gyro_mapped(&mpu, &gx, &gy, &gz);

        // ---- C. Accelerometer angle (absolute, noisy, no drift) ----
        float accMag   = sqrtf(ax * ax + ay * ay + az * az);  // ~1 g when still
        float accAngle = accel_angle_deg(ay, az) + offset;

        // ---- D. Gyro angle (smooth, but drifts) ----
        // gx is the right-hand rotation rate about X, and accel_angle_deg()
        // uses the same sign convention, so no flip is needed.
        // (If gyro= and acc= move in opposite directions, negate this.)
        float gyroRate = gx - gyroBias;           // deg/s
        angle += gyroRate * dt;                   // integrate rate -> angle

        // ---- E. Complementary filter ----
        // Gyro does the fast work; accel gently corrects the drift,
        // but only when it is measuring gravity alone.
        bool accQuiet  = fabsf(accMag - 1.0f) < ACC_MAG_TOLERANCE_G;
        bool rotSlow   = fabsf(gyroRate) < ACC_MAX_GYRO_DPS;
        bool trustAcc  = accQuiet && rotSlow;

        // Blend factor derived from time constant, so behaviour does not
        // change if the loop rate changes.  (tau=0.25 s, dt=5 ms -> k~0.02)
        float k = trustAcc ? dt / (FILTER_TAU_S + dt) : 0.0f;
        angle += k * wrap_deg(accAngle - angle);
        angle  = wrap_deg(angle);                 // keep angle in [-180,180]

        // ---- F. Slow gyro-bias tracking while nearly stationary ----
        if (trustAcc && fabsf(gyroRate) < BIAS_TRACK_MAX_DPS)
            gyroBias += BIAS_TRACK_GAIN * (gx - gyroBias);

        // ---- G. PID: target 0 deg, D-term uses gyro rate directly ----
        float correction = pid.update(0.0f, angle, gyroRate, dt);
        servo_write_angle(SERVO_PIN, SERVO_CENTER - angle);

        // ---- H. Debug output ----
        gyroOnly += gyroRate * dt;
        if (++print_count >= PRINT_EVERY_N_LOOPS) {
            print_count = 0;
            printf("fused=%.2f gyro=%.2f acc=%.2f mag=%.3f\n",
                   angle, gyroOnly, accAngle, accMag);
            printf("angle=%.2f correction=%.2f dt=%.4f\n",
                   angle, correction, dt);
        }

        sleep_ms(LOOP_PERIOD_MS);
    }
}