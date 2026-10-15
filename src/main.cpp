#include <stdio.h>
#include "pico/stdlib.h"
#include <math.h>
#include "hardware/pwm.h"
#include "hardware/clocks.h"

extern "C" {
#include "mpu.h"
}
#include "pid.hpp"

#define SPI spi0
#define SCK 18
#define MOSI 19
#define MISO 16
#define CS 17
#define BAUDRATE 1000 * 1000

#define SERVO_MIN_US 500
#define SERVO_MAX_US 2400
#define SERVO_CENTER 90.0f 
#define MPU_ADDR 0x68
#define SERVO_PIN 15

#define KP 1.9f
#define KI 0.0f
#define KD 0.0f

static void servo_init(uint pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(pin);
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, clock_get_hz(clk_sys) / 1000000.0f);
    pwm_config_set_wrap(&cfg, 20000 - 1);
    pwm_init(slice, &cfg, true);
}
static void servo_write_angle(uint pin, float deg) {
    if (deg < 0)   deg = 0;
    if (deg > 180) deg = 180;
    pwm_set_gpio_level(pin, SERVO_MIN_US + (deg / 180.0f) * (SERVO_MAX_US - SERVO_MIN_US));
}

float calibrate(const mpu_t *mpu, float *ax, float *ay, float *az) {
    printf("Calibrating...\n");
    float offset = 0;
    for (int i = 0; i < 100; i++) {
        mpu_read_accel_g(mpu, ax, ay, az);
        float angle = atan2(*ay, *ax);
        angle = angle*(180/M_PI);
        offset+=(0 - angle);
        sleep_ms(50);
    }
    return offset/100;
}


int main() {
    stdio_init_all();


    servo_init(SERVO_PIN);
    servo_write_angle(SERVO_PIN, SERVO_CENTER);
    servo_write_angle(SERVO_PIN, SERVO_CENTER);
    printf("Setting servo default angle...\n");
    sleep_ms(3000);
    


    spi_init(SPI, BAUDRATE);
    mpu_t mpu;
    mpu_config_t cfg = { SPI, SCK, MOSI, MISO, CS};
    if (!mpu_init_spi(&mpu, &cfg)) printf("MPU not found!\n");
    float offset = 0;
    float ax, ay, az, gx, gy, gz;
    // offset = calibrate(&mpu, &ax, &ay, &az);
    PID pid(KP, KI, KD, -90.0f, 90.0f);
    mpu_read_accel_g(&mpu, &ax, &ay, &az);
    float angle = atan2f(-ax, ay) * (180.0f / M_PI) + offset;

    float gyroBias = 0;
    printf("Finding gyro bias...\n");
    for (int i = 0; i < 500; i++) {
        mpu_read_gyro_dps(&mpu, NULL, NULL, &gz);
        gyroBias += gz; 
        sleep_ms(2); 
    }
    gyroBias /= 500.0f;
    printf("Gyro bias: %f\n", gyroBias);

    uint64_t last_us = time_us_64();
    int print_count = 0;
    while (true) {
        // mpu_read_accel_g(&mpu, &ax, &ay, &az);
        // printf("ax: %2f ; ay: %2f ; az: %2f\n", ax, ay, az);


        uint64_t now_us = time_us_64();
        float dt = (now_us - last_us) / 1e6f;//us to ms
        last_us = now_us;

        mpu_read_accel_g(&mpu, &ax, &ay, &az);
        mpu_read_gyro_dps(&mpu, &gx, &gy, &gz); 
        
        
        float accMag = sqrtf(ax*ax + ay*ay + az*az);
        float accAngle = atan2f(-ax, ay) * (180.0f / M_PI) + offset;

        float gyroRate = -(gz - gyroBias); 
        angle += gyroRate * dt;

        float err = fabsf(accMag - 1.0f);
        bool trustAcc = (err < 0.05f) && (fabsf(gyroRate) < 20.0f);
        float k = trustAcc ? 0.02f : 0.0f;

        float diff = accAngle - angle;
        while (diff >  180.0f) diff -= 360.0f;
        while (diff < -180.0f) diff += 360.0f;
        angle += k * diff;

        if (trustAcc && fabsf(gyroRate) < 2.0f)
        gyroBias += 0.001f * (gz - gyroBias);

        float correction = pid.update(0.0f, angle, gyroRate, dt);
        servo_write_angle(SERVO_PIN, SERVO_CENTER + correction);

        static float gyroOnly = 0;
        gyroOnly += gyroRate * dt;
        if (++print_count >= 10) { 
            print_count = 0;
            printf("fused=%.2f gyro=%.2f acc=%.2f mag=%.3f\n", angle, gyroOnly, accAngle, accMag);
            printf("angle=%.2f correction=%.2f dt=%.4f\n", angle, correction, dt);
        }
        sleep_ms(5);
    }

    // float Testangle = 90;
    // while (true) {
    //     servo_write_angle(SERVO_PIN,Testangle);
    //     Testangle+= 15;
    //     sleep_ms(100);
    // }
}