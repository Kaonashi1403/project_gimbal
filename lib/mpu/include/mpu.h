#include <stdint.h>
#include <stdbool.h>
#include "hardware/spi.h"

typedef struct {
    spi_inst_t *spi;
    uint8_t sck, mosi, miso, cs;
} mpu_config_t;

typedef struct {
    mpu_config_t cfg;
    float accel_lsb_per_g;// 16384 at ±2g (default)
    float gyro_lsb_per_dps;// 131 at ±250 dps (default)
} mpu_t;

// Returns false if WHO_AM_I doesn't answer
bool mpu_init_spi(mpu_t *mpu, const mpu_config_t *cfg);

uint8_t mpu_read_reg(const mpu_t *mpu, uint8_t reg);
void    mpu_write_reg(const mpu_t *mpu, uint8_t reg, uint8_t value);

void mpu_read_accel(const mpu_t *mpu, int16_t *ax, int16_t *ay, int16_t *az);
void mpu_read_gyro (const mpu_t *mpu, int16_t *gx, int16_t *gy, int16_t *gz);

void mpu_read_accel_g  (const mpu_t *mpu, float *ax, float *ay, float *az);
void mpu_read_gyro_dps (const mpu_t *mpu, float *gx, float *gy, float *gz);

