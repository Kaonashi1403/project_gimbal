#include "mpu.h"
#include "pico/stdlib.h"
#include <inttypes.h>
#include <stddef.h>

#define REG_WHO_AM_I   0x75
#define REG_PWR_MGMT_1 0x6B
#define REG_USER_CTRL  0x6A
#define REG_ACCEL_OUT  0x3B
#define REG_GYRO_OUT   0x43

static inline void cs_low (const mpu_t *m) { gpio_put(m->cfg.cs, 0); }
static inline void cs_high(const mpu_t *m) { gpio_put(m->cfg.cs, 1); }

// One burst read: reg address, then len bytes come back. buf[0] is junk.
static void read_burst(const mpu_t *m, uint8_t reg, uint8_t *buf, size_t len) {
    uint8_t tx[7] = {0};
    tx[0] = reg | 0x80;
    cs_low(m);
    spi_write_read_blocking(m->cfg.spi, tx, buf, len + 1);
    cs_high(m);
}

void mpu_write_reg(const mpu_t *m, uint8_t reg, uint8_t value) {
    uint8_t tx[2] = { reg & 0x7F, value };
    cs_low(m);
    spi_write_blocking(m->cfg.spi, tx, 2);
    cs_high(m);
}

uint8_t mpu_read_reg(const mpu_t *m, uint8_t reg) {
    uint8_t rx[2];
    read_burst(m, reg, rx, 1);
    return rx[1];
}

bool mpu_init_spi(mpu_t *m, const mpu_config_t *cfg) {
    m->cfg = *cfg;
    m->accel_lsb_per_g = 16384.0f;
    m->gyro_lsb_per_dps = 131.0f;

    gpio_set_function(cfg->sck,  GPIO_FUNC_SPI);
    gpio_set_function(cfg->mosi, GPIO_FUNC_SPI);
    gpio_set_function(cfg->miso, GPIO_FUNC_SPI);
    spi_set_format(cfg->spi, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

    gpio_init(cfg->cs);
    gpio_set_dir(cfg->cs, GPIO_OUT);
    gpio_put(cfg->cs, 1);

    mpu_write_reg(m, REG_PWR_MGMT_1, 0x80);  // reset
    sleep_ms(100);
    mpu_write_reg(m, REG_PWR_MGMT_1, 0x01);  // wake, use best clock
    mpu_write_reg(m, REG_USER_CTRL, 0x10);   // disable I2C mode (recommended when using SPI)
    sleep_ms(10);

    uint8_t id = mpu_read_reg(m, REG_WHO_AM_I);
    return id == 0x71 || id == 0x70 || id == 0x73;  // 9250 / 6500 / 9255
}

static void read_vec(const mpu_t *m, uint8_t reg, int16_t *x, int16_t *y, int16_t *z) {
    uint8_t rx[7];
    read_burst(m, reg, rx, 6);
    if (x) *x = (int16_t)((rx[1] << 8) | rx[2]);
    if (y) *y = (int16_t)((rx[3] << 8) | rx[4]);
    if (z) *z = (int16_t)((rx[5] << 8) | rx[6]);
}

void mpu_read_accel(const mpu_t *m, int16_t *ax, int16_t *ay, int16_t *az) {
    read_vec(m, REG_ACCEL_OUT, ax, ay, az);
}
void mpu_read_gyro(const mpu_t *m, int16_t *gx, int16_t *gy, int16_t *gz) {
    read_vec(m, REG_GYRO_OUT, gx, gy, gz);
}

void mpu_read_gyro_dps(const mpu_t *m, float *gx, float *gy, float *gz) {
    int16_t x, y, z;
    read_vec(m, REG_GYRO_OUT, &x, &y, &z);
    if (gx) *gx = x / m->gyro_lsb_per_dps;
    if (gy) *gy = y / m->gyro_lsb_per_dps;
    if (gz) *gz = z / m->gyro_lsb_per_dps;
}

void mpu_read_accel_g(const mpu_t *m, float *ax, float *ay, float *az) {
    int16_t x, y, z;
    read_vec(m, REG_ACCEL_OUT, &x, &y, &z);
    if (ax) *ax = x / m->accel_lsb_per_g;
    if (ay) *ay = y / m->accel_lsb_per_g;
    if (az) *az = z / m->accel_lsb_per_g;
}