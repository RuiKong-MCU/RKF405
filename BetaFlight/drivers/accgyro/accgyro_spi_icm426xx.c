#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#if defined(USE_GYRO_SPI_ICM42605) || defined(USE_GYRO_SPI_ICM42688P)

#include "common/axis.h"
#include "common/utils.h"
#include "build/debug.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/accgyro/accgyro_spi_icm426xx.h"
#include "drivers/bus_spi.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/sensor.h"
#include "drivers/time.h"

#include "sensors/gyro.h"

#ifndef ICM426XX_CLOCK
#define ICM426XX_MAX_SPI_CLK_HZ 24000000
#else
#define ICM426XX_MAX_SPI_CLK_HZ ICM426XX_CLOCK
#endif

#define ICM426XX_RA_REG_BANK_SEL                    0x76
#define ICM426XX_BANK_SELECT0                       0x00
#define ICM426XX_BANK_SELECT1                       0x01
#define ICM426XX_BANK_SELECT2                       0x02
#define ICM426XX_BANK_SELECT3                       0x03
#define ICM426XX_BANK_SELECT4                       0x04

#define ICM426XX_RA_INTF_CONFIG5                    0x7B  // User Bank1
#define ICM426XX_INTF_CONFIG5_PIN9_FUNCTION_MASK    0x06  // bits[2:1]
#define ICM426XX_INTF_CONFIG5_PIN9_FUNCTION_CLKIN   (2 << 1)  // 10b for CLKIN

#define ICM426XX_RA_INTF_CONFIG1                    0x4D
#define ICM426XX_INTF_CONFIG1_AFSR_MASK             0xC0
#define ICM426XX_INTF_CONFIG1_AFSR_DISABLE          0x40
#define ICM426XX_INTF_CONFIG1_RTC_MODE_BIT          2

#define ICM426XX_RA_PWR_MGMT0                       0x4E  // User Bank0
#define ICM426XX_PWR_MGMT0_ACCEL_MODE_LN            (3 << 0)
#define ICM426XX_PWR_MGMT0_GYRO_MODE_LN             (3 << 2)
#define ICM426XX_PWR_MGMT0_GYRO_ACCEL_MODE_OFF      ((0 << 0) | (0 << 2))
#define ICM426XX_PWR_MGMT0_TEMP_DISABLE_OFF         (0 << 5)
#define ICM426XX_PWR_MGMT0_TEMP_DISABLE_ON          (1 << 5)

#define ICM426XX_RA_GYRO_CONFIG0                    0x4F
#define ICM426XX_RA_ACCEL_CONFIG0                   0x50

#define ICM426XX_RA_GYRO_CONFIG_STATIC3             0x0C  // User Bank1
#define ICM426XX_RA_GYRO_CONFIG_STATIC4             0x0D  // User Bank1
#define ICM426XX_RA_GYRO_CONFIG_STATIC5             0x0E  // User Bank1
#define ICM426XX_RA_ACCEL_CONFIG_STATIC2            0x03  // User Bank2
#define ICM426XX_RA_ACCEL_CONFIG_STATIC3            0x04  // User Bank2
#define ICM426XX_RA_ACCEL_CONFIG_STATIC4            0x05  // User Bank2

#define ICM426XX_RA_GYRO_ACCEL_CONFIG0              0x52  // User Bank0
#define ICM426XX_ACCEL_UI_FILT_BW_LOW_LATENCY       (15 << 4)
#define ICM426XX_GYRO_UI_FILT_BW_LOW_LATENCY        (15 << 0)

#define ICM426XX_RA_GYRO_DATA_X1                    0x25  // User Bank0
#define ICM426XX_RA_ACCEL_DATA_X1                   0x1F  // User Bank0

#define ICM426XX_RA_INT_CONFIG                      0x14  // User Bank0
#define ICM426XX_INT1_MODE_PULSED                   (0 << 2)
#define ICM426XX_INT1_MODE_LATCHED                  (1 << 2)
#define ICM426XX_INT1_DRIVE_CIRCUIT_OD              (0 << 1)
#define ICM426XX_INT1_DRIVE_CIRCUIT_PP              (1 << 1)
#define ICM426XX_INT1_POLARITY_ACTIVE_LOW           (0 << 0)
#define ICM426XX_INT1_POLARITY_ACTIVE_HIGH          (1 << 0)

#define ICM426XX_RA_INT_CONFIG0                     0x63  // User Bank0
#define ICM426XX_UI_DRDY_INT_CLEAR_ON_SBR           ((0 << 5) || (0 << 4))
#define ICM426XX_UI_DRDY_INT_CLEAR_ON_F1BR          ((1 << 5) || (0 << 4))

#define ICM426XX_RA_INT_CONFIG1                     0x64  // User Bank0
#define ICM426XX_INT_ASYNC_RESET_BIT                4
#define ICM426XX_INT_TDEASSERT_DISABLE_BIT          5
#define ICM426XX_INT_TPULSE_DURATION_BIT            6
#define ICM426XX_INT_TPULSE_DURATION_8              (1 << ICM426XX_INT_TPULSE_DURATION_BIT)
#define ICM426XX_INT_TDEASSERT_DISABLED             (1 << ICM426XX_INT_TDEASSERT_DISABLE_BIT)

#define ICM426XX_RA_INT_SOURCE0                     0x65  // User Bank0
#define ICM426XX_UI_DRDY_INT1_EN_ENABLED            (1 << 3)

typedef enum {
    ODR_CONFIG_8K = 0,
    ODR_CONFIG_4K,
    ODR_CONFIG_2K,
    ODR_CONFIG_1K,
    ODR_CONFIG_COUNT
} odrConfig_e;

typedef enum {
    AAF_CONFIG_258HZ = 0,
    AAF_CONFIG_536HZ,
    AAF_CONFIG_997HZ,
    AAF_CONFIG_1962HZ,
    AAF_CONFIG_COUNT
} aafConfig_e;

typedef struct aafConfig_s {
    uint8_t delt;
    uint16_t deltSqr;
    uint8_t bitshift;
} aafConfig_t;

static uint8_t odrLUT[ODR_CONFIG_COUNT] = {3,4,5,6};
static aafConfig_t aafLUT42688[AAF_CONFIG_COUNT] = {
    {  6,   36, 10 },
    { 12,  144,  8 },
    { 21,  440,  6 },
    { 37, 1376,  4 }
};
static aafConfig_t aafLUT42605[AAF_CONFIG_COUNT] = {
    { 21,  440,  6 },
    { 39, 1536,  4 },
    { 63, 3968,  3 },
    { 63, 3968,  3 }
};

uint8_t icm426xxSpiDetect(const extDevice_t *dev)
{
    spiWriteReg(dev, ICM426XX_RA_PWR_MGMT0, 0x00);
    uint8_t who;
    for (int i = 0; i < 20; i++) {
        delay(150);
        who = spiReadRegMsk(dev, MPU_RA_WHO_AM_I);
        if (who == ICM42605_WHO_AM_I_CONST) return ICM_42605_SPI;
        if (who == ICM42688P_WHO_AM_I_CONST) return ICM_42688P_SPI;
    }
    return MPU_NONE;
}

void icm426xxAccInit(accDev_t *acc)
{
    acc->acc_1G = 512 * 4;
}

bool icm426xxSpiAccDetect(accDev_t *acc)
{
    switch (acc->mpuDetectionResult.sensor) {
    case ICM_42605_SPI:
    case ICM_42688P_SPI:
        acc->initFn = icm426xxAccInit;
        acc->readFn = mpuAccReadSPI;
        return true;
    default:
        return false;
    }
}

static aafConfig_t getGyroAafConfig(const mpuSensor_e model, const aafConfig_e cfg)
{
    if (model == ICM_42605_SPI) return aafLUT42605[cfg];
    return aafLUT42688[cfg];
}

static void turnGyroAccOff(const extDevice_t *dev)
{
    spiWriteReg(dev, ICM426XX_RA_PWR_MGMT0, ICM426XX_PWR_MGMT0_GYRO_ACCEL_MODE_OFF);
}

static void turnGyroAccOn(const extDevice_t *dev)
{
    spiWriteReg(dev, ICM426XX_RA_PWR_MGMT0,
                ICM426XX_PWR_MGMT0_TEMP_DISABLE_OFF |
                ICM426XX_PWR_MGMT0_ACCEL_MODE_LN |
                ICM426XX_PWR_MGMT0_GYRO_MODE_LN);
    delay(1);
}

static void setUserBank(const extDevice_t *dev, const uint8_t bank)
{
    spiWriteReg(dev, ICM426XX_RA_REG_BANK_SEL, bank & 0x07);
}

void icm426xxGyroInit(gyroDev_t *gyro)
{
    const extDevice_t *dev = &gyro->dev;
    uint8_t data;

    spiSetClkDivisor(dev, spiCalculateDivider(ICM426XX_MAX_SPI_CLK_HZ));
    mpuGyroInit(gyro);
    gyro->accDataReg  = ICM426XX_RA_ACCEL_DATA_X1;
    gyro->gyroDataReg = ICM426XX_RA_GYRO_DATA_X1;

    // 关闭传感器
    setUserBank(dev, ICM426XX_BANK_SELECT0);
    turnGyroAccOff(dev);

    // 配置 Gyro AAF
    {
        aafConfig_t cfg = getGyroAafConfig(gyro->mpuDetectionResult.sensor, gyroConfig()->gyro_hardware_lpf);
        setUserBank(dev, ICM426XX_BANK_SELECT1);
        spiWriteReg(dev, ICM426XX_RA_GYRO_CONFIG_STATIC3, cfg.delt);
        spiWriteReg(dev, ICM426XX_RA_GYRO_CONFIG_STATIC4, cfg.deltSqr & 0xFF);
        spiWriteReg(dev, ICM426XX_RA_GYRO_CONFIG_STATIC5, (cfg.deltSqr >> 8) | (cfg.bitshift << 4));
    }

    // 配置 Acc AAF
    {
        aafConfig_t cfg = getGyroAafConfig(gyro->mpuDetectionResult.sensor, AAF_CONFIG_258HZ);
        setUserBank(dev, ICM426XX_BANK_SELECT2);
        spiWriteReg(dev, ICM426XX_RA_ACCEL_CONFIG_STATIC2, cfg.delt << 1);
        spiWriteReg(dev, ICM426XX_RA_ACCEL_CONFIG_STATIC3, cfg.deltSqr & 0xFF);
        spiWriteReg(dev, ICM426XX_RA_ACCEL_CONFIG_STATIC4, (cfg.deltSqr >> 8) | (cfg.bitshift << 4));
    }

    // 配置 UI 滤波
    setUserBank(dev, ICM426XX_BANK_SELECT0);
    spiWriteReg(dev, ICM426XX_RA_GYRO_ACCEL_CONFIG0,
                ICM426XX_ACCEL_UI_FILT_BW_LOW_LATENCY |
                ICM426XX_GYRO_UI_FILT_BW_LOW_LATENCY);

    // 中断配置
    spiWriteReg(dev, ICM426XX_RA_INT_CONFIG,
                ICM426XX_INT1_MODE_PULSED |
                ICM426XX_INT1_DRIVE_CIRCUIT_PP |
                ICM426XX_INT1_POLARITY_ACTIVE_HIGH);
    spiWriteReg(dev, ICM426XX_RA_INT_CONFIG0, ICM426XX_UI_DRDY_INT_CLEAR_ON_SBR);
    spiWriteReg(dev, ICM426XX_RA_INT_SOURCE0, ICM426XX_UI_DRDY_INT1_EN_ENABLED);

    data = spiReadRegMsk(dev, ICM426XX_RA_INT_CONFIG1);
    data &= ~(1 << ICM426XX_INT_ASYNC_RESET_BIT);
    data |= (ICM426XX_INT_TPULSE_DURATION_8 | ICM426XX_INT_TDEASSERT_DISABLED);
    spiWriteReg(dev, ICM426XX_RA_INT_CONFIG1, data);

    // 禁用 AFSR
    data = spiReadRegMsk(dev, ICM426XX_RA_INTF_CONFIG1);
    data &= ~ICM426XX_INTF_CONFIG1_AFSR_MASK;
    data |= ICM426XX_INTF_CONFIG1_AFSR_DISABLE;
    spiWriteReg(dev, ICM426XX_RA_INTF_CONFIG1, data);

    // === 新增：外部 32kHz 时钟配置 ===
    // 切换到 Bank1，配置 PIN9=CLKIN
    setUserBank(dev, ICM426XX_BANK_SELECT1);
    data = spiReadRegMsk(dev, ICM426XX_RA_INTF_CONFIG5);
    data = (data & ~ICM426XX_INTF_CONFIG5_PIN9_FUNCTION_MASK)
         | ICM426XX_INTF_CONFIG5_PIN9_FUNCTION_CLKIN;
    spiWriteReg(dev, ICM426XX_RA_INTF_CONFIG5, data);
    // 切换回 Bank0，启用 RTC_MODE
    setUserBank(dev, ICM426XX_BANK_SELECT0);
    data = spiReadRegMsk(dev, ICM426XX_RA_INTF_CONFIG1);
    data |= (1 << ICM426XX_INTF_CONFIG1_RTC_MODE_BIT);
    spiWriteReg(dev, ICM426XX_RA_INTF_CONFIG1, data);
    // 清除 INT_ASYNC_RESET
    data = spiReadRegMsk(dev, ICM426XX_RA_INT_CONFIG1);
    data &= ~(1 << ICM426XX_INT_ASYNC_RESET_BIT);
    spiWriteReg(dev, ICM426XX_RA_INT_CONFIG1, data);

    // 重新开启传感器
    turnGyroAccOn(dev);

    // ODR/FSR 配置
    {
        uint8_t odrCfg;
        unsigned decim = llog2(gyro->mpuDividerDrops + 1);
        if (gyro->gyroRateKHz && decim < ODR_CONFIG_COUNT) {
            odrCfg = odrLUT[decim];
        } else {
            odrCfg = odrLUT[ODR_CONFIG_1K];
            gyro->gyroRateKHz = GYRO_RATE_1_kHz;
        }
        STATIC_ASSERT(INV_FSR_2000DPS == 3, "INV_FSR_2000DPS must be 3");
        spiWriteReg(dev, ICM426XX_RA_GYRO_CONFIG0,
                    (3 - INV_FSR_2000DPS) << 5 | (odrCfg & 0x0F));
        delay(15);

        STATIC_ASSERT(INV_FSR_16G == 3, "INV_FSR_16G must be 3");
        spiWriteReg(dev, ICM426XX_RA_ACCEL_CONFIG0,
                    (3 - INV_FSR_16G) << 5 | (odrCfg & 0x0F));
        delay(15);
    }
}

bool icm426xxSpiGyroDetect(gyroDev_t *gyro)
{
    if (gyro->mpuDetectionResult.sensor == ICM_42605_SPI ||
        gyro->mpuDetectionResult.sensor == ICM_42688P_SPI) {
        gyro->initFn = icm426xxGyroInit;
        gyro->readFn = mpuGyroReadSPI;
        gyro->scale = GYRO_SCALE_2000DPS;
        return true;
    }
    return false;
}

#endif // USE_GYRO_SPI_ICM42605 || USE_GYRO_SPI_ICM42688P
