#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/utils.h"
#include "common/log.h"

#include "drivers/system.h"
#include "drivers/time.h"

#include "drivers/sensor.h"
#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/accgyro/accgyro_icm42605.h"

// #define USE_IMU_ICM42605                     


#if defined(USE_IMU_ICM42605)

#define ICM42605_RA_PWR_MGMT0                       0x4E

#define ICM42605_PWR_MGMT0_ACCEL_MODE_LN            (3 << 0)
#define ICM42605_PWR_MGMT0_GYRO_MODE_LN             (3 << 2)
#define ICM42605_PWR_MGMT0_TEMP_DISABLE_OFF         (0 << 5)
#define ICM42605_PWR_MGMT0_TEMP_DISABLE_ON          (1 << 5)
#define ICM426XX_PWR_MGMT0_GYRO_ACCEL_MODE_OFF      ((0 << 0) | (0 << 2))


#define ICM426XX_RA_REG_BANK_SEL                    0x76
#define ICM426XX_BANK_SELECT0                       0x00
#define ICM426XX_BANK_SELECT1                       0x01
#define ICM426XX_BANK_SELECT2                       0x02
#define ICM426XX_BANK_SELECT3                       0x03
#define ICM426XX_BANK_SELECT4                       0x04

#define ICM42605_RA_GYRO_CONFIG0                    0x4F
#define ICM42605_RA_ACCEL_CONFIG0                   0x50

#define ICM42605_RA_GYRO_ACCEL_CONFIG0              0x52

#define ICM42605_ACCEL_UI_FILT_BW_LOW_LATENCY       (15 << 4)
#define ICM42605_GYRO_UI_FILT_BW_LOW_LATENCY        (15 << 0)

#define ICM42605_RA_GYRO_DATA_X1                    0x25
#define ICM42605_RA_ACCEL_DATA_X1                   0x1F
#define ICM42605_RA_TEMP_DATA1                      0x1D

#define ICM42605_RA_INT_CONFIG                      0x14
#define ICM42605_INT1_MODE_PULSED                   (0 << 2)
#define ICM42605_INT1_MODE_LATCHED                  (1 << 2)
#define ICM42605_INT1_DRIVE_CIRCUIT_OD              (0 << 1)
#define ICM42605_INT1_DRIVE_CIRCUIT_PP              (1 << 1)
#define ICM42605_INT1_POLARITY_ACTIVE_LOW           (0 << 0)
#define ICM42605_INT1_POLARITY_ACTIVE_HIGH          (1 << 0)

#define ICM42605_RA_INT_CONFIG0                     0x63
#define ICM42605_UI_DRDY_INT_CLEAR_ON_SBR           ((0 << 5) || (0 << 4))
#define ICM42605_UI_DRDY_INT_CLEAR_ON_SBR_DUPLICATE ((0 << 5) || (0 << 4)) // duplicate settings in datasheet, Rev 1.2.
#define ICM42605_UI_DRDY_INT_CLEAR_ON_F1BR          ((1 << 5) || (0 << 4))
#define ICM42605_UI_DRDY_INT_CLEAR_ON_SBR_AND_F1BR  ((1 << 5) || (1 << 4))

#define ICM42605_RA_INT_CONFIG1                     0x64
#define ICM42605_INT_ASYNC_RESET_BIT                4
#define ICM42605_INT_TDEASSERT_DISABLE_BIT          5
#define ICM42605_INT_TDEASSERT_ENABLED              (0 << ICM42605_INT_TDEASSERT_DISABLE_BIT)
#define ICM42605_INT_TDEASSERT_DISABLED             (1 << ICM42605_INT_TDEASSERT_DISABLE_BIT)
#define ICM42605_INT_TPULSE_DURATION_BIT            6
#define ICM42605_INT_TPULSE_DURATION_100            (0 << ICM42605_INT_TPULSE_DURATION_BIT)
#define ICM42605_INT_TPULSE_DURATION_8              (1 << ICM42605_INT_TPULSE_DURATION_BIT)

#define ICM42605_RA_INT_SOURCE0                     0x65
#define ICM42605_UI_DRDY_INT1_EN_DISABLED           (0 << 3)
#define ICM42605_UI_DRDY_INT1_EN_ENABLED            (1 << 3)

#define ICM42605_INTF_CONFIG1                       0x4D
#define ICM42605_INTF_CONFIG1_AFSR_MASK             0xC0
#define ICM42605_INTF_CONFIG1_AFSR_DISABLE          0x40

#define ICM426XX_INTF_CONFIG1_RTC_MODE_BIT          2

// --- Registers for gyro and acc Anti-Alias Filter ---------
#define ICM426XX_RA_GYRO_CONFIG_STATIC3             0x0C  // User Bank 1
#define ICM426XX_RA_GYRO_CONFIG_STATIC4             0x0D  // User Bank 1
#define ICM426XX_RA_GYRO_CONFIG_STATIC5             0x0E  // User Bank 1
#define ICM426XX_RA_ACCEL_CONFIG_STATIC2            0x03  // User Bank 2
#define ICM426XX_RA_ACCEL_CONFIG_STATIC3            0x04  // User Bank 2
#define ICM426XX_RA_ACCEL_CONFIG_STATIC4            0x05  // User Bank 2



#define ICM426XX_RA_INTF_CONFIG5                    0x7B  // User Bank1
#define ICM426XX_INTF_CONFIG5_PIN9_FUNCTION_MASK    0x06  // bits[2:1]
#define ICM426XX_INTF_CONFIG5_PIN9_FUNCTION_CLKIN   (2 << 1)  // 10b for CLKIN



static bool is42688P = false;

typedef struct aafConfig_s {
    uint16_t freq;
    uint8_t delt;
    uint16_t deltSqr;
    uint8_t bitshift;
} aafConfig_t;

// Possible gyro Anti-Alias Filter (AAF) cutoffs for ICM-42688P
static aafConfig_t aafLUT42688[] = {  // see table in section 5.3 https://invensense.tdk.com/wp-content/uploads/2020/04/ds-000347_icm-42688-p-datasheet.pdf
    // freq, delt, deltSqr, bitshift
    { 42,  1,    1, 15 },
    { 84,  2,    4, 13 },
    {126,  3,    9, 12 },
    {170,  4,   16, 11 },
    {213,  5,   25, 10 },
    {258,  6,   36, 10 },
    {303,  7,   49,  9 },
    {536, 12,  144,  8 },
    {997, 21,  440,  6 },
    {1962, 37, 1376,  4 },
    { 0, 0, 0, 0}, // 42HZ
};

// Possible gyro Anti-Alias Filter (AAF) cutoffs for ICM-42688P
static aafConfig_t aafLUT42605[] = {  // see table in section 5.3 https://invensense.tdk.com/wp-content/uploads/2022/09/DS-000292-ICM-42605-v1.7.pdf
    // freq, delt, deltSqr, bitshift
    {  10,  1,    1, 15 },
    {  21,  2,    4, 13 },
    {  32,  3,    9, 12 },
    {  42,  4,   16, 11 },
    {  99,  9,   81,  9 },
    { 171, 15,  224,  7 },
    { 184, 16,  256,  7 },
    { 196, 17,  288,  7 },
    { 249, 21,  440,  6 },
    { 524, 39, 1536,  4 },
    { 995, 63, 3968,  3 },
    {   0,  0,    0,  0 }
};

static const aafConfig_t *getGyroAafConfig(bool is42688, const uint16_t desiredLpf);

static void setUserBank(const busDevice_t *dev, const uint8_t user_bank)
{
    busWrite(dev, ICM426XX_RA_REG_BANK_SEL, user_bank & 7);
}

static void icm42605AccInit(accDev_t *acc)
{
    acc->acc_1G = 512 * 4;
}

static bool icm42605AccRead(accDev_t *acc)
{
    uint8_t data[6];

    const bool ack = busReadBuf(acc->busDev, ICM42605_RA_ACCEL_DATA_X1, data, 6);
    if (!ack) {
        return false;
    }

    acc->ADCRaw[X] = (float) int16_val_big_endian(data, 0);
    acc->ADCRaw[Y] = (float) int16_val_big_endian(data, 1);
    acc->ADCRaw[Z] = (float) int16_val_big_endian(data, 2);

    return true;
}

bool icm42605AccDetect(accDev_t *acc)
{
    acc->busDev = busDeviceOpen(BUSTYPE_ANY, DEVHW_ICM42605, acc->imuSensorToUse);
    if (acc->busDev == NULL) {
        return false;
    }

    mpuContextData_t * ctx = busDeviceGetScratchpadMemory(acc->busDev);
    if (ctx->chipMagicNumber != 0x4265) {
        return false;
    }

    acc->initFn = icm42605AccInit;
    acc->readFn = icm42605AccRead;
    acc->accAlign = acc->busDev->param;

    return true;
}

static const gyroFilterAndRateConfig_t icm42605GyroConfigs[] = {
    /*                            DLPF  ODR */
    { GYRO_LPF_256HZ,   8000,   { 6,    3  } }, /* 400 Hz LPF */
    { GYRO_LPF_256HZ,   4000,   { 5,    4  } }, /* 250 Hz LPF */
    { GYRO_LPF_256HZ,   2000,   { 3,    5  } }, /* 250 Hz LPF */
    { GYRO_LPF_256HZ,   1000,   { 1,    6  } }, /* 250 Hz LPF */
    { GYRO_LPF_256HZ,    500,   { 0,    15 } }, /* 250 Hz LPF */
};

static void icm42605AccAndGyroInit(gyroDev_t *gyro)
{

    uint8_t data;


    busDevice_t * dev = gyro->busDev;
    const gyroFilterAndRateConfig_t * config = chooseGyroConfig(gyro->lpf, 1000000 / gyro->requestedSampleIntervalUs,
                                                                &icm42605GyroConfigs[0], ARRAYLEN(icm42605GyroConfigs));
    gyro->sampleRateIntervalUs = 1000000 / config->gyroRateHz;

    busSetSpeed(dev, BUS_SPEED_INITIALIZATION);

    setUserBank(dev, ICM426XX_BANK_SELECT0);
    busWrite(dev, ICM42605_RA_PWR_MGMT0, ICM42605_PWR_MGMT0_TEMP_DISABLE_OFF | ICM426XX_PWR_MGMT0_GYRO_ACCEL_MODE_OFF);
    delay(15);

    // /* ODR and dynamic range */
    // busWrite(dev, ICM42605_RA_GYRO_CONFIG0, (0x00) << 5 | (config->gyroConfigValues[1] & 0x0F));    /* 2000 deg/s */
    // delay(15);

    // busWrite(dev, ICM42605_RA_ACCEL_CONFIG0, (0x00) << 5 | (config->gyroConfigValues[1] & 0x0F));    /* 16 G deg/s */
    // delay(15);

    if (gyro->lpf != GYRO_LPF_NONE) {
        // Configure gyro Anti-Alias Filter (see section 5.3 "ANTI-ALIAS FILTER")
        const aafConfig_t *aafConfig = getGyroAafConfig(is42688P, gyro->lpf);
    
        setUserBank(dev, ICM426XX_BANK_SELECT1);
        busWrite(dev, ICM426XX_RA_GYRO_CONFIG_STATIC3, aafConfig->delt);
        busWrite(dev, ICM426XX_RA_GYRO_CONFIG_STATIC4, aafConfig->deltSqr & 0xFF);
        busWrite(dev, ICM426XX_RA_GYRO_CONFIG_STATIC5, (aafConfig->deltSqr >> 8) | (aafConfig->bitshift << 4));

        aafConfig = getGyroAafConfig(is42688P, 256);  // This was hard coded on BF
        setUserBank(dev, ICM426XX_BANK_SELECT2);
        busWrite(dev, ICM426XX_RA_ACCEL_CONFIG_STATIC2, aafConfig->delt << 1);
        busWrite(dev, ICM426XX_RA_ACCEL_CONFIG_STATIC3, aafConfig->deltSqr & 0xFF);
        busWrite(dev, ICM426XX_RA_ACCEL_CONFIG_STATIC4, (aafConfig->deltSqr >> 8) | (aafConfig->bitshift << 4));
    }

    setUserBank(dev, ICM426XX_BANK_SELECT0);
    /* LPF bandwidth */
    // low latency, same as BF
    busWrite(dev, ICM42605_RA_GYRO_ACCEL_CONFIG0, ICM42605_ACCEL_UI_FILT_BW_LOW_LATENCY | ICM42605_GYRO_UI_FILT_BW_LOW_LATENCY);
    delay(15);



    busWrite(dev, ICM42605_RA_INT_CONFIG, ICM42605_INT1_MODE_PULSED | ICM42605_INT1_DRIVE_CIRCUIT_PP | ICM42605_INT1_POLARITY_ACTIVE_HIGH);
    delay(15);
    busWrite(dev, ICM42605_RA_INT_CONFIG0, ICM42605_UI_DRDY_INT_CLEAR_ON_SBR);
    delay(15);

    busWrite(dev, ICM42605_RA_INT_SOURCE0, ICM42605_UI_DRDY_INT1_EN_ENABLED);
    delay(15);

    uint8_t intConfig1Value;
    // Datasheet says: "User should change setting to 0 from default setting of 1, for proper INT1 and INT2 pin operation"
    busRead(dev, ICM42605_RA_INT_CONFIG1, &intConfig1Value);
    intConfig1Value &= ~(1 << ICM42605_INT_ASYNC_RESET_BIT);
    intConfig1Value |= (ICM42605_INT_TPULSE_DURATION_8 | ICM42605_INT_TDEASSERT_DISABLED);
    busWrite(dev, ICM42605_RA_INT_CONFIG1, intConfig1Value);
    delay(15);

    //Disable AFSR as in BF and Ardupilot
    uint8_t intfConfig1Value;
    busRead(dev, ICM42605_INTF_CONFIG1, &intfConfig1Value);
    intfConfig1Value &= ~ICM42605_INTF_CONFIG1_AFSR_MASK;
    intfConfig1Value |= ICM42605_INTF_CONFIG1_AFSR_DISABLE;
    busWrite(dev, ICM42605_INTF_CONFIG1, intfConfig1Value);

    delay(15);


    setUserBank(dev, ICM426XX_BANK_SELECT1);
    uint8_t BANK1_intConfig1Value;
    busRead(dev, ICM426XX_RA_INTF_CONFIG5, &BANK1_intConfig1Value);
    BANK1_intConfig1Value = (BANK1_intConfig1Value & ~ICM426XX_INTF_CONFIG5_PIN9_FUNCTION_MASK)
    | ICM426XX_INTF_CONFIG5_PIN9_FUNCTION_CLKIN;
    busWrite(dev, ICM426XX_RA_INTF_CONFIG5, BANK1_intConfig1Value);
    delay(15);

    // // 切换回 Bank0，启用 RTC_MODE
    setUserBank(dev, ICM426XX_BANK_SELECT0);
    busRead(dev, ICM42605_INTF_CONFIG1,&data);
    data |= (1 << ICM426XX_INTF_CONFIG1_RTC_MODE_BIT);
    busWrite(dev, ICM42605_INTF_CONFIG1, data);
    // 清除 INT_ASYNC_RESET
    busRead(dev, ICM42605_RA_INT_CONFIG1,&data);
    data &= ~(1 << ICM42605_INT_ASYNC_RESET_BIT);
    busWrite(dev, ICM42605_RA_INT_CONFIG1, data);

    setUserBank(dev, ICM426XX_BANK_SELECT0);
    busWrite(dev, ICM42605_RA_PWR_MGMT0, ICM42605_PWR_MGMT0_TEMP_DISABLE_OFF | ICM42605_PWR_MGMT0_ACCEL_MODE_LN | ICM42605_PWR_MGMT0_GYRO_MODE_LN);
    delay(15);

    /* ODR and dynamic range */
    busWrite(dev, ICM42605_RA_GYRO_CONFIG0, (0x00) << 5 | (config->gyroConfigValues[1] & 0x0F));    /* 2000 deg/s */
    delay(15);

    busWrite(dev, ICM42605_RA_ACCEL_CONFIG0, (0x00) << 5 | (config->gyroConfigValues[1] & 0x0F));    /* 16 G deg/s */
    delay(15);   

    busSetSpeed(dev, BUS_SPEED_FAST);
}

static bool icm42605DeviceDetect(busDevice_t * dev)
{
    uint8_t tmp;
    uint8_t attemptsRemaining = 5;

    busSetSpeed(dev, BUS_SPEED_INITIALIZATION);

    busWrite(dev, ICM42605_RA_PWR_MGMT0, 0x00);

    do {
        delay(150);

        busRead(dev, MPU_RA_WHO_AM_I, &tmp);

        switch (tmp) {
            /* ICM42605 and ICM42688P share the register structure*/
            case ICM42605_WHO_AM_I_CONST:
                is42688P = false;
                return true;
            case ICM42688P_WHO_AM_I_CONST:
                is42688P = true;
                return true;

            default:
                // Retry detection
                break;
        }
    } while (attemptsRemaining--);

    return false;
}

static bool icm42605GyroRead(gyroDev_t *gyro)
{
    uint8_t data[6];

    const bool ack = busReadBuf(gyro->busDev, ICM42605_RA_GYRO_DATA_X1, data, 6);
    if (!ack) {
        return false;
    }

    gyro->gyroADCRaw[X] = (float) int16_val_big_endian(data, 0);
    gyro->gyroADCRaw[Y] = (float) int16_val_big_endian(data, 1);
    gyro->gyroADCRaw[Z] = (float) int16_val_big_endian(data, 2);

    return true;
}

static bool icm42605ReadTemperature(gyroDev_t *gyro, int16_t * temp)
{
    uint8_t data[2];

    const bool ack = busReadBuf(gyro->busDev, ICM42605_RA_TEMP_DATA1, data, 2);
    if (!ack) {
        return false;
    }
    // From datasheet: Temperature in Degrees Centigrade = (TEMP_DATA / 132.48) + 25 
    *temp = ( int16_val_big_endian(data, 0) / 13.248 ) + 250; // Temperature stored as degC*10

    return true;
}

bool icm42605GyroDetect(gyroDev_t *gyro)
{
    gyro->busDev = busDeviceInit(BUSTYPE_ANY, DEVHW_ICM42605, gyro->imuSensorToUse, OWNER_MPU);
    if (gyro->busDev == NULL) {
        return false;
    }

    if (!icm42605DeviceDetect(gyro->busDev)) {
        busDeviceDeInit(gyro->busDev);
        return false;
    }

    // Magic number for ACC detection to indicate that we have detected icm42605 gyro
    mpuContextData_t * ctx = busDeviceGetScratchpadMemory(gyro->busDev);
    ctx->chipMagicNumber = 0x4265;

    gyro->initFn = icm42605AccAndGyroInit;
    gyro->readFn = icm42605GyroRead;
    gyro->intStatusFn = gyroCheckDataReady;
    gyro->temperatureFn = icm42605ReadTemperature;
    gyro->scale = 1.0f / 16.4f;     // 16.4 dps/lsb scalefactor
    gyro->gyroAlign = gyro->busDev->param;

    return true;
}

static uint16_t getAafFreq(const uint8_t gyroLpf)
{
    switch (gyroLpf) {
        default:
        case GYRO_LPF_256HZ:
            return 256;
        case GYRO_LPF_188HZ:
            return 188;
        case GYRO_LPF_98HZ:
            return 98;
        case GYRO_LPF_42HZ:
            return 42;
        case GYRO_LPF_20HZ:
            return 20;
        case GYRO_LPF_10HZ:
            return 10;
        case GYRO_LPF_5HZ:
            return 5;
        case GYRO_LPF_NONE:
            return 0;
    }
}

static const aafConfig_t *getGyroAafConfig(bool is42688, const uint16_t desiredLpf)
{
    uint16_t desiredFreq = getAafFreq(desiredLpf);
    const aafConfig_t *aafConfigs = NULL;
    if (is42688) {
        aafConfigs = aafLUT42688;
    } else {
        aafConfigs = aafLUT42605;
    }
    int i;
    int8_t selectedFreq = aafConfigs[0].freq;
    const aafConfig_t * candidate = &aafConfigs[0];

    // Choose closest supported LPF value
    for (i = 1; aafConfigs[i].freq != 0; i++) {
        if (ABS(desiredFreq - aafConfigs[i].freq) < ABS(desiredFreq - selectedFreq)) {
            selectedFreq = aafConfigs[i].freq;
            candidate = &aafConfigs[i];
        }
    }

    LOG_VERBOSE(GYRO, "ICM426%s AAF CONFIG { %d, %d } -> { %d }; delt: %d deltSqr: %d, shift: %d",
		(is42688P ? "88" : "05"),
                desiredLpf, desiredFreq,
                candidate->freq,
                candidate->delt, candidate->deltSqr, candidate->bitshift);

    return candidate;
}

#endif
