/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 * 这是30.5mm的标准版飞控
 */

#pragma once

#define FC_TARGET_MCU     STM32F405

#define BOARD_NAME        RKF405
#define MANUFACTURER_ID   RuiKong

#define USE_GYRO
#define USE_GYRO_SPI_ICM42688P
#define USE_ACC
#define USE_ACC_SPI_ICM42688P
#define USE_BARO
#define USE_BARO_DPS310
#define USE_BARO_BMP280

#define USE_FLASH
#define USE_FLASH_W25Q128FV

//#define USE_SDCARD

#define USE_MAX7456


#define BEEPER_PIN PB8//蜂鸣器
/*定义电机引脚*/
#define MOTOR1_PIN PC6
#define MOTOR2_PIN PC7
#define MOTOR3_PIN PC8
#define MOTOR4_PIN PC9
#define MOTOR5_PIN PA15
#define MOTOR6_PIN PA8
#define MOTOR7_PIN PB10
#define MOTOR8_PIN PB11

//#define RX_PPM_PIN PB12//PPM信号接收引脚

#define LED_STRIP_PIN PB1//2812LED

#define UART1_TX_PIN PA9
#define UART2_TX_PIN PA2
#define UART3_TX_PIN PC10
#define UART4_TX_PIN PA0
#define UART5_TX_PIN PC12

#define UART1_RX_PIN PA10
#define UART2_RX_PIN PA3
#define UART3_RX_PIN PC11
#define UART4_RX_PIN PA1
#define UART5_RX_PIN PD2

#define I2C1_SCL_PIN PB6
#define I2C1_SDA_PIN PB7

#define LED0_PIN PC13

#define SPI1_SCK_PIN PA5
#define SPI2_SCK_PIN PB13
#define SPI3_SCK_PIN PB3
#define SPI1_SDI_PIN PA6
#define SPI2_SDI_PIN PB14
#define SPI3_SDI_PIN PB4
#define SPI1_SDO_PIN PA7
#define SPI2_SDO_PIN PB15
#define SPI3_SDO_PIN PB5

#define ESCSERIAL_PIN PA3//必须UART的RX

#define ADC_VBAT_PIN PC2
#define ADC_RSSI_PIN PC3//接收机向飞控报告信号质量
#define ADC_CURR_PIN PC1

#define FLASH_CS_PIN PC0
#define MAX7456_SPI_CS_PIN PA13
#define GYRO_1_EXTI_PIN PC4
#define GYRO_1_CS_PIN PA4

//#define USB_DETECT_PIN PB12

#define PINIO1_PIN PC5//目前接VTX开关
//#define PINIO2_PIN PA14
//#define PINIO3_PIN PC15

#define TIMER_PIN_MAPPING \
    TIMER_PIN_MAP( 0, PA3 , 2,  0) \
    TIMER_PIN_MAP( 1, PC6 , 2,  1) \
    TIMER_PIN_MAP( 2, PC7 , 2,  1) \
    TIMER_PIN_MAP( 3, PC8 , 2,  1) \
    TIMER_PIN_MAP( 4, PC9 , 2,  0) \
    TIMER_PIN_MAP( 5, PA15, 1,  0) \
    TIMER_PIN_MAP( 6, PA8 , 1,  1) \
    TIMER_PIN_MAP( 7, PB10, 1,  0) \
    TIMER_PIN_MAP( 8, PB11, 1,  0) \
    TIMER_PIN_MAP( 9, PB1 , 2,  0)

#define ADC_INSTANCE ADC3
#define ADC3_DMA_OPT        1

#define GPS_UART SERIAL_PORT_USART1//自定义GPS的口
#define SERIALRX_UART SERIAL_PORT_USART3//使用 USART3 作为串口，用来接收遥测信号，比如SBUS、IBUS、CRSF（Crossfire）、ELRS 等数字协议。
#define MSP_UART SERIAL_PORT_UART5//地面站（GCS）、OSD、外部控制器

#define MAG_I2C_INSTANCE I2CDEV_1
#define BARO_I2C_INSTANCE I2CDEV_1
#define DEFAULT_BLACKBOX_DEVICE BLACKBOX_DEVICE_FLASH
#define DEFAULT_DSHOT_BURST DSHOT_DMAR_OFF
#define DEFAULT_DSHOT_BITBANG DSHOT_BITBANG_OFF
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SCALE 206
#define BEEPER_INVERTED//蜂鸣器反向触发

#define SYSTEM_HSE_MHZ 8

#define PINIO1_BOX 40
#define PINIO2_BOX 41
#define PINIO3_BOX 42

#define PINIO1_CONFIG 129
#define PINIO2_CONFIG 129

#define FLASH_SPI_INSTANCE SPI3
#define MAX7456_SPI_INSTANCE SPI2
#define SERIALRX_PROVIDER SERIALRX_CRSF
//#define SERIALRX_PROVIDER SERIALRX_SBUS   // SBUS（FrSky、Futaba）
#define GYRO_1_SPI_INSTANCE SPI1
#define GYRO_1_ALIGN CW0_DEG
