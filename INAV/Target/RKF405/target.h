/*
 * This file is part of INAV.
 *
 * INAV is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * INAV is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with INAV.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once
 
#define USE_TARGET_CONFIG

#define TARGET_BOARD_IDENTIFIER "RKF405"
#define USBD_PRODUCT_STRING     "RuiKongF405"

// ******** Board LEDs  **********************
#define LED0                            PC13

// ******* Beeper ***********
#define BEEPER                          PB8
#define BEEPER_INVERTED


// ******* GYRO and ACC ********
#define USE_IMU_ICM42605
#define IMU_ICM42605_ALIGN               CW0_DEG
#define ICM42605_SPI_BUS                 BUS_SPI1
#define ICM42605_CS_PIN                  PA4


// *************** Baro **************************
#define USE_I2C
#define USE_I2C_DEVICE_1

#define I2C1_SCL                        PB6        // SCL
#define I2C1_SDA                        PB7        // SDA
#define DEFAULT_I2C_BUS                 BUS_I2C1

#define USE_BARO
#define BARO_I2C_BUS                    DEFAULT_I2C_BUS

#define USE_BARO_DPS310

//*********** Magnetometer / Compass *************
#define USE_MAG
#define MAG_I2C_BUS                     DEFAULT_I2C_BUS
#define USE_MAG_ALL

// ******* SERIAL ********
#define USE_VCP
#define VBUS_SENSING_PIN        PB12
#define VBUS_SENSING_ENABLED

#define USE_UART1
#define UART1_TX_PIN                    PA9
#define UART1_RX_PIN                    PA10

#define USE_UART2
#define UART2_TX_PIN                    PA2
#define UART2_RX_PIN                    PA3

#define USE_UART3
#define UART3_TX_PIN                    PC10
#define UART3_RX_PIN                    PC11

#define USE_UART4
#define UART4_TX_PIN                    PA0
#define UART4_RX_PIN                    PA1

#define USE_UART5
#define UART5_TX_PIN                    PC12
#define UART5_RX_PIN                    PD2


#define SERIAL_PORT_COUNT              6


// ******* SPI ********
#define USE_SPI

#define USE_SPI_DEVICE_1
#define SPI1_NSS_PIN                    PA4
#define SPI1_SCK_PIN                    PA5
#define SPI1_MISO_PIN                   PA6
#define SPI1_MOSI_PIN                   PA7

#define USE_SPI_DEVICE_2
#define SPI2_NSS_PIN                    PA13
#define SPI2_SCK_PIN                    PB13
#define SPI2_MISO_PIN                   PB14
#define SPI2_MOSI_PIN                   PB15

#define USE_SPI_DEVICE_3
#define SPI3_NSS_PIN                    PC0
#define SPI3_SCK_PIN                    PB3
#define SPI3_MISO_PIN                   PB4
#define SPI3_MOSI_PIN                   PB5

// ******* ADC ********
#define USE_ADC
#define ADC_CHANNEL_1_PIN               PC1
#define ADC_CHANNEL_2_PIN               PC2
#define ADC_CHANNEL_3_PIN               PC3

#define VBAT_ADC_CHANNEL                ADC_CHN_2
#define RSSI_ADC_CHANNEL                ADC_CHN_3
#define CURRENT_METER_ADC_CHANNEL       ADC_CHN_1

#define VBAT_SCALE_DEFAULT              1100
#define CURRENT_METER_SCALE             206

// ******* OSD ********
#define USE_OSD
#define USE_MAX7456
#define MAX7456_SPI_BUS                 BUS_SPI2
#define MAX7456_CS_PIN                  PA13

//******* FLASH ********

#define USE_FLASHFS
#define USE_FLASH_M25P16
#define M25P16_SPI_BUS                  BUS_SPI3
#define M25P16_CS_PIN                   PC0
#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT

// *************** PINIO ***************************
#define USE_PINIO
#define USE_PINIOBOX
#define PINIO1_PIN                  PC5  // VTX power switcher
#define PINIO2_PIN                  PA14  //bluetooth
#define PINIO3_PIN                  PC15 //Camera control
#define PINIO1_FLAGS				PINIO_FLAGS_INVERTED
#define PINIO2_FLAGS				PINIO_FLAGS_INVERTED

//************ LEDSTRIP *****************
#define USE_LED_STRIP
#define WS2811_PIN                      PB1

// ******* FEATURES ********
#define DEFAULT_RX_TYPE                 RX_TYPE_SERIAL
#define SERIALRX_UART                   SERIAL_PORT_USART2
#define SERIALRX_PROVIDER               SERIALRX_CRSF

#define DEFAULT_FEATURES                (FEATURE_OSD | FEATURE_TELEMETRY | FEATURE_CURRENT_METER | FEATURE_VBAT | FEATURE_TX_PROF_SEL | FEATURE_GPS | FEATURE_BLACKBOX | FEATURE_LED_STRIP)

#define TARGET_IO_PORTA                 0xffff
#define TARGET_IO_PORTB                 0xffff
#define TARGET_IO_PORTC                 0xffff
#define TARGET_IO_PORTD                 0xffff

#define MAX_PWM_OUTPUT_PORTS            10

// ESC-related features
#define USE_DSHOT
#define USE_SERIALSHOT
#define USE_ESC_SENSOR
#define USE_SERIAL_4WAY_BLHELI_INTERFACE
