/*
        Copyright 2015 Benjamin Vedder	benjamin@vedder.se

        This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#ifndef HW_410E_H_
#define HW_410E_H_

#define HW_NAME "410e"
#define HW_MAJOR 4
#define HW_MINOR 10
#define DISABLE_HW_LIMITS
// Macros
#define ENABLE_GATE() palSetPad(GPIOC, 10)
#define DISABLE_GATE() palClearPad(GPIOC, 10)
#define DCCAL_ON() palSetPad(GPIOB, 12)
#define DCCAL_OFF() palClearPad(GPIOB, 12)
#define IS_DRV_FAULT() (!palReadPad(GPIOC, 12))

#define LED_GREEN_ON() palSetPad(GPIOC, 4)
#define LED_GREEN_OFF() palClearPad(GPIOC, 4)
#define LED_RED_ON() palSetPad(GPIOC, 5)
#define LED_RED_OFF() palClearPad(GPIOC, 5)

/*
 * ADC Vector
 *
 * 0:	ADC1_IN0		  SENS3
 * 1:	ADC2_IN1		  SENS2
 * 2:	ADC3_IN2		  SENS1
 * 3:	ADC1/2_IN8		CURR2
 * 4:	ADC1/2_IN9		CURR1
 * 5:	ADC3_IN3		  TEMP_MOSFET
 * 6:	ADC1_Vrefint  -
 * 7:	ADC2_IN1		  SENS2
 * 8:	ADC3_IN12	    AN_IN
 * 9:	ADC1_IN0		  SENS3
 * 10:ADC2_IN2      SENS1
 * 11:ADC3_IN10 	  TEMP_MOTOR
 */

#define HW_ADC_CHANNELS 12
#define HW_ADC_INJ_CHANNELS 2
/*
Specifies the number of ADC conversions that will be done using the sequencer
for regular channel group.

I'm interpreting this as such that the this should be the highest rank number
used to setup the ADCs. For example if this were set 3 then
TEMP_MOTOR that is set to the rank 4 would not be measured
*/
#define HW_ADC_NBR_CONV 4

/**
 * @brief Change the ADC channels used. Since this board
 supports both low side (from driver) and high side (from instrumentation
 amplifiers)
 *
 */
// #define USE_HIGH_SIDE_CURRENT_MEASURE

// ADC Indexes
#define ADC_IND_SENS1 2
#define ADC_IND_SENS2 1
#define ADC_IND_SENS3 0
#define ADC_IND_CURR1 4
#define ADC_IND_CURR2 3

// ADC Indexes
#define ADC_IND_VIN_SENS 8
#define ADC_IND_EXT 10
#define ADC_IND_EXT2 7
#define ADC_IND_TEMP_MOS 5
#define ADC_IND_TEMP_MOTOR 11
#define ADC_IND_VREFINT 6

// ADC macros and settings

// Component parameters (can be overridden)
#ifndef V_REG
#define V_REG 3.3
#endif
#ifndef VIN_R1
#define VIN_R1 39000.0
#endif
#ifndef VIN_R2
#define VIN_R2 2200.0
#endif
// Override gain and shunt resistance
#ifdef USE_HIGH_SIDE_CURRENT_MEASURE

#define CURRENT_AMP_GAIN (1.0f + (49.4e3f / 5.1e3f))
#define CURRENT_SHUNT_RES 0.008f
#define CURRENT_V_OFFSET (V_REG / 2.0f)
#define GET_CURRENT1()                                                         \
  ((float)(ADC_Value[ADC_IND_CURR1] - (CURRENT_V_OFFSET * (V_REG / 4095.0))))

#define GET_CURRENT2()                                                         \
  ((float)(ADC_Value[ADC_IND_CURR2] - (CURRENT_V_OFFSET * (V_REG / 4095.0))))

#endif
#ifndef CURRENT_AMP_GAIN
#define CURRENT_AMP_GAIN 10.0
#endif
#ifndef CURRENT_SHUNT_RES
#define CURRENT_SHUNT_RES 0.001
#endif

// Input voltage
#define GET_INPUT_VOLTAGE()                                                    \
  ((V_REG / 4095.0) * (float)ADC_Value[ADC_IND_VIN_SENS] *                     \
   ((VIN_R1 + VIN_R2) / VIN_R2))

// Voltage on ADC channel
#define ADC_VOLTS(ch) ((float)ADC_Value[ch] / 4095.0 * V_REG)

// NTC Termistors
#define NTC_RES(adc_val) ((4095.0 * 10000.0) / adc_val - 10000.0)
#define NTC_TEMP(adc_ind)                                                      \
  (1.0 / ((logf(NTC_RES(ADC_Value[adc_ind]) / 10000.0) / 3434.0) +             \
          (1.0 / 298.15)) -                                                    \
   273.15)

#define NTC_RES_MOTOR(adc_val)                                                 \
  (10000.0 / ((4095.0 / (float)adc_val) - 1.0)) // Motor temp sensor on low side
#define NTC_TEMP_MOTOR(beta)                                                   \
  (1.0 / ((logf(NTC_RES_MOTOR(ADC_Value[ADC_IND_TEMP_MOTOR]) / 10000.0) /      \
           beta) +                                                             \
          (1.0 / 298.15)) -                                                    \
   273.15)

// Double samples in beginning and end for positive current measurement.
// Useful when the shunt sense traces have noise that causes offset.
#ifndef CURR1_DOUBLE_SAMPLE
#define CURR1_DOUBLE_SAMPLE 0
#endif
#ifndef CURR2_DOUBLE_SAMPLE
#define CURR2_DOUBLE_SAMPLE 0
#endif

// UART Peripheral
#define HW_UART_DEV SD3
#define HW_UART_GPIO_AF GPIO_AF_USART3
#define HW_UART_TX_PORT GPIOB
#define HW_UART_TX_PIN 10
#define HW_UART_RX_PORT GPIOB
#define HW_UART_RX_PIN 11

// ICU Peripheral for servo decoding
#define HW_ICU_TIMER TIM4
#define HW_ICU_TIM_CLK_EN() RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE)
#define HW_ICU_DEV ICUD4
#define HW_ICU_CHANNEL ICU_CHANNEL_1
#define HW_ICU_GPIO_AF GPIO_AF_TIM4
#define HW_ICU_GPIO GPIOB
#define HW_ICU_PIN 6

// I2C Peripheral
#define HW_I2C_DEV I2CD2
#define HW_I2C_GPIO_AF GPIO_AF_I2C2
#define HW_I2C_SCL_PORT GPIOB
#define HW_I2C_SCL_PIN 10
#define HW_I2C_SDA_PORT GPIOB
#define HW_I2C_SDA_PIN 11

// Taking advantage of existing code.
#define HW_USE_SERVO_TIM4 // disables IRQ generation for TIM3, lets LLD handle
                          // them

// Encoder dedicated pins
#define HW_HALL_ENC_GPIO1 GPIOC
#define HW_HALL_ENC_PIN1 6
#define HW_HALL_ENC_GPIO2 GPIOC
#define HW_HALL_ENC_PIN2 7
#define HW_HALL_ENC_GPIO3 GPIOC
#define HW_HALL_ENC_PIN3 8
#define HW_ENC_TIM TIM3
#define HW_ENC_TIM_AF GPIO_AF_TIM3
#define HW_ENC_TIM_CLK_EN() RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE)
#define HW_ENC_EXTI_PORTSRC EXTI_PortSourceGPIOC
#define HW_ENC_EXTI_PINSRC EXTI_PinSource8
#define HW_ENC_EXTI_CH EXTI9_5_IRQn
#define HW_ENC_EXTI_LINE EXTI_Line8
#define HW_ENC_EXTI_ISR_VEC EXTI9_5_IRQHandler
#define HW_ENC_TIM_ISR_CH TIM3_IRQn
#define HW_ENC_TIM_ISR_VEC TIM3_IRQHandler

// Hall Sensors dedicated pins
#define HW_HALL_GPIO1 GPIOB
#define HW_HALL_PIN1 5
#define HW_HALL_GPIO2 GPIOB
#define HW_HALL_PIN2 7
#define HW_HALL_GPIO3 GPIOC
#define HW_HALL_PIN3 11

/**
 * @brief SPI NSS and SCK pins are used as DAC outputs
 *
 */
// SPI pins
#define HW_SPI_DEV SPID1
#define HW_SPI_GPIO_AF GPIO_AF_SPI1
#define HW_SPI_PORT_NSS GPIOA
#define HW_SPI_PIN_NSS 4
#define HW_SPI_PORT_SCK GPIOA
#define HW_SPI_PIN_SCK 5
#define HW_SPI_PORT_MOSI GPIOA
#define HW_SPI_PIN_MOSI 7
#define HW_SPI_PORT_MISO GPIOA
#define HW_SPI_PIN_MISO 6

// Measurement macros
#define ADC_V_L1 ADC_Value[ADC_IND_SENS1]
#define ADC_V_L2 ADC_Value[ADC_IND_SENS2]
#define ADC_V_L3 ADC_Value[ADC_IND_SENS3]
#define ADC_V_ZERO (ADC_Value[ADC_IND_VIN_SENS] / 2)

// Macros
#define READ_HALL1() palReadPad(HW_HALL_GPIO1, HW_HALL_PIN1)
#define READ_HALL2() palReadPad(HW_HALL_GPIO2, HW_HALL_PIN2)
#define READ_HALL3() palReadPad(HW_HALL_GPIO3, HW_HALL_PIN3)

// Setting limits
#define HW_LIM_CURRENT -100.0, 100.0
#define HW_LIM_CURRENT_IN -100.0, 100.0
#define HW_LIM_CURRENT_ABS 0.0, 150.0
#define HW_LIM_VIN 6.0, 57.0
#define HW_LIM_ERPM -200e3, 200e3
#define HW_LIM_DUTY_MIN 0.0, 0.1
#define HW_LIM_DUTY_MAX 0.0, 0.95
#define HW_LIM_TEMP_FET -40.0, 110.0

#endif /* HW_410E_H_ */
