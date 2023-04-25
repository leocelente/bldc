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

#include "hw.h"

#include "ch.h"
#include "hal.h"
#include "pal.h"
#include "stm32f4xx_conf.h"
#include "vesc4/hw_410_LACEP.h"

// Variables
static volatile bool i2c_running = false;

// I2C configuration
static const I2CConfig i2cfg = {OPMODE_I2C, 100000, STD_DUTY_CYCLE};

void hw_init_gpio(void) {
  // GPIO clock enable
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

  // LEDs
  // LED_GREEN
  palSetPadMode(GPIOC, 4, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
  // LED_RED
  palSetPadMode(GPIOC, 5, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);

  // RS485
  // DE
  palSetPadMode(HW_RS485_DE_PORT, HW_RS485_DE_PIN,
                PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
  // RE
  palSetPadMode(HW_RS485_RE_PORT, HW_RS485_RE_PIN,
                PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);

  palWritePad(HW_RS485_DE_PORT, HW_RS485_DE_PIN, 0);
  palWritePad(HW_RS485_RE_PORT, HW_RS485_RE_PIN, 0);

  // GPIOC (ENABLE_GATE)
  palSetPadMode(GPIOC, 10, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
  DISABLE_GATE();

  // GPIOB (DCCAL)
  palSetPadMode(GPIOB, 12, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);

  // GPIOA Configuration: Channel 1 to 3 as alternate function push-pull
  palSetPadMode(GPIOA, 8,
                PAL_MODE_ALTERNATE(GPIO_AF_TIM1) | PAL_STM32_OSPEED_HIGHEST |
                    PAL_STM32_PUDR_FLOATING); // PWM_H3
  palSetPadMode(GPIOA, 9,
                PAL_MODE_ALTERNATE(GPIO_AF_TIM1) | PAL_STM32_OSPEED_HIGHEST |
                    PAL_STM32_PUDR_FLOATING); // PWM_H2
  palSetPadMode(GPIOA, 10,
                PAL_MODE_ALTERNATE(GPIO_AF_TIM1) | PAL_STM32_OSPEED_HIGHEST |
                    PAL_STM32_PUDR_FLOATING); // PWM_H1

  palSetPadMode(GPIOB, 13,
                PAL_MODE_ALTERNATE(GPIO_AF_TIM1) | PAL_STM32_OSPEED_HIGHEST |
                    PAL_STM32_PUDR_FLOATING); // PWM_L3
  palSetPadMode(GPIOB, 14,
                PAL_MODE_ALTERNATE(GPIO_AF_TIM1) | PAL_STM32_OSPEED_HIGHEST |
                    PAL_STM32_PUDR_FLOATING); // PWM_L2
  palSetPadMode(GPIOB, 15,
                PAL_MODE_ALTERNATE(GPIO_AF_TIM1) | PAL_STM32_OSPEED_HIGHEST |
                    PAL_STM32_PUDR_FLOATING); // PWM_L1

  // Hall sensors
  palSetPadMode(HW_HALL_GPIO1, HW_HALL_PIN1, PAL_MODE_INPUT_PULLUP);
  palSetPadMode(HW_HALL_GPIO2, HW_HALL_PIN2, PAL_MODE_INPUT_PULLUP);
  palSetPadMode(HW_HALL_GPIO3, HW_HALL_PIN3, PAL_MODE_INPUT_PULLUP);

  // Fault pin
  palSetPadMode(GPIOC, 12, PAL_MODE_INPUT_PULLUP);

  // ADC Pins
  palSetPadMode(GPIOA, 0, PAL_MODE_INPUT_ANALOG); // SENS3
  palSetPadMode(GPIOA, 1, PAL_MODE_INPUT_ANALOG); // SENS2
  palSetPadMode(GPIOA, 2, PAL_MODE_INPUT_ANALOG); // SENS1

  // Optinal ADC Inputs REMOVED
  // palSetPadMode(GPIOA, 4, PAL_MODE_INPUT_ANALOG); // TX_SDA_NSS rerouted as
  // SENS palSetPadMode(GPIOA, 5, PAL_MODE_INPUT_ANALOG); // ADC_EXT  rerouted
  // as SENS palSetPadMode(GPIOA, 6, PAL_MODE_INPUT_ANALOG); // ADC_EXT2
  // rerouted as SENS

#ifdef USE_AMP_CURRENT_SENS
  palSetPadMode(GPIOA, 7, PAL_MODE_INPUT_ANALOG); // CURR2
  palSetPadMode(GPIOA, 6, PAL_MODE_INPUT_ANALOG); // CURR1
#else
  palSetPadMode(GPIOB, 0, PAL_MODE_INPUT_ANALOG); // CURR2
  palSetPadMode(GPIOB, 1, PAL_MODE_INPUT_ANALOG); // CURR1
#endif

  palSetPadMode(GPIOC, 0, PAL_MODE_INPUT_ANALOG); // TEMP_MOTOR
  palSetPadMode(GPIOA, 3, PAL_MODE_INPUT_ANALOG); // ADC_TEMP
  palSetPadMode(GPIOC, 2, PAL_MODE_INPUT_ANALOG); // AN_IN
}

void hw_setup_adc_channels(void) {
  /**
   * The ADC configuration in this project works by running a DMA channel on all
   ADCs. The DMA will populate the samples to the `ADC_Values` global variable.
   The order they'll appear in the buffer is determined by the Rank set and the
   ADCx number
   *
   */

  // SENS3
  ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_15Cycles);

  // CURR2
#ifdef USE_AMP_CURRENT_SENS
  ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 2, ADC_SampleTime_15Cycles);
#else
  ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 2, ADC_SampleTime_15Cycles);
#endif

  // Vrefint
  ADC_RegularChannelConfig(ADC1, ADC_Channel_Vrefint, 3,
                           ADC_SampleTime_15Cycles);

  //! TX_SDA_NSS routed as SENS3
  ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 4, ADC_SampleTime_15Cycles);

  // ADC2 Channels

  // SENS2
  ADC_RegularChannelConfig(ADC2, ADC_Channel_1, 1, ADC_SampleTime_15Cycles);

  // CURR1
#ifdef USE_AMP_CURRENT_SENS
  ADC_RegularChannelConfig(ADC2, ADC_Channel_6, 2, ADC_SampleTime_15Cycles);
#else
  ADC_RegularChannelConfig(ADC2, ADC_Channel_9, 2, ADC_SampleTime_15Cycles);
#endif

  //! ADC_EXT2 routed as SENS2
  ADC_RegularChannelConfig(ADC2, ADC_Channel_1, 3, ADC_SampleTime_15Cycles);
  //! ADC_EXT routed as SENS1
  ADC_RegularChannelConfig(ADC2, ADC_Channel_2, 4, ADC_SampleTime_15Cycles);

  // ADC3 Channels

  // SENS1
  ADC_RegularChannelConfig(ADC3, ADC_Channel_2, 1, ADC_SampleTime_15Cycles);

  // TEMP_MOSFET (ADC_TEMP)
  ADC_RegularChannelConfig(ADC3, ADC_Channel_3, 2, ADC_SampleTime_15Cycles);
  // AN_IN
  ADC_RegularChannelConfig(ADC3, ADC_Channel_12, 3, ADC_SampleTime_15Cycles);
  // TEMP_MOTOR
  ADC_RegularChannelConfig(ADC3, ADC_Channel_10, 4, ADC_SampleTime_15Cycles);

  /**
   * All the previous settings make sense, they align with exactly one pin and
   * usage. BUT, these InjectedChannels They have higher priority, so current
   * measurements are more imediate and synchronized to avoid noise. Its is my
   * hypothesis that they are each configured twice to couple them (sync them)
   * no matter which triggers the event
   */
#ifdef USE_AMP_CURRENT_SENS
  ADC_InjectedChannelConfig(ADC1, ADC_Channel_6, 1, ADC_SampleTime_15Cycles);
  ADC_InjectedChannelConfig(ADC1, ADC_Channel_7, 2, ADC_SampleTime_15Cycles);
  ADC_InjectedChannelConfig(ADC2, ADC_Channel_7, 1, ADC_SampleTime_15Cycles);
  ADC_InjectedChannelConfig(ADC2, ADC_Channel_6, 2, ADC_SampleTime_15Cycles);
#else
  // Injected channels
  ADC_InjectedChannelConfig(ADC1, ADC_Channel_9, 1, ADC_SampleTime_15Cycles);
  ADC_InjectedChannelConfig(ADC1, ADC_Channel_8, 2, ADC_SampleTime_15Cycles);
  ADC_InjectedChannelConfig(ADC2, ADC_Channel_8, 1, ADC_SampleTime_15Cycles);
  ADC_InjectedChannelConfig(ADC2, ADC_Channel_9, 2, ADC_SampleTime_15Cycles);
#endif
}

void hw_start_i2c(void) {
  i2cAcquireBus(&HW_I2C_DEV);

  if (!i2c_running) {
    palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN,
                  PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
                      PAL_STM32_OTYPE_OPENDRAIN | PAL_STM32_OSPEED_MID1 |
                      PAL_STM32_PUDR_PULLUP);
    palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN,
                  PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
                      PAL_STM32_OTYPE_OPENDRAIN | PAL_STM32_OSPEED_MID1 |
                      PAL_STM32_PUDR_PULLUP);

    i2cStart(&HW_I2C_DEV, &i2cfg);
    i2c_running = true;
  }

  i2cReleaseBus(&HW_I2C_DEV);
}

void hw_stop_i2c(void) {
  i2cAcquireBus(&HW_I2C_DEV);

  if (i2c_running) {
    palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN, PAL_MODE_INPUT);
    palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN, PAL_MODE_INPUT);

    i2cStop(&HW_I2C_DEV);
    i2c_running = false;
  }

  i2cReleaseBus(&HW_I2C_DEV);
}

/**
 * Try to restore the i2c bus
 */
void hw_try_restore_i2c(void) {
  if (i2c_running) {
    i2cAcquireBus(&HW_I2C_DEV);

    palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN,
                  PAL_STM32_OTYPE_OPENDRAIN | PAL_STM32_OSPEED_MID1 |
                      PAL_STM32_PUDR_PULLUP);

    palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN,
                  PAL_STM32_OTYPE_OPENDRAIN | PAL_STM32_OSPEED_MID1 |
                      PAL_STM32_PUDR_PULLUP);

    palSetPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
    palSetPad(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN);

    chThdSleep(1);

    for (int i = 0; i < 16; i++) {
      palClearPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
      chThdSleep(1);
      palSetPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
      chThdSleep(1);
    }

    // Generate start then stop condition
    palClearPad(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN);
    chThdSleep(1);
    palClearPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
    chThdSleep(1);
    palSetPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
    chThdSleep(1);
    palSetPad(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN);

    palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN,
                  PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
                      PAL_STM32_OTYPE_OPENDRAIN | PAL_STM32_OSPEED_MID1 |
                      PAL_STM32_PUDR_PULLUP);

    palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN,
                  PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
                      PAL_STM32_OTYPE_OPENDRAIN | PAL_STM32_OSPEED_MID1 |
                      PAL_STM32_PUDR_PULLUP);

    HW_I2C_DEV.state = I2C_STOP;
    i2cStart(&HW_I2C_DEV, &i2cfg);

    i2cReleaseBus(&HW_I2C_DEV);
  }
}
