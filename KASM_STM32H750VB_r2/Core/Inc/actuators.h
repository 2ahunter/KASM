/**
  ******************************************************************************
  * @file           : actuators.h
  * @brief          : Header only file for actuator command index
  * @author Aaron Hunter
  *
  * @bug Rev2 actuator board has timer 2 as the clock for the LDC (eddy current sensor)
  * This file defines two versions, one with the LDC and one without. Change the #define to switch
  * between versions.
  *
  ******************************************************************************
  */

#ifndef INC_ACTUATORS_H_
#define INC_ACTUATORS_H_

#include <stdint.h>  /* include for uint32_t type */
#define NUM_ACTUATORS 26
#define BITS 32

typedef enum actuators{
	HRTIM_CHA1,
	HRTIM_CHA2,
	HRTIM_CHB1,
	TIM8_CH4,
	TIM2_CH1,
	TIM5_CH2,
	HRTIM_CHB2,
	HRTIM_CHC1,
	TIM_5_CH3,
	TIM13_CH1,
	HRTIM_CHC2,
	HRTIM_CHD1,
	TIM14_CH1,
	TIM1_CH1,
	HRTIM_CHD2,
	TIM4_CH2,
	TIM1_CH2,
	TIM1_CH3,
	TIM16_CH1,
	TIM15_CH1,
	TIM1_CH4,
	TIM12_CH2,
	TIM15_CH2,
	TIM4_CH1,
	TIM4_CH3,
	TIM4_CH4
}actuators;   /* actuator index values */

/* each bit represents whether the channel is present */
uint32_t  actuator_config = 0xFFFF >> (BITS - NUM_ACTUATORS);  /* by default enable all actuators */

//#define HAS_LDC 1
//
//#if HAS_LDC /* LDC uses Timer2 channel */
//
//uint32_t config_bitmask = ~(1<<TIM2_CH1); /* bitwise complement creates a mask with timer2 cleared */
//actuator_config = actuator_config & config_bitmask;
//
//#endif /* HAS_LDC */


#endif /* INC_ACTUATORS_H_ */
