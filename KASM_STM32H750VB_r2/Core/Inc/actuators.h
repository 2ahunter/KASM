/**
  ******************************************************************************
  * @file           : actuators.h
  * @brief          : Header only file for actuator command index
  * @author Aaron Hunter
  *
  * @bug Rev2 actuator board has timer 2 as the clock for the LDC (eddy current sensor)
  * This file defines two versions, one with the LDC and one without. Change the #define to switch
  * between versions. ***Not implemented yet***
  *
  ******************************************************************************
  */

#ifndef INC_ACTUATORS_H_
#define INC_ACTUATORS_H_


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>  /* include for uint32_t type */

/* Defines -------------------------------------------------------------------*/

#define NUM_ACTUATORS 26

/* Typedefs -------------------------------------------------------------------*/


typedef struct actuator_t{
	uint32_t ID;  				// unique identifier for the actuator on the DM
	GPIO_TypeDef *  phase_port; 		// GPIO bank
	uint16_t phase_pin; 		// GPIO pin
	volatile uint32_t * dutycycle;  // dutycycle register address
} actuator_t;


typedef enum actuator_index{
	TIM1_CH1,
	TIM1_CH2,
	TIM1_CH3,
	TIM1_CH4,
	TIM2_CH1,
	TIM4_CH1,
	TIM4_CH2,
	TIM4_CH3,
	TIM4_CH4,
	TIM5_CH2,
	TIM5_CH3,
	TIM8_CH4,
	TIM12_CH2,
	TIM13_CH1,
	TIM14_CH1,
	TIM15_CH1,
	TIM15_CH2,
	TIM16_CH1,
	HRTIM_CHA1,
	HRTIM_CHA2,
	HRTIM_CHB1,
	HRTIM_CHB2,
	HRTIM_CHC1,
	HRTIM_CHC2,
	HRTIM_CHD1,
	HRTIM_CHD2
}actuator_index;   /* actuator index values */

/* Variables -------------------------------------------------------------------*/

/* each bit represents whether the channel is present */
//uint32_t  actuator_config = 0xFFFF >> (BITS - NUM_ACTUATORS);  /* by default enable all actuators */

//#define HAS_LDC 1
//
//#if HAS_LDC /* LDC uses Timer2 channel */
//
//uint32_t config_bitmask = ~(1<<TIM2_CH1); /* bitwise complement creates a mask with timer2 cleared */
//actuator_config = actuator_config & config_bitmask;
//
//#endif /* HAS_LDC */

/* Function prototypes ----------------------------------------------------------*/

/**
 * @function tim_actuator_init
 * @brief initializes a standard timer actuator struct
 * @param actuator : pointer to the struct
 * @param ID : unique identifier of the actuator corresponding to the position in a DM
 * @param port : GPIO bank for the phase pin
 * @param pin : pin in the GPIO bank corresponding to the phase pin
 * @param compare : address of the compare register
 */
void actuator_init(
		actuator_t * actuator,
		uint32_t id,
		GPIO_TypeDef * port,
		uint16_t pin,
		volatile uint32_t * compare_add
	);

#endif /* INC_ACTUATORS_H_ */
