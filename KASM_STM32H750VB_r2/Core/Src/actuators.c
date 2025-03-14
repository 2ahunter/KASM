/*
 * actuators.c
 *
 *  Created on: Mar 6, 2025
 *      Author: aaronhunter
 */


#include "actuators.h"

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
	){
	actuator->ID = id;
	actuator->phase_port = port;
	actuator->phase_pin = pin;
	actuator->dutycycle = compare_add;
}
