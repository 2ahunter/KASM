# KASM
## Keck Adaptive Secondary Mirror Control

This repository contains firmware for the hardware elements for the Keck adaptive secondary mirror project. 
## Files:
### KASM_STM32H750VB_r2: 
Baseline working PWM-based actuator driver with SPI interface
### Lwip_bare_2: 
KASM interface PCB. It uses the LWIP ethernet driver to receive a command over a UDP message through a socket, parses the message into separate SPI commands. Can be converted to talk to the KASM_STM32H750VB_r2 board, or speak directly to other SPI devices (e.g., DACs).
### KASM_FilterTesting:
Modified version of KASM_STM32H750VB_r2, probably obsolete at this point and should be removed
### KASM_HRTIM_Test1:
Modified version of KASM_STM32H750VB_r2, also probably obsolete
### KASM_STM32H750VB:
Original baseline code for the STM32H750VB-based pwm driver board, also probably obsolete
### utilities
Useful utilities including circular buffers and crc algorithms
