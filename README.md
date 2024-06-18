# Project: Button Control with Timer and PWM 
This project demonstrates the control of two buttons using Timer 6, which operates at a refresh rate of 10 Hz. The buttons trigger interrupts that flash LEDs using Pulse Width Modulation (PWM). The LEDs are controlled by Timer 3 using PWM, with the timer set to a refresh rate of 150 Hz.

This project serves as a foundational step towards a larger goal: controlling a DC motor using buttons and PWM. In this future application, the STM microcontroller will receive information from another microcontroller about the surroundings. This data will be sent at a refresh rate of 10 Hz and will be synchronized with a timer that checks for data every 10 Hz.
