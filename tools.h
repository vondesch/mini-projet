/*
 * tools.h
 *
 *  Created on: May 1, 2021
 *      Author: Loic Von Deschwanden and Raphael Kohler
 */

#ifndef TOOLS_H_
#define TOOLS_H_

#define ERROR_THRESHOLD			1		//admissible error of the accelerometer

enum {
	off, on, toggle		//state of the LED
};

/**
 * @brief 	toggles front LED as well as body LED
 */
void led_signal(void);

/**
 * @brief 	PID controller that corrects the speed to prevent a halting movement
 */
int16_t pid_controller(float error);

#endif /* TOOLS_H_ */
