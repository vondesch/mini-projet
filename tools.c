/*
 * tools.c
 *
 *  Created on: May 1, 2021
 *      Author: Loic Von Deschwanden and Raphael Kohler
 */

#include "ch.h"
#include "hal.h"

#include <math.h>
#include <leds.h>
#include <motors.h>

#include "tools.h"

#define KP						200.0f
#define KI 						0.05
#define KD						3
#define MAX_SUM_ERROR 			(MOTOR_SPEED_LIMIT/KI)

int16_t pid_controller(float error) {

	float speed = 0;

	static float sum_error = 0;
	static float previous_error = 0;

	//disables the PID controller if the error is too small
	if (fabs(error) < ERROR_THRESHOLD) {
		error = 0;
		sum_error = 0;
		previous_error = 0;
	}

	sum_error += error;
	previous_error = error - previous_error;

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if (sum_error > MAX_SUM_ERROR) {
		sum_error = MAX_SUM_ERROR;
	} else if (sum_error < -MAX_SUM_ERROR) {
		sum_error = -MAX_SUM_ERROR;
	}

	speed = KP * error + KI * sum_error + KD * previous_error;

	return (int16_t) speed;
}

void led_signal(void) {
	set_body_led(toggle);
	set_front_led(toggle);
}
