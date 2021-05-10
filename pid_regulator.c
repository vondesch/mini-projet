/*
 * piD_regulator.c
 *
 *  Created on: May 1, 2021
 *      Author: Loic Von Deschwanden and Raphael Kohler
 */

#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <sensors/imu.h>

#include <motors.h>
#include <pid_regulator.h>

#define MAX_SUM_ERROR 			(MOTOR_SPEED_LIMIT/KI)
#define KP						200.0f
#define KI 						0.05
#define KD						3
//#define ROTATION_THRESHOLD		10
//#define ROTATION_COEFF			10

int16_t pid_regulator(float error) {

	float speed = 0;

	static float sum_error = 0;
	static float previous_error = 0;

	//disables the PID regulator if the error is too small
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
