/*
 * piD_regulator.c
 *
 *  Created on: 1 mai 2021
 *      Author: Loic
 */

#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <sensors/imu.h>


#include <motors.h>
#include <pid_regulator.h>


#define ERROR_THRESHOLD			1
#define MAX_SUM_ERROR 			(MOTOR_SPEED_LIMIT/KI)
#define KP						200.0f
#define KI 						0.05
#define KD						3
#define ROTATION_THRESHOLD		10
#define ROTATION_COEFF			10


//PID regulator
int16_t pid_regulator(float deviation){

	float error = 0;
	float speed = 0;

	static float sum_error = 0;
	static float previous_error =0;

	error = deviation;

	//disables the PI regulator if the error is to small
	if(fabs(error) < ERROR_THRESHOLD){
		error = 0;
		sum_error = 0;
		previous_error = 0;
	}

	sum_error += error;
	previous_error = error - previous_error;


	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if(sum_error > MAX_SUM_ERROR){
		sum_error = MAX_SUM_ERROR;
	}else if(sum_error < -MAX_SUM_ERROR){
		sum_error = -MAX_SUM_ERROR;
	}

	speed = KP * error + KI * sum_error + KD * previous_error;

    return (int16_t)speed;
}
