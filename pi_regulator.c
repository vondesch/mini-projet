/*
 * pi_regulator.c
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
#include <pi_regulator.h>


#define ERROR_THRESHOLD			0.5f
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
	//this avoids to always move as we cannot exactly be where we want and
	//the camera is a bit noisy
	if(fabs(error) < ERROR_THRESHOLD){
		return 0;
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

//static THD_WORKING_AREA(waPiRegulator, 256);
//static THD_FUNCTION(PiRegulator, arg) {
//
//    chRegSetThreadName(__FUNCTION__);
//    (void)arg;
//
//    systime_t time;
//
//    int16_t speed = 0;
//    int16_t speed_correction = 0;
//
//    while(1){
//        time = chVTGetSystemTime();
//
//        //computes the speed to give to the motors
//        //distance_cm is modified by the image processing thread
//        speed = pi_regulator(get_acceleration(X_AXIS));
//        //computes a correction factor to let the robot rotate to be in front of the line
//        speed_correction = (get_acceleration(X_AXIS));
//
//        //if the line is nearly in front of the camera, don't rotate
//        if(abs(speed_correction) < ROTATION_THRESHOLD){
//        	speed_correction = 0;
//        }
//
//        //applies the speed from the PI regulator and the correction for the rotation
//		right_motor_set_speed(speed - ROTATION_COEFF * speed_correction);
//		left_motor_set_speed(speed + ROTATION_COEFF * speed_correction);
//
//        //100Hz
//        chThdSleepUntilWindowed(time, time + MS2ST(10));
//    }
//}
//
//void pi_regulator_start(void){
//	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
//}


