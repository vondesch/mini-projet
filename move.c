/*
 * move.c
 *
 *  Created on: May 4, 2021
 *      Author: Loic Von Deschwanden and Raphael Kohler
 */

#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>

#include <main.h>
#include <motors.h>
#include <move.h>
#include <sensors/imu.h>
//#include <sensors/proximity.h>
#include <pid_regulator.h>
#include <leds.h>
#include <selector.h>
#include "detect_obstacle.h"

void led_signal(void);

#define MOTOR_OBSTACLE 		400		//nb step of the motor to set the next position
#define COEFF_ROT			0.8f	//little rotation while moving around the obstacle
#define NB_SAMPLE			5		//nb sample for the mean value
#define AXIS				2		//axis X and Y
#define KP_Y				200		//coefficient for the rotation regulator when the Y acceleration is negative

//different speed for the selector
#define SPEED0 				200
#define SPEED1 				400
#define SPEED2 				500
#define SPEED3 				600
#define SPEED4 				800

enum {
	off, on, toggle		//state of the LED
};

static float mean_acc[AXIS];		//mean acceleration
static uint16_t speed;				//speed of the robot

/**
 * @brief 	Thread that coordinates the movement of the robot
 *
 */
static THD_WORKING_AREA(waMoveThd, 128);
static THD_FUNCTION(MoveThd, arg) {
	(void) arg;
	chRegSetThreadName(__FUNCTION__);

	int32_t pos_motor_right;
	int32_t pos_motor_left;

	int16_t speed_pid;//speed rectification to go in the right direction (PID regulator X acceleration)
	int16_t speed_neg_slope;//speed rectification for negative Y acceleration

	while (1) {

		//no obstacle in front of robot
		if (get_free_path() == straight) {

			speed_pid = pid_regulator(mean_acc[X_AXIS]);

			//accelerate the rotation if the robot is in the opposite direction
			if (mean_acc[Y_AXIS] < -ERROR_THRESHOLD) {
				speed_neg_slope = mean_acc[Y_AXIS] * KP_Y;
			} else
				speed_neg_slope = 0;

			//move the robot in the right direction
			if (mean_acc[X_AXIS] > 0) {
				//clockwise rotation or straight
				left_motor_set_speed(speed + speed_pid - speed_neg_slope);
				right_motor_set_speed(speed - speed_pid + speed_neg_slope);
			} else {
				//counter clockwise rotation or straight
				left_motor_set_speed(speed + speed_pid + speed_neg_slope);
				right_motor_set_speed(speed - speed_pid - speed_neg_slope);
			}
		}

		//obstacle detected on the right
		else if (get_free_path() == left) {	//go left

			//set led to signal an obstacle
			led_signal();

			//clockwise rotation until no obstacle is in front of the robot
			while (obstacle_in_range(FRONTRIGHT45)
					|| obstacle_in_range(FRONTRIGHT)) {
				left_motor_set_speed(-speed);
				right_motor_set_speed(speed);
			}

			//wait to turn a little bit more and have a free way
			chThdSleepMilliseconds(5);

			//go straight with a slight rotation to pass the obstacle
			pos_motor_left = left_motor_get_pos() + MOTOR_OBSTACLE;
			while (left_motor_get_pos() <= pos_motor_left
					&& get_free_path() == straight) {
				left_motor_set_speed(speed);
				right_motor_set_speed(COEFF_ROT * speed);
			}

			//clear led obstacle
			led_signal();
		}

		//obstacle detected on the left
		else if (get_free_path() == right) {

			//set led to signal an obstacle
			led_signal();

			//right rotation until no obstacle in front
			while (obstacle_in_range(FRONTLEFT45)
					|| obstacle_in_range(FRONTLEFT)) {
				left_motor_set_speed(speed);
				right_motor_set_speed(-speed);
			}

			//wait to turn a little bit more and have a free way
			chThdSleepMilliseconds(5);

			//go straight with a slight rotation to pass the obstacle
			pos_motor_right = right_motor_get_pos() + MOTOR_OBSTACLE;
			while (right_motor_get_pos() <= pos_motor_right
					&& get_free_path() == straight) {
				left_motor_set_speed(COEFF_ROT * speed);		// magic number
				right_motor_set_speed(speed);
			}

			//clear led obstacle
			led_signal();
		}
//		else {
//			left_motor_set_speed(0);
//			right_motor_set_speed(0);
//		}

		chThdSleepMilliseconds(10);
	}
}

/**
 * @brief 	Thread that calculates the mean of several acceleration values along x-and y-direction
 * 			The averaged values are stored in the array mean_acc
 */
static THD_WORKING_AREA(waMeanAccThd, 128);
static THD_FUNCTION(MeanAccThd, arg) {
	(void) arg;
	chRegSetThreadName(__FUNCTION__);

	messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus,
			"/imu");
	imu_msg_t imu_values;

	//table to store acceleration sample of X and Y
	static float accel[AXIS][NB_SAMPLE];

	float sum_x_acc = 0;
	float sum_y_acc = 0;

	uint8_t sample = 0;

	//reset accel
	for (uint8_t i = 0; i < AXIS; i++) {
		for (sample = 0; sample < NB_SAMPLE; sample++) {
			accel[i][sample] = 0;
		}
	}

	sample = 0;

	while (1) {
		//wait imu_values
		messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));

		//subtract oldest acceleration values
		sum_x_acc -= accel[X_AXIS][sample];
		sum_y_acc -= accel[Y_AXIS][sample];

		//update acceleration values
		accel[X_AXIS][sample] = imu_values.acceleration[X_AXIS];
		accel[Y_AXIS][sample] = imu_values.acceleration[Y_AXIS];

		//add updated acceleration values
		sum_x_acc += accel[X_AXIS][sample];
		sum_y_acc += accel[Y_AXIS][sample];

		//calculate mean
		mean_acc[X_AXIS] = sum_x_acc / NB_SAMPLE;
		mean_acc[Y_AXIS] = sum_y_acc / NB_SAMPLE;

		sample++;

		if (sample >= NB_SAMPLE) {
			sample = 0;
		}
	}
}

/**
 * @brief 	Thread that checks if the selector has been rotated and updates the speed accordingly
 *
 */
static THD_WORKING_AREA(waSpeedSelectThd, 128);
static THD_FUNCTION(SpeedSelectThd, arg) {
	(void) arg;
	chRegSetThreadName(__FUNCTION__);

	while (1) {

		// set the selected speed and indicate usinig LEDS
		switch (get_selector()) {
		case 0:
			set_led(LED1, on);
			set_led(LED3, off);
			set_led(LED5, off);
			set_led(LED7, off);
			speed = SPEED0;
			break;

		case 1:
			set_led(LED1, off);
			set_led(LED3, on);
			set_led(LED5, off);
			set_led(LED7, off);
			speed = SPEED1;
			break;

		case 2:	// default speed - all LEDS on
			set_led(LED1, on);
			set_led(LED3, on);
			set_led(LED5, on);
			set_led(LED7, on);
			speed = SPEED2;
			break;

		case 3:
			set_led(LED1, off);
			set_led(LED3, off);
			set_led(LED5, on);
			set_led(LED7, off);
			speed = SPEED3;
			break;

		case 4:
			set_led(LED1, off);
			set_led(LED3, off);
			set_led(LED5, off);
			set_led(LED7, on);
			speed = SPEED4;
			break;

		default:
			set_led(LED1, on);
			set_led(LED3, on);
			set_led(LED5, on);
			set_led(LED7, on);
			speed = SPEED2;
			break;
		}
		chThdSleepMilliseconds(1000);
	}
}

void led_signal(void) {
	set_body_led(toggle);
	set_front_led(toggle);
}

void speed_select_start() {
	chThdCreateStatic(waSpeedSelectThd, sizeof(waSpeedSelectThd), NORMALPRIO,
			SpeedSelectThd, NULL);

}

void move_start() {
	chThdCreateStatic(waMoveThd, sizeof(waMoveThd), NORMALPRIO, MoveThd, NULL);
	chThdCreateStatic(waMeanAccThd, sizeof(waMeanAccThd), NORMALPRIO,
			MeanAccThd, NULL);

}

