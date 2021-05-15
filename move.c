/*
 * move.c
 *
 *  Created on: May 4, 2021
 *      Author: Loic Von Deschwanden and Raphael Kohler
 */

#include "ch.h"
#include "hal.h"

#include <main.h>
#include <motors.h>
#include <sensors/imu.h>
#include <leds.h>
#include <selector.h>

#include "move.h"
#include "tools.h"
#include "detect_obstacle.h"

#define MOTOR_OBSTACLE 		400		//nb step of the motor to set the next position
#define COEFF_ROT			0.8f	//little rotation while moving around the obstacle
#define NB_SAMPLE			5		//nb sample for the mean value
#define AXIS				2		//axis X and Y
#define KP_Y				200		//coefficient for the rotation controller when the Y acceleration is negative
#define SLEEP_5				5		//sleep 5ms
#define SLEEP_10			10		//sleep 10ms

//different speed for the selector
//speed[cm/s]=[steps/s]/[steps/turn]*[cm/turn]
#define SPEED0 				200 	//2.6 cm/s
#define SPEED1 				400		//5.2 cm/s
#define SPEED2 				500		//6.5 cm/s
#define SPEED3 				600		//7.8 cm/s
#define SPEED4 				800		//10.4 cm/s



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

	int16_t speed_pid;//speed rectification to go in the right direction (PID controller X acceleration)
	int16_t speed_neg_slope;//speed rectification for negative Y acceleration

	while (1) {

		//no obstacle in front of robot
		if (get_free_path() == straight) {

			speed_pid = pid_controller(mean_acc[X_AXIS]);

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
			while (get_free_path() == left) {
				left_motor_set_speed(-speed);
				right_motor_set_speed(speed);
			}

			//wait to turn a little bit more and have a free way
			chThdSleepMilliseconds(SLEEP_5);

			//go straight with a slight rotation to pass the obstacle
			int32_t pos_motor_left = left_motor_get_pos() + MOTOR_OBSTACLE;
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
			while (get_free_path() == right) {
				left_motor_set_speed(speed);
				right_motor_set_speed(-speed);
			}

			//wait to turn a little bit more and have a free way
			chThdSleepMilliseconds(SLEEP_5);

			//go straight with a slight rotation to pass the obstacle
			int32_t pos_motor_right = right_motor_get_pos() + MOTOR_OBSTACLE;
			while (right_motor_get_pos() <= pos_motor_right
					&& get_free_path() == straight) {
				left_motor_set_speed(COEFF_ROT * speed);
				right_motor_set_speed(speed);
			}

			//clear led obstacle
			led_signal();
		} else if (get_free_path() == stop) {
			left_motor_set_speed(0);
			right_motor_set_speed(0);
		}

		chThdSleepMilliseconds(SLEEP_10);
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
		chThdSleepMilliseconds(SLEEP);
	}
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

