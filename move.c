/*
 * move.c
 *
 *  Created on: 4 mai 2021
 *      Author: Loic
 */

#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <main.h>
#include <motors.h>
#include <move.h>
#include <sensors/imu.h>
#include <sensors/proximity.h>
#include <wallDetect.h>
#include <pi_regulator.h>


#define NB_SAMPLES_OFFSET 	200
#define MOTOR_OBSTACLE 		400
#define COEFF_ROT			0.8f
#define error  				1
#define NB_SAMPLE			5
#define AXIS				1		//axis X and Y
#define KI_Y				200

static float mean_acc[AXIS];


static THD_WORKING_AREA(waMoveThd, 128);
static THD_FUNCTION(MoveThd, arg) {
	(void) arg;
	chRegSetThreadName(__FUNCTION__);

	uint16_t speed = 600;

	uint16_t pos_motor_right;
	uint16_t pos_motor_left;
	int16_t speed_pid;
	int16_t speed_neg_slope;
//	static uint16_t i = 0;
//	static uint16_t j = 0;

	while (1) {

		if (get_free_path() == straight ) { //no obstacle in front of robot
			speed_pid = pid_regulator(mean_acc[X_AXIS]);
			//clockwise rotation
			if (mean_acc[X_AXIS] > 0){
				left_motor_set_speed(speed);
				right_motor_set_speed(speed - speed_pid);
			}
			//counter clockwise rotation
			else {
				left_motor_set_speed(speed + speed_pid);
				right_motor_set_speed(speed);
			}
			//case where the robot is straight on the opposite direction
//			if (get_acceleration(Y_AXIS) < -2 && speed_pid == 0) {
//				i++;
//				//annulation of fluctuation when plane
//				if (i==60000 || j <= 60000){
//					left_motor_set_speed(-speed);
//					right_motor_set_speed(speed);
//					if (j == 60000){
//						j=0;
//					}
//				}
//			}
			if (mean_acc[Y_AXIS] < -error && speed_pid == 0) {
				speed_neg_slope = mean_acc[Y_AXIS] * KI_Y;
				left_motor_set_speed(speed + speed_neg_slope);
				right_motor_set_speed(speed);
			}
//			else{
//				i=0;
//			}
		}

		else if (get_free_path() == left) {
			while (obstacle_in_range(FRONTRIGHT45) || obstacle_in_range(FRONTRIGHT)) { // rotate to get goal distance
				left_motor_set_speed(-speed);
				right_motor_set_speed(speed);
			}
			pos_motor_left = left_motor_get_pos() + MOTOR_OBSTACLE;
			while (left_motor_get_pos() != pos_motor_left
					&& get_free_path() == straight) {
				left_motor_set_speed(speed);
				right_motor_set_speed(COEFF_ROT * speed);				//magic number
			}
		} else if (get_free_path() == right) { //free right
			while (obstacle_in_range(FRONTLEFT45) || obstacle_in_range(FRONTLEFT)) { // rotate to get goal distance
				left_motor_set_speed(speed);
				right_motor_set_speed(-speed);
			}
			pos_motor_right = right_motor_get_pos() + MOTOR_OBSTACLE;

			while (right_motor_get_pos() != pos_motor_right
					&& get_free_path() == 1) {
				left_motor_set_speed(COEFF_ROT * speed);			// magic number
				right_motor_set_speed(speed);
			}
		} else {
			left_motor_set_speed(0);
			right_motor_set_speed(0);
		}

		chThdSleepMilliseconds(10);
	}
}


static THD_WORKING_AREA(waMeanAccThd, 128);
static THD_FUNCTION(MeanAccThd, arg) {
	(void) arg;
	chRegSetThreadName(__FUNCTION__);

    messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");
    imu_msg_t imu_values;


    //table to store acceleration sample of X and Y
    float accel[AXIS][NB_SAMPLE];

	float sum_x_acc = 0;
	float sum_y_acc = 0;

	uint8_t sample = 0;

	//reset accel
    for (uint8_t i = 0; i < AXIS; i++)
    {
    	for (sample = 0; sample < NB_SAMPLE; sample++){
    		accel[i][sample] = 0;
    	}
    }

    sample = 0;

    while(1)
    {
    	messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));		//wait imu_values

        //subtract oldest acceleration values
        sum_x_acc = sum_x_acc - accel[X_AXIS][sample];
        sum_y_acc = sum_y_acc - accel[Y_AXIS][sample];

        //update acceleration values
        accel[X_AXIS][sample] = imu_values.acceleration[X_AXIS];
        accel[Y_AXIS][sample] = imu_values.acceleration[Y_AXIS];

        //add updated acceleration values
        sum_x_acc = sum_x_acc + accel[X_AXIS][sample];
        sum_y_acc = sum_y_acc + accel[Y_AXIS][sample];

        //calculate mean
        mean_acc[X_AXIS] = sum_x_acc/NB_SAMPLE;
        mean_acc[Y_AXIS] = sum_y_acc/NB_SAMPLE;

        sample++;

        if (sample == NB_SAMPLE){
        	sample = 0;
        }
    }
}


void move_start() {
	chThdCreateStatic(waMoveThd, sizeof(waMoveThd), NORMALPRIO,
			MoveThd, NULL);
	chThdCreateStatic(waMeanAccThd, sizeof(waMeanAccThd), NORMALPRIO,
			MeanAccThd, NULL);

}
