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

#include <main.h>
#include <motors.h>
#include <move.h>
#include <sensors/imu.h>
//#include <sensors/proximity.h>
#include <wallDetect.h>
#include <pid_regulator.h>
#include <leds.h>
#include <selector.h>


#define NB_SAMPLES_OFFSET 	200
#define MOTOR_OBSTACLE 		400
#define COEFF_ROT			0.8f
#define error  				1
#define NB_SAMPLE			5
#define AXIS				2		//axis X and Y
#define KI_Y				200

#define SPEED0 				200
#define SPEED1 				400
#define SPEED2 				500
#define SPEED3 				600
#define SPEED4 				800

#define INTENSITY 			100		// LED intensity when ON

#define MINDISTANCESEESAW 	150
#define MINDISTANCEGAME 	400

enum {off, on, toggle};

static float mean_acc[AXIS];
static uint16_t speed;


static THD_WORKING_AREA(waMoveThd, 128);
static THD_FUNCTION(MoveThd, arg) {
	(void) arg;
	chRegSetThreadName(__FUNCTION__);

//	uint16_t speed = 600;

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

			if (mean_acc[Y_AXIS] < -error) {
							speed_neg_slope = mean_acc[Y_AXIS] * KI_Y;
						}
			else speed_neg_slope = 0;

			if (mean_acc[X_AXIS] > 0){
				left_motor_set_speed(speed + speed_pid - speed_neg_slope);
				right_motor_set_speed(speed - speed_pid + speed_neg_slope);
			}
			//counter clockwise rotation
			else {
				left_motor_set_speed(speed + speed_pid + speed_neg_slope);
				right_motor_set_speed(speed - speed_pid - speed_neg_slope);
			}

		}

		else if (get_free_path() == left) {
			led_signal();
			while (obstacle_in_range(FRONTRIGHT45) || obstacle_in_range(FRONTRIGHT)) { // rotate to get goal distance
				left_motor_set_speed(-speed);
				right_motor_set_speed(speed);
			}
			pos_motor_left = left_motor_get_pos() + MOTOR_OBSTACLE;
			while (left_motor_get_pos() <= pos_motor_left
					&& get_free_path() == straight) {
				left_motor_set_speed(speed);
				right_motor_set_speed(COEFF_ROT * speed);				//magic number
			}
			led_signal();
		} else if (get_free_path() == right) { //free right
			led_signal();
			while (obstacle_in_range(FRONTLEFT45) || obstacle_in_range(FRONTLEFT)) { // rotate to get goal distance
				left_motor_set_speed(speed);
				right_motor_set_speed(-speed);
			}
			pos_motor_right = right_motor_get_pos() + MOTOR_OBSTACLE;

			while (right_motor_get_pos() <= pos_motor_right
					&& get_free_path() == straight) {
				left_motor_set_speed(COEFF_ROT * speed);			// magic number
				right_motor_set_speed(speed);
			}
			led_signal();
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
    static float accel[AXIS][NB_SAMPLE];

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

        if (sample >= NB_SAMPLE){
        	sample = 0;
        }
    }
}

static THD_WORKING_AREA(waSpeedSelectThd, 128);
static THD_FUNCTION(SpeedSelectThd, arg) {
	(void) arg;
	chRegSetThreadName(__FUNCTION__);

	while (1) {
		switch (get_selector()) {
		case 0:
			set_led(LED1, on);
			set_led(LED3, off);
			set_led(LED5, off);
			set_led(LED7, off);
			speed = SPEED0;
			break;
		case 1:
			set_led(LED1, off);			//set mode to game at speed0
			set_led(LED3, on);
			set_led(LED5, off);
			set_led(LED7, off);
			speed = SPEED1;
			break;
		case 2: 				//set mode to game at speed1
			set_led(LED1, on);
			set_led(LED3, on);
			set_led(LED5, on);
			set_led(LED7, on);
			speed = SPEED2;
			break;
		case 3:			//set mode to game at speed 2
			set_led(LED1, off);
			set_led(LED3, off);
			set_led(LED5, on);
			set_led(LED7, off);
			speed = SPEED3;
			break;
		case 4:			//set mode to game at speed 3
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
	chThdCreateStatic(waMoveThd, sizeof(waMoveThd), NORMALPRIO,
			MoveThd, NULL);
	chThdCreateStatic(waMeanAccThd, sizeof(waMeanAccThd), NORMALPRIO,
			MeanAccThd, NULL);

}


