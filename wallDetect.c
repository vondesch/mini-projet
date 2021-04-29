/*
 * sensor.c
 *
 *  Created on: Apr 15, 2021
 *      Author: raf-k
 */
#include <main.h>
#include "ch.h"			// do ch.h/hal.h need to be included?
#include "hal.h"
#include "wallDetect.h"
#include <sensors/proximity.h>
#include <leds.h>
#include <selector.h>

//#define FRONTLEFT 7				//proximity sensor front-left-5deg
//#define FRONTLEFT45 6			//proximity sensor front-left-45deg
//#define FRONTRIGHT 0			//proximity sensor front-left-5deg
//#define FRONTRIGHT45 1			//proximity sensor front-left-45deg
//#define MINDISTANCE 3
//#define MINDISTANCE45 2.5

#define INTENSITY 80		// LED intensity when ON
#define OFF 0

// - IR0 (front-right) + IR4 (back-left)
// - IR1 (front-right-45deg) + IR5 (left)
// - IR2 (right) + IR6 (front-left-45deg)
// - IR3 (back-right) + IR7 (front-left)

static uint8_t freePath;
static uint16_t mindistance;
static uint16_t speed;

uint8_t obstacle_in_range(uint8_t sensor) {
	if (get_prox(sensor) > RANGE) {
		return true;
	} else {
		return false;
	}

}

static THD_WORKING_AREA(waFreePathThd, 128);
static THD_FUNCTION(FreePathThd, arg) {
	(void) arg;
	chRegSetThreadName(__FUNCTION__);

	while (1) {
		//free forward   smaller value means larger distance
		if (get_prox(FRONTRIGHT)
				< mindistance&& get_prox(FRONTLEFT) < mindistance
				&& get_prox(FRONTRIGHT45) < mindistance*CORR45
				&& get_prox(FRONTLEFT45) < mindistance*CORR45) {
			freePath = straight;
		}	//free right
		else if (get_prox(FRONTRIGHT45) < mindistance * CORR45
				&& get_prox(FRONTLEFT45) > mindistance * CORR45) {
			freePath = left;
		}
		//free left
		else if (get_prox(FRONTRIGHT45) > mindistance * CORR45
				&& get_prox(FRONTLEFT45) < mindistance * CORR45) {
			freePath = right;
		}

		else if (get_prox(FRONTRIGHT) > get_prox(FRONTLEFT)) {//obstacle closer to the right than to the left sensor
			freePath = left;
		}

		else {
			freePath = right;
		}
		chThdSleepMilliseconds(4);
	}

}

void free_path_start() {
	chThdCreateStatic(waFreePathThd, sizeof(waFreePathThd), NORMALPRIO,
			FreePathThd, NULL);

}

void set_mindistance(uint16_t distance) {
	mindistance = distance;
}
uint8_t get_free_path(void) {
	return freePath;
}

uint16_t get_speed(void) {
	return speed;
}

static THD_WORKING_AREA(waSpeedSelectThd, 128);
static THD_FUNCTION(SpeedSelectThd, arg) {
	(void) arg;
	chRegSetThreadName(__FUNCTION__);

	while (1) {
		switch (get_selector()) {
		case 0:
			set_led(LED8, OFF);					//set mode to seesaw
			set_led(LED1, INTENSITY);
			set_led(LED3, INTENSITY);
			set_led(LED5, INTENSITY);
			set_led(LED7, INTENSITY);
			set_mindistance(MINDISTANCESEESAW);
			speed = SPEED0;
			break;
		case 1:
			set_led(LED8, OFF);
			set_led(LED1, INTENSITY);			//set mode to game at speed0
			set_mindistance(MINDISTANCEGAME);
			speed = SPEED0;
			break;
		case 2:
			set_led(LED8, OFF);					//set mode to game at speed1
			set_led(LED3, INTENSITY);
			set_mindistance(MINDISTANCEGAME);
			speed = SPEED1;
			break;
		case 3:
			set_led(LED8, OFF);					//set mode to game at speed 2
			set_led(LED5, INTENSITY);
			set_mindistance(MINDISTANCEGAME);
			speed = SPEED2;
			break;
		case 4:
			set_led(LED8, OFF);					//set mode to game at speed 3
			set_led(LED7, INTENSITY);
			set_mindistance(MINDISTANCEGAME);
			speed = SPEED3;
			break;
		default:
			set_led(LED8, OFF);
			set_led(LED1, INTENSITY);
			set_mindistance(MINDISTANCEGAME);
			speed = 600;

		}
		chThdSleepMilliseconds(1000);

	}
}

void speed_select_start() {
	chThdCreateStatic(waSpeedSelectThd, sizeof(waSpeedSelectThd), NORMALPRIO,
			SpeedSelectThd, NULL);

}

void led_signal(void) {
	toggle_rgb_led(LED2, BLUE_LED, INTENSITY);
	toggle_rgb_led(LED4, BLUE_LED, INTENSITY);
	toggle_rgb_led(LED6, BLUE_LED, INTENSITY); 		//toggles rgb leds
	toggle_rgb_led(LED8, BLUE_LED, INTENSITY);
}

float PI_correction(uint8_t sensor) {
	//if (wall_detected()){
	// rotate();		// until tangential (distance 45 and 90deg)
	//PI keep 45 deg sensor measurements constant
	// while loop break if accelerometer facing highest slope or if other obstacle in front or if certain distance passed

	int GOAL = 80;
	int KP = 5;
	int KI = 5;
	int error = 0;
	float correction = 1;

	static int sum_error = 0;

	error = get_prox(sensor) - GOAL;

	sum_error += error;

	correction = KP * error + KI * sum_error;
	return correction;
}

//void print_distances(void){
//	chprintf((BaseSequentialStream *)&SD3, "proximity left45=␣%d\n", get_prox(FRONTLEFT45));
//	chprintf((BaseSequentialStream *)&SD3, "proximity left=␣%d\n", get_prox(FRONTLEFT));
//	chprintf((BaseSequentialStream *)&SD3, "proximity right=␣%d\n", get_prox(FRONTRIGHT));
//	chprintf((BaseSequentialStream *)&SD3, "proximity right45=␣%d\n", get_prox(FRONTRIGHT45));
//}
//add another LED function using green LEDs
