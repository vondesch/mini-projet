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

#define INTENSITY 80
#define OFF 0
#define ON 80

// - IR0 (front-right) + IR4 (back-left)
// - IR1 (front-right-45deg) + IR5 (left)
// - IR2 (right) + IR6 (front-left-45deg)
// - IR3 (back-right) + IR7 (front-left)

uint8_t obstacle_in_range(uint8_t sensor) {
	if (get_prox(sensor) > RANGE) {
		return true;
	} else {
		return false;
	}

}

uint8_t free_path() {
	//free forward   smaller value means larger distance
	if (get_prox(FRONTRIGHT) < MINDISTANCE && get_prox(FRONTLEFT) < MINDISTANCE && get_prox(FRONTRIGHT45) < MINDISTANCE45 && get_prox(FRONTLEFT45) < MINDISTANCE45) {
		return straight;
	}	//free right
	else if (get_prox(FRONTRIGHT45) < MINDISTANCE
				&& get_prox(FRONTLEFT45) > MINDISTANCE) {
		return left;
	}
	//free left
	else if (get_prox(FRONTRIGHT45) > MINDISTANCE
			&& get_prox(FRONTLEFT45) < MINDISTANCE) {
		return right;
	}


	 else if (get_prox(FRONTRIGHT) > get_prox(FRONTLEFT)) {//obstacle closer to the right than to the left sensor
		return left;
	}

	else {
		return right;
	}
}

float PI_correction(uint8_t sensor) {
	//if (wall_detected()){
	// rotate();		// until tangential (distance 45 and 90deg)
	//PI keep 45 deg sensor measurements constant
	// while loop break if accelerometer facing highest slope or if other obstacle in front or if certain distance passed
	int error = 0;
	float correction = 1;

	static int sum_error = 0;

	error = get_prox(sensor) - GOAL;

	sum_error += error;

	correction = KP * error + KI * sum_error;
	return correction;
}


uint16_t speed_select() {
	uint8_t selector = get_selector();

	while (selector==10) {											//speed adjustment possible for 3 seconds add timer!!
		switch (selector) {
		case 0:
			set_led(LED8, OFF);
			set_led(LED1,ON);
			break;
		case 1:
			set_led(LED8, OFF);
			set_led(LED3, ON);
			break;
		case 2:
			set_led(LED8, OFF);
			set_led(LED5, ON);
			break;
		case 3:
			set_led(LED8, OFF);
			set_led(LED7, ON);
			break;
		default:
			break;
		}
		set_led(LED8, OFF);
	}

	switch (selector) {
	case 0:
		return 200;
	case 1:
		return 400;
	case 2:
		return 600;
	case 3:
	return 800;
	default: return 600;

}
}








uint8_t wall_left(void) {
	if (get_prox(FRONTLEFT) >= MINDISTANCE)
		return true;

	else if (get_prox(FRONTLEFT45) >= MINDISTANCE45)
		return true;

	else
		return false;
}

uint8_t wall_right(void) {
	if (get_prox(FRONTRIGHT) >= MINDISTANCE)
		return true;

	else if (get_prox(FRONTRIGHT45) >= MINDISTANCE45)
		return true;

	else
		return false;
}

uint8_t wall_detected(void) {
	if (wall_left() == true || wall_right() == true)
		return true;
	else
		return false;
}


/*uint8_t obstacle_detect(void) {
if (wall_detected()) {
	if (get_prox(FRONTLEFT) > get_prox(FRONTRIGHT)) {
		return left;
	} else if (get_prox(FRONTLEFT45) > get_prox(FRONTRIGHT45)) {
		return left;
	} else {
		return right;
	}
} else {
	return straight;
}
}*/

//void print_distances(void){
//	chprintf((BaseSequentialStream *)&SD3, "proximity left45=␣%d\n", get_prox(FRONTLEFT45));
//	chprintf((BaseSequentialStream *)&SD3, "proximity left=␣%d\n", get_prox(FRONTLEFT));
//	chprintf((BaseSequentialStream *)&SD3, "proximity right=␣%d\n", get_prox(FRONTRIGHT));
//	chprintf((BaseSequentialStream *)&SD3, "proximity right45=␣%d\n", get_prox(FRONTRIGHT45));
//}

void led_signal(void){
	toggle_rgb_led(LED2, BLUE_LED, INTENSITY);
	toggle_rgb_led(LED4, BLUE_LED, INTENSITY);
	toggle_rgb_led(LED6, BLUE_LED, INTENSITY); 		//toggles rgb leds
	toggle_rgb_led(LED8, BLUE_LED, INTENSITY);
}																					//add another LED function using green LEDs
