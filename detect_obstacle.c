/*
 * detect_obstacle.c
 *
 *  Created on: Apr 15, 2021
 *      Author: Loic Von Deschwanden and Raphael Kohler
 */
#include <main.h>
#include "ch.h"
#include "hal.h"
#include <sensors/proximity.h>

#include "detect_obstacle.h"

#define MINDISTANCE 	500		// minimal distance to an obstacle situated in the direction of movement
#define COVERED 		800
static uint8_t freePath;

/**
 * @brief 	thread that checks for a direction where no obstacle is present
 * 			The updated information is stored in the variable freePath
 */
static THD_WORKING_AREA(waFreePathThd, 128);
static THD_FUNCTION(FreePathThd, arg) {
	(void) arg;
	chRegSetThreadName(__FUNCTION__);

	while (1) {

		if (get_prox(RIGHT) > COVERED && get_prox(LEFT) > COVERED) {
			freePath = stop;
		}

		//check if obstacle closer to the left than to the right sensor
		else if (get_prox(FRONTRIGHT45) < get_prox(FRONTLEFT45) &&
		get_prox(FRONTLEFT45) > MINDISTANCE) {
			freePath = right;
		} else if (get_prox(FRONTLEFT) > get_prox(FRONTRIGHT) &&
		get_prox(FRONTLEFT) > MINDISTANCE) {
			freePath = right;
		} else if (get_prox(LEFT) > MINDISTANCE) {
			freePath = right;
		}

		//check if obstacle closer to the right than to the left sensor
		else if (get_prox(FRONTRIGHT45) > MINDISTANCE
				&& get_prox(FRONTLEFT45) < get_prox(FRONTRIGHT45)) {
			freePath = left;
		} else if (get_prox(FRONTRIGHT) > get_prox(FRONTLEFT) &&
		get_prox(FRONTRIGHT) > MINDISTANCE) {
			freePath = left;
		} else if (get_prox(RIGHT) > MINDISTANCE) {
			freePath = left;
		}

		//no obstacle
		else {
			freePath = straight;
		}
		chThdSleepMilliseconds(4);
	}

}

void free_path_start() {
	chThdCreateStatic(waFreePathThd, sizeof(waFreePathThd), NORMALPRIO,
			FreePathThd, NULL);
}

uint8_t get_free_path(void) {
	return freePath;
}
