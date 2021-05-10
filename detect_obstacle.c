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

#define MINDISTANCE 	150		// minimal distance to an obstacle situated in the direction of movement
#define CORR45 			0.7		// correction factor to keep the minimal distance to objects located at 45deg to the direction of movement
#define RANGE 			110		// IR sensor value that corresponds to the minimally accepted distance to an obstacle while rotating

static uint8_t freePath;

uint8_t obstacle_in_range(uint8_t sensor) {
	if (get_prox(sensor) > RANGE) {
		return true;
	} else {
		return false;
	}

}

/**
 * @brief 	thread that checks for a direction where no obstacle is present
 * 			The updated information is stored in the variable freePath
 */
static THD_WORKING_AREA(waFreePathThd, 128);
static THD_FUNCTION(FreePathThd, arg) {
	(void) arg;
	chRegSetThreadName(__FUNCTION__);

	while (1) {

		//check if obstacle closer to the left than to the right sensor
		if (get_prox(FRONTRIGHT45) < get_prox(FRONTLEFT45) &&
		get_prox(FRONTLEFT45) >= MINDISTANCE * CORR45) {
			freePath = right;
		} else if (get_prox(FRONTLEFT) >= get_prox(FRONTRIGHT) &&
		get_prox(FRONTLEFT) > MINDISTANCE) {
			freePath = right;
		}

		//check if obstacle closer to the right than to the left sensor
		else if (get_prox(FRONTRIGHT45) > MINDISTANCE * CORR45
				&& get_prox(FRONTLEFT45) < get_prox(FRONTRIGHT45)) {
			freePath = left;
		} else if (get_prox(FRONTRIGHT) > get_prox(FRONTLEFT) &&
		get_prox(FRONTRIGHT) > MINDISTANCE) {
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
