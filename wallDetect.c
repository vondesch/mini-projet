/*
 * obstacle_detect.c
 *
 *  Created on: Apr 15, 2021
 *      Author: Loic Von Deschwanden and Raphael Kohler
 */
#include <main.h>
#include "ch.h"
#include "hal.h"
#include "wallDetect.h"
#include <sensors/proximity.h>

#define RANGE 			110
#define CORR45 			0.8
#define MINDISTANCE 	150

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
		if (get_prox(FRONTRIGHT45) < get_prox(FRONTLEFT45) &&
				get_prox(FRONTLEFT45) >= MINDISTANCE * CORR45) {
			freePath = right;
		}
		//free left
		else if (get_prox(FRONTRIGHT45) > MINDISTANCE * CORR45 &&
				get_prox(FRONTLEFT45) < get_prox(FRONTRIGHT45)) {
			freePath = left;
		}

		else if (get_prox(FRONTRIGHT) > get_prox(FRONTLEFT) &&
				get_prox(FRONTRIGHT) > MINDISTANCE) { //obstacle closer to the right than to the left sensor
			freePath = left;
		}

		else if (get_prox(FRONTLEFT) >= get_prox(FRONTRIGHT) &&
				get_prox(FRONTLEFT) > MINDISTANCE) {
			freePath = right;
		} else {
			freePath = straight;
		}
		chThdSleepMilliseconds(4);
	}

}

void free_path_start() {
	chThdCreateStatic(waFreePathThd, sizeof(waFreePathThd), NORMALPRIO, FreePathThd, NULL);
}

uint8_t get_free_path(void) {
	return freePath;
}
