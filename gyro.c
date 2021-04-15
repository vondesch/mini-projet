/*
 * gyro.c
 *
 *  Created on: 15 avr. 2021
 *      Author: Loic
 */

#include "ch.h"
#include "hal.h"
#include <gyro.h>
#include "imu.h"


void get_position(void)
{
	imu_start();

	float *accel = imu_values->acceleration;

}



