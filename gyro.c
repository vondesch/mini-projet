/*
 * gyro.c
 *
 *  Created on: 15 avr. 2021
 *      Author: Loic
 */

#include "ch.h"
#include "hal.h"
#include <gyro.h>
#include <main.h>
#include <sensors/imu.h>
#include <msgbus/messagebus.h>
#include <i2c_bus.h>
#include <motors.h>

#define vitesse 1200

void get_position(void) {

	//initialisation gyroscope
	imu_start();

	messagebus_t bus;
	MUTEX_DECL(bus_lock);
	CONDVAR_DECL(bus_condvar);

	messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");
	messagebus_init(&bus, &bus_lock, &bus_condvar);
	imu_msg_t imu_values;
	messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));

//	float *accel = imu_values->acceleration;

	uint8_t erreur = 1;
//	uint8_t mode = 0;
//	int a;
//	a = imu_values.acceleration[X_AXIS];

	if (erreur > imu_values.acceleration[X_AXIS]) {
		//rotation à droite
		left_motor_set_speed(vitesse);
		right_motor_set_speed(-vitesse);
	} else if (erreur < imu_values.acceleration[X_AXIS]) {
		//rotation à gauche
		left_motor_set_speed(-vitesse);
		right_motor_set_speed(vitesse);
	} else if (imu_values.acceleration[Y_AXIS] > erreur) {
		//avance
		left_motor_set_speed(vitesse);
		right_motor_set_speed(vitesse);
	} else {
		left_motor_set_speed(0);
		right_motor_set_speed(0);
	}

}

