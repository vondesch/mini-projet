#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"

#include <main.h>
#include <motors.h>
#include <sensors/imu.h>
#include <sensors/proximity.h>
#include <msgbus/messagebus.h>
#include <selector.h>

#include "tools.h"
#include "move.h"
#include "detect_obstacle.h"

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);


int main(void) {
	//system initalisation
	halInit();
	chSysInit();
	mpu_init();

	//initializes the motors the accelerometer and proximity sensor
	motors_init();
	imu_start();
	proximity_start();

	//messagebus_t bus;
	messagebus_init(&bus, &bus_lock, &bus_condvar);

	//wait to be stable
	chThdSleepMilliseconds(SLEEP);

	//calibration of the sensors
	calibrate_acc();
	calibrate_ir();

	//initialization of threads
	free_path_start();
	speed_select_start();
	move_start();

	while (1) {
		chThdSleepMilliseconds(SLEEP);
	}
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void) {
	chSysHalt("Stack smashing detected");
}
