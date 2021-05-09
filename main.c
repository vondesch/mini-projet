#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
//#include <chprintf.h>
#include <motors.h>
#include <math.h>

//#include <arm_math.h>
#include <sensors/imu.h>
#include <sensors/proximity.h>
#include <msgbus/messagebus.h>
#include <i2c_bus.h>
#include <selector.h>

#include <pid_regulator.h>
#include <wallDetect.h>
#include <move.h>

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

static void serial_start(void) {
	static SerialConfig ser_cfg = { 115200, 0, 0, 0, };

	sdStart(&SD3, &ser_cfg); // UART3.
}

int main(void) {
	//system initalisation
	halInit();
	chSysInit();
	mpu_init();

	//starts the serial communication
	serial_start();

	//initializes the motors
	motors_init();

	i2c_start();

	//initializes the gyroscope and proximity sensor
	imu_start();
	proximity_start();

	//messagebus_t bus;
	messagebus_init(&bus, &bus_lock, &bus_condvar);

	//wait to be stable
	chThdSleepMilliseconds(1000);

	//calibration of the sensors
	calibrate_acc();
	calibrate_ir();

	//execution of threads
	move_start();
	free_path_start();
	speed_select_start();

//	imu_compute_offset(imu_topic, NB_SAMPLES_OFFSET);
	while (1) {
		//wait 1s
		chThdSleepMilliseconds(1000);
//		messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));
//		chprintf((BaseSequentialStream *) &SD3, "%Ax=%.2f Ay=%.2f (%x)\r\n\n",
//				get_acc_filtered(X_AXIS, filter_size), get_acc_filtered(Y_AXIS, filter_size));
//		chprintf((BaseSequentialStream *) &SD3,
//				"%proximity_left45=%d proximity_left=%d proximity_right=%d proximity_right45=%d (%x)\r\n\n",
//				get_prox(FRONTLEFT45), get_prox(FRONTLEFT),
//				get_prox(FRONTRIGHT), get_prox(FRONTRIGHT45));
	}
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void) {
	chSysHalt("Stack smashing detected");
}
