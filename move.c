/*
 * gyro.c
 *
 *  Created on: 15 avr. 2021
 *      Author: Loic
 */

#include "ch.h"
#include "hal.h"
#include <main.h>
#include <sensors/imu.h>
#include <msgbus/messagebus.h>
#include <i2c_bus.h>
#include <motors.h>
#include "move.h"
#include "wallDetect.h"



