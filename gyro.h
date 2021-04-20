/*
 * gyro.h
 *
 *  Created on: 15 avr. 2021
 *      Author: Loic
 */

#ifndef GYRO_H_
#define GYRO_H_

#define STOP			0
#define GO				1
#define ROT_DROITE		2
#define ROT_GAUCHE		3

void get_position(void);

uint8_t detect_movement(void);

#endif /* GYRO_H_ */
