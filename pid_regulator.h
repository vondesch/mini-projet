/*
 * pid_regulator.h
 *
 *  Created on: 1 mai 2021
 *      Author: Loic Von Deschwanden and Raphael Kohler
 */

#ifndef PID_REGULATOR_H_
#define PID_REGULATOR_H_

#define ERROR_THRESHOLD			1		//admissible error of the accelerometer

/**
 * @brief 	PID regulator that corrects the speed to prevent a halting movement
 */
int16_t pid_regulator(float error);



#endif /* PID_REGULATOR_H_ */
