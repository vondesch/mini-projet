#ifndef WALLDETECT_H
#define WALLDETECT_H

/*
 * sensor.h
 *
 *  Created on: Apr 15, 2021
 *      Author: raf-k
 */

 /**
 * @brief   Checks if an obstacle is located somewhere in to the left ahead of the robot
 */
uint8_t wallLeft(void);

/**
* @brief   Checks if an obstacle is located somewhere to the right ahead of the robot
*/
uint8_t wallRight(void);

 /**
 * @brief   Checks if an obstacle is located somewhere in front of the robot
 */
uint8_t wallDetected(void);

												//debugging purpose only
void printDistances(void);


#endif /* WALLDETECT_H */



