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
int wallLeft();

/**
* @brief   Checks if an obstacle is located somewhere to the right ahead of the robot
*/
int wallRight();

 /**
 * @brief   Checks if an obstacle is located somewhere in front of the robot
 */
int wallDetected();


#endif /* WALLDETECT_H */



