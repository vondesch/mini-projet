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
uint8_t wall_left(void);

/**
* @brief   Checks if an obstacle is located somewhere to the right ahead of the robot
*/
uint8_t wall_right(void);

 /**
 * @brief   Checks if an obstacle is located somewhere in front of the robot
 */
uint8_t wall_detected(void);

												//debugging purpose only
void print_distances(void);

void led_signal(void);


#endif /* WALLDETECT_H */



