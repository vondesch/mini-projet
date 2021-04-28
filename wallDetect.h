#ifndef WALLDETECT_H
#define WALLDETECT_H

/*
 * sensor.h
 *
 *  Created on: Apr 15, 2021
 *      Author: raf-k
 */

#define FRONTLEFT 7				//proximity sensor front-left-5deg
#define FRONTLEFT45 6			//proximity sensor front-left-45deg
#define FRONTRIGHT 0			//proximity sensor front-left-5deg
#define FRONTRIGHT45 1			//proximity sensor front-left-45deg
#define RIGHT 2					//proximity sensor right
#define LEFT 5					//proximity sensor left

#define RANGE 110
#define MINDISTANCE 160
#define MINDISTANCE45 150

#define GOAL 80
#define KP 5
#define KI 5

enum {stuck, straight, left, right};

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

// uint8_t obstacle_detect(void);

uint8_t obstacle_in_range(uint8_t sensor);

float PI_correction(uint8_t sensor);

uint8_t free_path();

uint16_t speed_select();		//
												//debugging purpose only
//void print_distances(void);

void led_signal(void);


#endif /* WALLDETECT_H */



