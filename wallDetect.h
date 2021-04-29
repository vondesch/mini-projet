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


uint8_t obstacle_in_range(uint8_t sensor);

float PI_correction(uint8_t sensor);

void free_path_start(void);

uint8_t get_free_path(void);



uint16_t speed_select(void);		//
												//debugging purpose only
//void print_distances(void);

void led_signal(void);


#endif /* WALLDETECT_H */



