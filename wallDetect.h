#ifndef WALLDETECT_H
#define WALLDETECT_H

/*
 * obstacle_detect.h
 *
 *  Created on: Apr 15, 2021
 *      Author: Loic Von Deschwanden and Raphael Kohler
 */

#define FRONTRIGHT 		0			//proximity sensor front-left-5deg
#define FRONTRIGHT45 	1			//proximity sensor front-left-45deg
#define RIGHT 			2			//proximity sensor right-90deg
#define LEFT 			5			//proximity sensor left-90deg
#define FRONTLEFT45 	6			//proximity sensor front-left-45deg
#define FRONTLEFT 		7			//proximity sensor front-left-5deg


enum {straight, left, right};

/**
 * @brief 	Determines if an obstacle is close to a specific IR-sensor
 * 			Returns true if obstacle is closer than RANGE
 */
uint8_t obstacle_in_range(uint8_t sensor);

/**
 * @brief 	Starts the thread that checks in what direction no obstacle is present
 */
void free_path_start(void);

/**
 * @brief 	Returns in what direction no obstacle is present
 */
uint8_t get_free_path(void);


#endif /* WALLDETECT_H */



