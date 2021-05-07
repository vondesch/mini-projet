/*
 * move.h
 *
 *  Created on: 4 mai 2021
 *      Author: Loic Von Deschwanden and Raphael Kohler
 */

#ifndef MOVE_H_
#define MOVE_H_

/**
 * @brief 	Starts the thread that manages the speed selector
 */
void speed_select_start(void);

/**
 * @brief	Starts the thread  that coordinates sthe movement
 */
void move_start(void);


#endif /* MOVE_H_ */
