/*
 * sensor.c
 *
 *  Created on: Apr 15, 2021
 *      Author: raf-k
 */
#include <main.h>
#include "ch.h"			// do ch.h/hal.h need to be included?
#include "hal.h"
#include "wallDetect.h"
#include <sensors/proximity.h>
#include <leds.h>

#define FRONTLEFT 7				//proximity sensor front-left-5deg
#define FRONTLEFT45 6			//proximity sensor front-left-45deg
#define FRONTRIGHT 0			//proximity sensor front-left-5deg
#define FRONTRIGHT45 1			//proximity sensor front-left-45deg
#define MINDISTANCE 3
#define MINDISTANCE45 3.5

#define INTENSITY 80
#define OFF 0

// - IR0 (front-right) + IR4 (back-left)
// - IR1 (front-right-45deg) + IR5 (left)
// - IR2 (right) + IR6 (front-left-45deg)
// - IR3 (back-right) + IR7 (front-left)

uint8_t wall_left(void){
	if (get_prox(FRONTLEFT)<=MINDISTANCE)
		return true;

	else if(get_prox(FRONTLEFT45)<=MINDISTANCE45)
		return true;

	else
		return false;
}

uint8_t wall_right(void){
	if(get_prox(FRONTRIGHT)<=MINDISTANCE)
		return true;

	else if( get_prox(FRONTRIGHT45)<=MINDISTANCE45)
		return true;

	else
		return false;
	}

uint8_t wall_detected(void){
	if (wall_left()==true || wall_right()==true)
		return true;
	else
		return false;
}

void print_distances(void){
	chprintf((BaseSequentialStream *)&SD3, "proximity left45=␣%d\n", get_prox(FRONTLEFT45));
	chprintf((BaseSequentialStream *)&SD3, "proximity left=␣%d\n", get_prox(FRONTLEFT));
	chprintf((BaseSequentialStream *)&SD3, "proximity right=␣%d\n", get_prox(FRONTRIGHT));
	chprintf((BaseSequentialStream *)&SD3, "proximity right45=␣%d\n", get_prox(FRONTRIGHT45));

}

void led_signal(void){
	set_led(LED8, INTENSITY ); //led8 corresponds to setting all LEDS at the same time
	delay(SystemCoreClock/16);
	set_led(LED8, OFF);

	set_led(LED1, INTENSITY); 		//setting every other LED to high
	delay(SystemCoreClock/16);
	set_led(LED1, OFF);

	set_led(LED3, INTENSITY );
	delay(SystemCoreClock/16);
	set_led(LED3, OFF);

	set_led(LED5, INTENSITY);
	delay(SystemCoreClock/16);
	set_led(LED5, OFF);

	set_led(LED7, INTENSITY);
	delay(SystemCoreClock/16);
	set_led(LED7, OFF);

	set_led(LED8, INTENSITY ); //setting all LEDS at the same time
	delay(SystemCoreClock/16);
	set_led(LED8, OFF);

	set_led(LED8, INTENSITY ); //setting all LEDS at the same time
	delay(SystemCoreClock/16);
	set_led(LED8, OFF);

}																					//add another LED function using green LEDs
