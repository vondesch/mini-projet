#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <chprintf.h>
#include <motors.h>
#include <audio/microphone.h>
#include <math.h>

#include <audio_processing.h>
#include <fft.h>
#include <communications.h>
#include <arm_math.h>
#include <gyro.h>
#include <sensors/imu.h>
#include <sensors/proximity.h>
#include <msgbus/messagebus.h>
#include <i2c_bus.h>
#include <selector.h>

#include <wallDetect.h>

#define NB_SAMPLES_OFFSET 200
#define MOTOR_OBSTACLE 400

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

void object_PI(uint16_t speed) {
	uint16_t pos_motor_right;
	uint16_t pos_motor_left;

	uint8_t error = 1;					// create define

	if (free_path() == straight) { //no obstacle in front of robot
		if (get_acceleration(Y_AXIS) < -error) {
			if (get_acceleration(X_AXIS) < -error) {
				//rotation à droite
				left_motor_set_speed(-speed);
				right_motor_set_speed(+speed);
			} else if (get_acceleration(X_AXIS) > error) {
				//rotation à gauche
				left_motor_set_speed(speed);
				right_motor_set_speed(-speed);
			} else {
				//avance
				left_motor_set_speed(speed);
				right_motor_set_speed(speed);
			}
		} else if (get_acceleration(X_AXIS) > error) {
			//cw rotation
			left_motor_set_speed(speed);
			right_motor_set_speed(-speed);
		} else if (get_acceleration(X_AXIS) < -error) {
			//ccw rotation
			left_motor_set_speed(-speed);
			right_motor_set_speed(speed);
		} else if (get_acceleration(Y_AXIS) > error) {
			//move forward
			left_motor_set_speed(speed);
			right_motor_set_speed(speed);
		} else {
			left_motor_set_speed(0);
			right_motor_set_speed(0);
		}
	}

	else if (free_path() == left) {
		led_signal(); 				//free left
		while (obstacle_in_range(FRONTRIGHT45)) { // rotate to get goal distance
			left_motor_set_speed(-speed);
			right_motor_set_speed(speed);
		}
		pos_motor_left = left_motor_get_pos() + MOTOR_OBSTACLE;
		while (left_motor_get_pos() != pos_motor_left && free_path() == straight) {
			left_motor_set_speed(speed);
			right_motor_set_speed(0.8 * speed);					//magic number
		}
		led_signal();
	} else if (free_path() == right) { //free right
		led_signal();
		while (obstacle_in_range(FRONTLEFT45)) { // rotate to get goal distance
			left_motor_set_speed(speed);
			right_motor_set_speed(-speed);
		}
		pos_motor_right = right_motor_get_pos() + MOTOR_OBSTACLE;

		while (right_motor_get_pos() != pos_motor_right && free_path() == 1) {
			left_motor_set_speed(0.8 * speed);			// magic number
			right_motor_set_speed(speed);
		}
		led_signal();

	} else {
		left_motor_set_speed(0);
		right_motor_set_speed(0);
	}
}

/*void motor_gyro(void) {

 uint16_t pos_motor_right;
 uint16_t pos_motor_left;

 uint8_t error = 1;

 //	if (get_acceleration(X_AXIS) < -error) {
 //		//rotation à droite
 //		left_motor_set_speed(speed);
 //		right_motor_set_speed(-speed);
 //	} else if (get_acceleration(X_AXIS) > error) {
 //		//rotation à gauche
 //		left_motor_set_speed(-speed);
 //		right_motor_set_speed(speed);
 //	} else if(error + 0.2  > abs(get_acceleration(Y_AXIS)) || abs(get_acceleration(Y_AXIS)) > 0){
 //		chThdSleepMilliseconds(200);

 if (obstacle_detect() == right) {
 if (!obstacle_in_range()) {			// rotate to get goal distance
 left_motor_set_speed(-speed);
 right_motor_set_speed(speed);
 }
 while (!free_path() || get_acceleration(X_AXIS) > error) {
 left_motor_set_speed(PI_correction(FRONTRIGHT45) * speed);// modify speed of left wheel
 right_motor_set_speed(-speed);
 }
 } else if (obstacle_detect() == left) {
 if (!obstacle_in_range()) {
 left_motor_set_speed(speed);
 right_motor_set_speed(-speed);
 }
 while (!free_path() || get_acceleration(X_AXIS) > error) {
 left_motor_set_speed(speed);
 right_motor_set_speed(PI_correction(FRONTRIGHT45) * speed);	// modify speed of right wheel
 }
 } else if (obstacle_in_range()) {
 //		right_motor_get_pos
 left_motor_set_speed(speed);
 right_motor_set_speed(speed);
 } else {
 if (get_acceleration(Y_AXIS) < -error) {
 if (get_acceleration(X_AXIS) < -error) {
 //rotation à droite
 left_motor_set_speed(speed);
 right_motor_set_speed(-speed);
 } else if (get_acceleration(X_AXIS) > error) {
 //rotation à gauche
 left_motor_set_speed(-speed);
 right_motor_set_speed(speed);
 } else {
 //recule
 left_motor_set_speed(-speed);
 right_motor_set_speed(-speed);
 }
 } else if (get_acceleration(X_AXIS) > error) {
 //rotation à droite
 left_motor_set_speed(speed);
 right_motor_set_speed(-speed);
 } else if (get_acceleration(X_AXIS) < -error) {
 //rotation à gauche
 left_motor_set_speed(-speed);
 right_motor_set_speed(speed);
 } else if (get_acceleration(Y_AXIS) > error) {
 //avance
 left_motor_set_speed(speed);
 right_motor_set_speed(speed);
 } else {
 left_motor_set_speed(0);
 right_motor_set_speed(0);
 }
 }
 }*/

//uncomment to use double buffering to send the FFT to the computer
//#define DOUBLE_BUFFERING
static void serial_start(void) {
	static SerialConfig ser_cfg = { 115200, 0, 0, 0, };

	sdStart(&SD3, &ser_cfg); // UART3.
}

static void timer12_start(void) {
//General Purpose Timer configuration
//timer 12 is a 16 bit timer so we can measure time
//to about 65ms with a 1Mhz counter
	static const GPTConfig gpt12cfg = { 1000000, /* 1MHz timer clock in order to measure uS.*/
	NULL, /* Timer callback.*/
	0, 0 };

	gptStart(&GPTD12, &gpt12cfg);
//let the timer count to max value
	gptStartContinuous(&GPTD12, 0xFFFF);
}

int main(void) {
	halInit();
	chSysInit();
	mpu_init();

//starts the serial communication
	serial_start();
//starts timer 12
	timer12_start();
//inits the motors
	motors_init();

	i2c_start();

//initialisation gyroscope
	imu_start();
	proximity_start();

//init_selector();
	uint16_t speed = 600; //speed_select();//define variable speed using the selector

//messagebus_t bus;
	messagebus_init(&bus, &bus_lock, &bus_condvar);

	messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus,
			"/imu");
	imu_msg_t imu_values;

	calibrate_acc();
	calibrate_ir();
//	imu_compute_offset(imu_topic, NB_SAMPLES_OFFSET);
	int16_t val_acc[2];
	while (1) {
		messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));
		val_acc[0] = get_acceleration(X_AXIS);
		val_acc[1] = get_acceleration(Y_AXIS);
		//motor_gyro();
//		chprintf((BaseSequentialStream *) &SD3, "%Ax=%.2f Ay=%.2f (%x)\r\n\n", val_acc[0], val_acc[1]);
		chprintf((BaseSequentialStream *) &SD3,
				"%proximity_left45=%d proximity_left=%d proximity_right=%d proximity_right45=%d (%x)\r\n\n",
				get_prox(FRONTLEFT45), get_prox(FRONTLEFT),
				get_prox(FRONTRIGHT), get_prox(FRONTRIGHT45));
//		chprintf((BaseSequentialStream *)&SD3, "proximity left=%d\n", get_prox(FRONTLEFT));
//		chprintf((BaseSequentialStream *)&SD3, "proximity right=%d\n", get_prox(FRONTRIGHT));
//		chprintf((BaseSequentialStream *)&SD3, "proximity right45=%d\n", get_prox(FRONTRIGHT45));

		object_PI(speed);

	}

}

//int main(void)
//{
//
//    halInit();
//    chSysInit();
//    mpu_init();
//
//    //starts the serial communication
//    serial_start();
//    //starts the USB communication
//    usb_start();
//    //starts timer 12
//    timer12_start();
//    //inits the motors
//    motors_init();
//
//    //send_tab is used to save the state of the buffer to send (double buffering)
//    //to avoid modifications of the buffer while sending it
//    static float send_tab[FFT_SIZE];
//
//#ifdef SEND_FROM_MIC
//    //starts the microphones processing thread.
//    //it calls the callback given in parameter when samples are ready
//    mic_start(&processAudioData);
//#endif  /* SEND_FROM_MIC */
//
//    /* Infinite loop. */
//    while (1) {
//#ifdef SEND_FROM_MIC
//        //waits until a result must be sent to the computer
//        wait_send_to_computer();
//#ifdef DOUBLE_BUFFERING
//        //we copy the buffer to avoid conflicts
//        arm_copy_f32(get_audio_buffer_ptr(LEFT_OUTPUT), send_tab, FFT_SIZE);
//        SendFloatToComputer((BaseSequentialStream *) &SD3, send_tab, FFT_SIZE);
//#else
//        SendFloatToComputer((BaseSequentialStream *) &SD3, get_audio_buffer_ptr(LEFT_OUTPUT), FFT_SIZE);
//#endif  /* DOUBLE_BUFFERING */
//#else
//
//        float* bufferCmplxInput = get_audio_buffer_ptr(LEFT_CMPLX_INPUT);
//        float* bufferOutput = get_audio_buffer_ptr(LEFT_OUTPUT);
//
//        //temp tab used to store values in complex_float format
//        //needed bx doFFT_c
//
//
//
//        uint16_t size = ReceiveInt16FromComputer((BaseSequentialStream *) &SD3, bufferCmplxInput, FFT_SIZE);
//
//        if(size == FFT_SIZE){
//
////        	volatile uint16_t time_1 = 0;
////			chSysLock();
////			//reset the timer counter
////			GPTD12.tim->CNT = 0;
//////
////        	doFFT_optimized(FFT_SIZE, bufferCmplxInput);
//////
////        	time_1 = GPTD12.tim->CNT;
////			chSysUnlock();
////			chprintf((BaseSequentialStream *)&SDU1, "time_1=%dus\n", time_1);
//
//        	static complex_float temp_tab[FFT_SIZE];
//
//			//need to convert the float buffer into complex_float struct array
//			for(uint16_t i = 0 ; i < (2*FFT_SIZE) ; i+=2){
//				temp_tab[i/2].real = bufferCmplxInput[i];
//				temp_tab[i/2].imag = bufferCmplxInput[i+1];
//			}
//			//do a non optimized FFT
//
//			volatile uint16_t time_2 = 0;
//			chSysLock();
//			//reset the timer counter
//			GPTD12.tim->CNT = 0;
//
//			doFFT_c(FFT_SIZE, temp_tab);
//
//			time_2 = GPTD12.tim->CNT;
//			chSysUnlock();
//			chprintf((BaseSequentialStream *)&SDU1, "time_2=%dus\n", time_2);
//
//			//reconverts the result into a float buffer
//			for(uint16_t i = 0 ; i < (2*FFT_SIZE) ; i+=2){
//				bufferCmplxInput[i] = temp_tab[i/2].real;
//				bufferCmplxInput[i+1] = temp_tab[i/2].imag;
//			}
//
////			volatile uint16_t time = 0;
////			chSysLock();
////			//reset the timer counter
////			GPTD12.tim->CNT = 0;
//
//            arm_cmplx_mag_f32(bufferCmplxInput, bufferOutput, FFT_SIZE);
//
////            time = GPTD12.tim->CNT;
////			chSysUnlock();
////			chprintf((BaseSequentialStream *)&SDU1, "time=%dus\n", time);
//
//            SendFloatToComputer((BaseSequentialStream *) &SD3, bufferOutput, FFT_SIZE);
//
//        }
//#endif  /* SEND_FROM_MIC */
//    }
//}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void) {
	chSysHalt("Stack smashing detected");
}
