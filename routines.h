/*
 * routines.h
 *
 * ECE4191: Team E25
 *
 * this file contains all subroutines and functions used by the robot for actuation and movement
 *
 *  Created on: Sep 19, 2024
 *      Author: Anthony Verbni, Will Middlewick
 */

#ifndef ROUTINES_H_
#define ROUTINES_H_

#include "main.h"
#include <math.h>

// define functions and external variables
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim16;
extern int start_time;
extern int balls_collected;
extern int global_timeout_mins;
void zero_all_positions();
void motors_disable();
void acknowledge_ball();
int follow_ball();
int follow_box();
void raise_scooper_arms();
void lower_scooper_arms();
void open_ball_trapper();
void close_ball_trapper();
void open_release_gate();
void close_release_gate();
void move_away_from_line(int motor_dir);
void rotate_180_deg();
int uart_delay(int time, int enable_interruption, int motor_dir);
int uart_delay_box(int time, int enable_interruption, int motor_dir);
void scan_for_balls(int num_turn_ticks, int scanning_mode, int ball_scan);

// UART from main.c
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern uint8_t rx_buff[10];
float norm;

//I2C
extern I2C_HandleTypeDef hi2c3;
void write8 (uint8_t reg, uint32_t value);
uint8_t read8(uint8_t reg);
uint16_t read16(uint8_t reg);
void enable(void);
void disable(void);
void setIntegrationTime(uint8_t it);
void setGain(uint8_t gain);
void tcs3272_init( void );
void getRawData (uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c);
#define TCS34725_ADDRESS          (0x29 << 1) /* I2C address */
/* Datasheet is at https://cdn-shop.adafruit.com/datasheets/TCS34725.pdf */
#define TCS34725_COMMAND_BIT      (0x80)      /* Command bit */
#define TCS34725_ENABLE           (0x00)      /* Enable register */
#define TCS34725_ENABLE_AEN       (0x02)      /* RGBC Enable */
#define TCS34725_ENABLE_PON       (0x01)      /* Power on */
#define TCS34725_ATIME            (0x01)      /* Integration time */
#define TCS34725_CONTROL          (0x0F)      /* Set the gain level */
#define TCS34725_ID               (0x12)
/* 0x44 = TCS34721/TCS34725, 0x4D = TCS34723/TCS34727 */
#define TCS34725_CDATAL           (0x14)      /* Clear channel data */
#define TCS34725_CDATAH           (0x15)
#define TCS34725_RDATAL           (0x16)      /* Red channel data */
#define TCS34725_RDATAH           (0x17)
#define TCS34725_GDATAL           (0x18)      /* Green channel data */
#define TCS34725_GDATAH           (0x19)
#define TCS34725_BDATAL           (0x1A)      /* Blue channel data */
#define TCS34725_BDATAH           (0x1B)
/* set the correct delay time in void getRawData() for TCS34725_INTEGRATIONTIME */
#define TCS34725_INTEGRATIONTIME_50MS   0xEB  /* 50ms  - 20 cycles */
#define TCS34725_GAIN_4X                0x01  /* 4x gain  */
uint8_t _tcs34725Initialised = 0;


typedef struct {
    float r;
    float g;
    float b;
} RGB;

typedef struct {
    float h;
    float s;
    float v;
} HSV;


RGB normalize_rgb(int r, int g, int b);
RGB getRGB();
HSV rgb_to_hsv(float r, float g, float b);


/* VARIABLES */
uint16_t i;

//////////////////////////////////////////////// SERVO CONTROL ////////////////////////////////////////////////

/* RESET ALL SERVOS TO THEIR ZERO POSITIONS */
void zero_all_positions() {
	lower_scooper_arms();
	close_release_gate();
	open_ball_trapper();
}

/* RAISE SCOOPER ARMS */
void raise_scooper_arms() {
	htim2.Instance->CCR2 = 35;  // scoop the ball
	htim2.Instance->CCR4 = 90;
}

/* LOWER SCOOPER ARMS */
void lower_scooper_arms() {
	// zero scooper
	htim2.Instance->CCR2 = 98;
	htim2.Instance->CCR4 = 27;
}

/* CLOSE BALL TRAPPER SERVO */
void close_ball_trapper() {
	HAL_GPIO_WritePin(SERVO_CTRL_GPIO_Port, SERVO_CTRL_Pin, 0);
	htim16.Instance->CCR1 = 125;  // trap the ball
}

/* OPEN BALL TRAPPER SERVO */
void open_ball_trapper() {
	HAL_GPIO_WritePin(SERVO_CTRL_GPIO_Port, SERVO_CTRL_Pin, 0);
	htim16.Instance->CCR1 = 25;
}

/* CLOSE RELEASE GATE SERVO */
void close_release_gate() {
	HAL_GPIO_WritePin(SERVO_CTRL_GPIO_Port, SERVO_CTRL_Pin, 1);
	htim16.Instance->CCR1 = 75;
}

/* OPEN RELEASE GATE SERVO */
void open_release_gate() {
	HAL_GPIO_WritePin(SERVO_CTRL_GPIO_Port, SERVO_CTRL_Pin, 1);
	htim16.Instance->CCR1 = 25;
}

//////////////////////////////////////////////// MOTORS ////////////////////////////////////////////////

/* MOTORS PARAMETERS */
uint16_t ramp_rate = 5;
uint16_t ramp_delay = 10;
uint16_t motor_speed_off = 150;
float motor_l_correction_factor = 1;
float motor_r_correction_factor = 1.2;

// NOTE: timeout parameter 'runtime' in milliseconds


/* MOTORS OFF */
void motors_disable() {
	TIM1->CCR1 = 0;
	TIM1->CCR4 = 0;
}

/* MOTORS FWD */
void motors_forward(uint16_t motor_speed) {

	HAL_GPIO_WritePin(MOTOR_L_FWD_GPIO_Port, MOTOR_L_FWD_Pin, 1);
	HAL_GPIO_WritePin(MOTOR_R_FWD_GPIO_Port, MOTOR_R_FWD_Pin, 1);

	for(i=motor_speed_off; i<motor_speed+1; i+=ramp_rate){
		TIM1->CCR1 = i;
		TIM1->CCR4 = i;
		HAL_Delay(ramp_delay);
	}
}

/* MOTORS REV */
void motors_reverse(uint16_t motor_speed) {

	HAL_GPIO_WritePin(MOTOR_L_FWD_GPIO_Port, MOTOR_L_FWD_Pin, 0);
	HAL_GPIO_WritePin(MOTOR_R_FWD_GPIO_Port, MOTOR_R_FWD_Pin, 0);


	for(i=motor_speed_off; i<motor_speed+1; i+=ramp_rate){
		TIM1->CCR1 = i*motor_l_correction_factor;
		TIM1->CCR4 = i*motor_r_correction_factor;
		HAL_Delay(ramp_delay);
	}
}

/* MOTORS LEFT */
void motors_left(uint16_t motor_speed) {

	HAL_GPIO_WritePin(MOTOR_L_FWD_GPIO_Port, MOTOR_L_FWD_Pin, 1);
	HAL_GPIO_WritePin(MOTOR_R_FWD_GPIO_Port, MOTOR_R_FWD_Pin, 0);

	for(i=motor_speed_off; i<motor_speed+1; i+=ramp_rate){
		TIM1->CCR1 = i;
		TIM1->CCR4 = i;
		HAL_Delay(ramp_delay);
	}
}

/* MOTORS RIGHT */
void motors_right(uint16_t motor_speed) {

	HAL_GPIO_WritePin(MOTOR_L_FWD_GPIO_Port, MOTOR_L_FWD_Pin, 0);
	HAL_GPIO_WritePin(MOTOR_R_FWD_GPIO_Port, MOTOR_R_FWD_Pin, 1);

	for(i=motor_speed_off; i<motor_speed+1; i+=ramp_rate){
		TIM1->CCR1 = i;
		TIM1->CCR4 = i;
		HAL_Delay(ramp_delay);
	}
}

/* MOTORS RIGHT SLOW */
void motors_right_slow(uint16_t motor_speed) {

	HAL_GPIO_WritePin(MOTOR_R_FWD_GPIO_Port, MOTOR_R_FWD_Pin, 1);
	TIM1->CCR1 = 0;

	for(i=motor_speed_off; i<motor_speed+1; i+=ramp_rate) {
		TIM1->CCR4 = i;
		HAL_Delay(ramp_delay);
	}
}

/* MOTORS LEFT SLOW */
void motors_left_slow(uint16_t motor_speed) {

	HAL_GPIO_WritePin(MOTOR_L_FWD_GPIO_Port, MOTOR_L_FWD_Pin, 1);
	TIM1->CCR4 = 0;

	for(i=motor_speed_off; i<motor_speed+1; i+=ramp_rate) {
		TIM1->CCR1 = i;
		HAL_Delay(ramp_delay);
	}
}

//////////////////////////////////////////////// SUBROUTINES AND HELPER FUNCTIONS ////////////////////////////////////////////////

void pick_up_ball() {
/*
 * Subroutine to pick up a ball using the scooper
 */
	motors_forward(250); //move forward to get closer to the ball
	uart_delay(500, 0, 1);
	close_ball_trapper(); //trap the ball
	uart_delay(1000, 0, 1);
	motors_disable();
	raise_scooper_arms(); //scoop the ball
	HAL_Delay(100);
	open_ball_trapper(); //stop the ball from gaining speed
	HAL_Delay(1000);
	close_ball_trapper(); //release the ball
	HAL_Delay(1000);
	zero_all_positions(); //zero the servos
	HAL_Delay(200);
	motors_reverse(250);
	uart_delay(500, 0, 0);
	motors_disable();
}

void deposit_balls() {
/*
 * Sub routine to turn around and deposit the balls
 */
	while (1) {

		//rotate 180deg
		rotate_180_deg();

		//drive back until it hits the limit switch
		motors_reverse(200);
		int limit_switches_triggered = 1;
		int failed = 0;
		int time_init = HAL_GetTick();
		while (limit_switches_triggered) {

			if (HAL_GetTick() - time_init > 3000) {
				failed = 1;
				break;
			}

			limit_switches_triggered = HAL_GPIO_ReadPin(LS_GPIO_Port, LS_Pin);
			HAL_Delay(5);

		}

		if (failed == 0) {
			break; //success
		} else {
			follow_box();
		}
	}

	//release the balls
	motors_disable();
	open_release_gate();
	HAL_Delay(3000);

	// move the robot away from the box to prepare for getting more balls
	motors_forward(250);
	HAL_Delay(1000);
	motors_disable();
	HAL_Delay(500);

	close_release_gate();
}

int follow_ball() {
/*
 * Subroutine to approach a ball based on commands from the RPI.
 *
 *
 * Output:
 *   1 = ball has been approached and is ready to collect
 *   0 = ball was lost and not able to be recovered
 *
 */
	int turning_time_default = 15;
	int forward_time_ms = 40;   // default speed for forward movement when close to ball
	int turning_time_ms = 20;   // default adjustment speed for left/right turning
	int adjusted_time = 10;     // adjusted left/right speed for when ball is close
	int forward_ticks = 0;      // counter for number of times robot moved toward ball when close
	int max_forward_ticks = 6;  // max number of times robot should move closer to a ball before stopping

	int consecutive_ticks = 0;
	char prev_move = 'L';

	int scan_speed = 250;

	while (1) {

		if(HAL_UART_Receive(&huart1, rx_buff, 10, 1000)==HAL_OK) {

			// ball is in front, move forward
			if (rx_buff[0] == 'F') {
				motors_forward(250);

				// if the ball is close to the robot, only move forward in steps (slowly)
				if (turning_time_ms == adjusted_time) {
					uart_delay(forward_time_ms, 0, 1);
					motors_disable();
					forward_ticks += 1; // increment a counter
					if (forward_ticks == max_forward_ticks) { turning_time_ms = turning_time_default; };
				} else {
					uart_delay(60, 0, 1);
				}
			}

			// move backward
			if (rx_buff[0] == 'B') {
				motors_reverse(250);

				// moving backward means the robot is trying to recover a lost ball, reset any closeness indicators
				turning_time_ms = 50;
				forward_ticks = 0;
			}

			// ball is to the left, turn left
			if (rx_buff[0] == 'L') {
				motors_left_slow(scan_speed);
				uart_delay(turning_time_ms, 0, 1);
				motors_disable();

				if (prev_move == 'R') {
					consecutive_ticks = 0;
				} else {
					consecutive_ticks++;
				}

				if (consecutive_ticks >= 10) {
					consecutive_ticks = 0;
					motors_left(350);
					uart_delay(20, 0, 1);
					motors_disable();
				}

				prev_move = 'L';

			}

			// ball is to the right, turn right
			if (rx_buff[0] == 'R') {
				motors_right_slow(scan_speed);
				uart_delay(turning_time_ms, 0, 1);
				motors_disable();

				if (prev_move == 'L') {
					consecutive_ticks = 0;
				} else {
					consecutive_ticks++;
				}

				if (consecutive_ticks >= 10) {
					consecutive_ticks = 0;
					motors_right(350);
					uart_delay(20, 0, 1);
					motors_disable();
				}

				prev_move = 'R';
			}

			// Ball is close to the robot
			if (rx_buff[0] == 'C') {
				turning_time_ms = adjusted_time; // reduce the turning time to make finer adjustments
				motors_disable();
			}

			// if robot is within scooping distance, exit the function
			if (forward_ticks == max_forward_ticks) {
				motors_disable();
				zero_all_positions();
				HAL_Delay(1000);
				return 1; // ball is ready to scoop up
			}

			// ball was not found in the camera, try to recover its position
			if (rx_buff[0] == 'S') {
				// give the RPI some time to try recover first
				motors_disable();
				if (uart_delay(1000, 1, 1) >= 1) {
					continue; // something was found, continue navigation
				}

				// robot was still not able to recover
				// reverse the robot in case it had driven past the ball
				motors_reverse(250);
				turning_time_ms = turning_time_default;
				forward_ticks = 0;
				if (uart_delay(1000, 1, 0) >= 1) {
					continue; // something was found, continue navigation
				}

				// ball was still not found, wait a little longer for a camera recovery
				motors_disable();
				if (uart_delay(1000, 1, 1) >= 1) {
					continue; // something was found, continue navigation
				}

				return 0; // nothing was found at all. exit the function
			}
		}
	}
}

int follow_box() {
/*
 * Subroutine to approach a box based on commands from the RPI.
 *
 *
 * Output:
 *   1 = box has been approached and is ready to deposit
 *   0 = box was lost and not able to be recovered
 *
 */

	int turning_time_ms = 50;   // default adjustment speed for left/right turning
	int consecutive_ticks = 0;
	char prev_move = 'F';

	int scan_speed = 250;

	while (1) {

		if(HAL_UART_Receive(&huart1, rx_buff, 10, 1000)==HAL_OK) {

			// ball is in front, move forward
			if (rx_buff[0] == 'W') {
				motors_forward(250);
				uart_delay_box(80, 0, 1);
				consecutive_ticks = 0;
			}

			// ball is to the left, turn left
			if (rx_buff[0] == 'A') {
				motors_left_slow(scan_speed);
				uart_delay_box(turning_time_ms, 0, 1);
				motors_disable();

				if (prev_move == 'R') {
					consecutive_ticks = 0;
				} else {
					consecutive_ticks++;
				}

				if (consecutive_ticks >= 15) {
//					consecutive_ticks = 0;
//					motors_left(350);
//					uart_delay_box(20, 0, 1);
					motors_disable();
					motors_reverse(250);
					HAL_Delay(500);
					motors_disable();
					return 1;
				}

				prev_move = 'L';
			}

			// ball is to the right, turn right
			if (rx_buff[0] == 'D') {
				motors_right_slow(scan_speed);
				uart_delay_box(turning_time_ms, 0, 1);
				motors_disable();

				if (prev_move == 'L') {
					consecutive_ticks = 0;
				} else {
					consecutive_ticks++;
				}

				if (consecutive_ticks >= 15) {
//					consecutive_ticks = 0;
//					motors_right(350);
//					uart_delay_box(20, 0, 1);
					motors_disable();
					motors_reverse(250);
					HAL_Delay(500);
					motors_disable();
					return 1;
				}

				prev_move = 'R';
			}

			// ball was not found in the camera, try to recover its position
			if (rx_buff[0] == 'M') {
				// give the RPI some time to try recover first
				motors_disable();
				if (uart_delay_box(1500, 1, 1) >= 1) {
					continue; // something was found, continue navigation
				}

				// scan for box again
				motors_right(350);
				HAL_Delay(600);
				motors_disable();

				// give the RPI some time to try recover first
				if (uart_delay_box(1500, 1, 1) >= 1) {
					continue; // something was found, continue navigation
				}

				// scan for box again
				motors_right(350);
				HAL_Delay(200);
				motors_disable();

				return 0; // nothing was found at all. exit the function
			}

			if (rx_buff[0] == 'Y') {
				return 1; // exit when close enough to the box
			}
		}
	}
}

void scan_for_balls(int num_turn_ticks, int scanning_mode, int ball_scan) {
/*
 * Subroutine to generically scan for balls on the court
 *
 * Function exits once a ball was spotted in the camera
 *
 * Inputs:
 *   num_turn_ticks = number of ticks to scan left/right
 *   scanning_mode = scanning mode (1 = radial pattern, 2 = 360deg pattern)
 *
 */
	int time_limit_fwd = 3000;
	int time_limit_turning = 50;
	int wait_time = 700;

	int scan_speed = 350; // speed the robot turns to scan - must be from 250-500

	// repetitively scan in a radar fashion
	while (1) {

		if (scanning_mode == 1) {
			// -- drive forward -- //
			motors_forward(220);
			if (ball_scan) {
				if(uart_delay(time_limit_fwd, 1, 1) == 1) {
					goto end; //break if ball found
				}
			} else {
				if(uart_delay_box(time_limit_fwd, 1, 1) == 1) {
					goto end; //break if ball found
				}
			}
		}

		// -- scan right -- //
		for (int i = 0; i < num_turn_ticks; i++) {
			motors_right(scan_speed);
			if (ball_scan) {
				if(uart_delay(time_limit_turning, 1, 1) == 1) {
						goto end; //break if ball found
					}
			} else {
					if(uart_delay_box(time_limit_turning, 1, 1) == 1) {
						goto end; //break if ball found
					}
			}
			motors_disable();
			if (ball_scan) {
				if(uart_delay(wait_time, 1, 1) == 1) {
						goto end; //break if ball found
				}
			} else {
				if(uart_delay_box(wait_time, 1, 1) == 1) {
					goto end; //break if ball found
				}
			}
		}

		if (scanning_mode == 1) {
			// -- scan left -- //
			for (int i = 0; i < num_turn_ticks*2; i++) {
				motors_left(scan_speed);
				if (ball_scan) {
					if(uart_delay(time_limit_turning, 1, 1) == 1) {
							goto end; //break if ball found
						}
				} else {
						if(uart_delay_box(time_limit_turning, 1, 1) == 1) {
							goto end; //break if ball found
						}
				}
				motors_disable();
				if (ball_scan) {
					if(uart_delay(wait_time, 1, 1) == 1) {
							goto end; //break if ball found
					}
				} else {
					if(uart_delay_box(wait_time, 1, 1) == 1) {
						goto end; //break if ball found
					}
				}
			}

			// -- scan 90 degrees right -- //
			for (int i = 0; i < num_turn_ticks; i++) {
				motors_right(scan_speed);
				if (ball_scan) {
					if(uart_delay(time_limit_turning, 1, 1) == 1) {
							goto end; //break if ball found
						}
				} else {
						if(uart_delay_box(time_limit_turning, 1, 1) == 1) {
							goto end; //break if ball found
						}
				}
				motors_disable();
				if (ball_scan) {
					if(uart_delay(wait_time, 1, 1) == 1) {
							goto end; //break if ball found
					}
				} else {
					if(uart_delay_box(wait_time, 1, 1) == 1) {
						goto end; //break if ball found
					}
				}
			}
		}

		if (scanning_mode == 2) {
			// -- drive forward -- //
			motors_forward(200);
			if (ball_scan) {
				if(uart_delay(time_limit_fwd, 1, 1) == 1) {
					goto end; //break if ball found
				}
			} else {
				if(uart_delay_box(time_limit_fwd, 1, 1) == 1) {
					goto end; //break if ball found
				}
			}
		}
	}

	// ball has been found.
	end:
	return;
}

int uart_delay(int time, int enable_interruption, int motor_dir) {
	/*
	 * Function to wait X milliseconds whilst monitoring UART
	 *
	 * Inputs:
	 *   - time: time to wait (in milliseconds)
	 *   - enable_interruption: allow the waiting to be interrupt early based on received commands (1 = enabled)
	 *
	 * Output:
	 *   2 = robot drove over a white line during wait time (out of bounds)
	 *   1 = ball was found in camera during wait time
	 *   0 = wait time completed fully
	 */

	// test global timeout for the milestone
	int time_elapsed_since_start = (HAL_GetTick() - start_time) / 60000;
	if (time_elapsed_since_start > global_timeout_mins && balls_collected > 0) {
		motors_disable();
		HAL_Delay(1000);
		if (follow_box()) {
		  // Turn 180deg and deposit the balls in the box
		  deposit_balls();
		  NVIC_SystemReset(); // RESET THE STM!!!!
		}
	}

	int time_init = HAL_GetTick();
	while(HAL_GetTick() - time_init < time) {

		// monitor color sensor data here too. stop the ball from going over the white line
		RGB color_sensor_reading = getRGB();
		if (color_sensor_reading.r == 0 || color_sensor_reading.g == 0 || color_sensor_reading.b == 0) {
			continue;
		}
		HSV hsv_color = rgb_to_hsv(color_sensor_reading.r, color_sensor_reading.g, color_sensor_reading.b);
		norm = sqrt((hsv_color.h*hsv_color.h) + (hsv_color.s*hsv_color.s) + (hsv_color.v*hsv_color.v));
		if (norm < 230 && norm > 200) {
			move_away_from_line(motor_dir);

			// exit the function
			return 2;
		}

		// monitor UART while waiting
		if (enable_interruption) {
			if(HAL_UART_Receive(&huart1, rx_buff, 10, 1000)==HAL_OK) {
				// allow the delay to be interrupted early in case of a ball being found
				if (rx_buff[0] == 'F' || rx_buff[0] == 'L' || rx_buff[0] == 'R') {
					return 1;
				}
			}
		}
	}

	return 0;
}

int uart_delay_box(int time, int enable_interruption, int motor_dir) {
	/*
	 * Function to wait X milliseconds whilst monitoring UART
	 *
	 * Inputs:
	 *   - time: time to wait (in milliseconds)
	 *   - enable_interruption: allow the waiting to be interrupt early based on received commands (1 = enabled)
	 *
	 * Output:
	 *   2 = robot drove over a white line during wait time (out of bounds)
	 *   1 = ball was found in camera during wait time
	 *   0 = wait time completed fully
	 */

	int time_init = HAL_GetTick();
	while(HAL_GetTick() - time_init < time) {

		// monitor color sensor data here too. stop the ball from going over the white line
		RGB color_sensor_reading = getRGB();
		if (color_sensor_reading.r == 0 || color_sensor_reading.g == 0 || color_sensor_reading.b == 0) {
			continue;
		}
		HSV hsv_color = rgb_to_hsv(color_sensor_reading.r, color_sensor_reading.g, color_sensor_reading.b);
		norm = sqrt((hsv_color.h*hsv_color.h) + (hsv_color.s*hsv_color.s) + (hsv_color.v*hsv_color.v));
		if (norm < 230 && norm > 200) {
			move_away_from_line(motor_dir);

			// exit the function
			return 2;
		}

		// monitor UART while waiting
		if (enable_interruption) {
			if(HAL_UART_Receive(&huart1, rx_buff, 10, 1000)==HAL_OK) {
				// allow the delay to be interrupted early in case of a ball being found
				if (rx_buff[0] == 'W' || rx_buff[0] == 'A' || rx_buff[0] == 'D') {
					return 1;
				}
			}
		}
	}

	return 0;
}

void rotate_180_deg() {
	int rotate_time = 2200 * 1.05;
	motors_disable();
	HAL_Delay(500);
	motors_right(220);
	HAL_Delay(rotate_time);
	motors_disable();
	HAL_Delay(500);
}

void move_away_from_line(int motor_dir) {

	motors_disable();
	HAL_Delay(500);

	// move away from the line depending on the robot's direction
	if (motor_dir == 1) {
		// motor is going forward, reverse
		motors_reverse(250);
		HAL_Delay(2500);
		motors_disable();

		//rotate 180deg
		rotate_180_deg();
	} else {
		// motor is going backward, go forward
		motors_forward(250);
		HAL_Delay(2500);
		motors_disable();
	}
}

//////////////////////////////////////////////// I2C AND COLOUR SENSOR FUNCTIONS ////////////////////////////////////////////////

/* Writes a register and an 8 bit value over I2C */
void write8 (uint8_t reg, uint32_t value)
{
    uint8_t txBuffer[2];
    txBuffer[0] = (TCS34725_COMMAND_BIT | reg);
    txBuffer[1] = (value & 0xFF);
    HAL_I2C_Master_Transmit(&hi2c3, TCS34725_ADDRESS, txBuffer, 2, 100);
}

/* Reads an 8 bit value over I2C */
uint8_t read8(uint8_t reg)
{
    uint8_t buffer[1];
    buffer[0] = (TCS34725_COMMAND_BIT | reg);
    HAL_I2C_Master_Transmit(&hi2c3, TCS34725_ADDRESS, buffer, 1, 100);
    HAL_I2C_Master_Receive(&hi2c3, TCS34725_ADDRESS, buffer, 1, 100);
    return buffer[0];
}

/* Reads a 16 bit values over I2C */
uint16_t read16(uint8_t reg)
{
  uint16_t ret;
    uint8_t txBuffer[1],rxBuffer[2];
    txBuffer[0] = (TCS34725_COMMAND_BIT | reg);
    HAL_I2C_Master_Transmit(&hi2c3, TCS34725_ADDRESS, txBuffer, 1, 100);
    HAL_I2C_Master_Receive(&hi2c3, TCS34725_ADDRESS, rxBuffer, 2, 100);
    ret = rxBuffer[1];
    ret <<= 8;
    ret |= rxBuffer[0] & 0xFF;
  return ret;
}

void enable(void)
{
  write8(TCS34725_ENABLE, TCS34725_ENABLE_PON);
  HAL_Delay(3);
  write8(TCS34725_ENABLE, TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN);
  HAL_Delay(50);
}

void disable(void)
{
  /* Turn the device off to save power */
  uint8_t reg = 0;
  reg = read8(TCS34725_ENABLE);
  write8(TCS34725_ENABLE, reg & ~(TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN));
}

void setIntegrationTime(uint8_t itime)
{
  if (_tcs34725Initialised == 0) tcs3272_init();
  write8(TCS34725_ATIME, itime);
}

void setGain(uint8_t gain)
{
  if (_tcs34725Initialised == 0) tcs3272_init();
  write8(TCS34725_CONTROL, gain);
}

void tcs3272_init(void)
{
  /* Make sure we're actually connected */
  uint8_t readValue = read8(TCS34725_ID);
  if ((readValue != 0x44) && (readValue != 0x10) && (readValue != 0x4d))
  {
    return;
  }
  _tcs34725Initialised = 1;
  /* Set default integration time and gain */
  setIntegrationTime(TCS34725_INTEGRATIONTIME_50MS);
  setGain(TCS34725_GAIN_4X);
  /* Note: by default, the device is in power down mode on bootup */
  enable();
}

// COLOR SENSOR FUNCTIONS
/* Get raw data */
void getRawData (uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c)
{
  if (_tcs34725Initialised == 0) tcs3272_init();

  *c = read16(TCS34725_CDATAL);
  *r = read16(TCS34725_RDATAL);
  *g = read16(TCS34725_GDATAL);
  *b = read16(TCS34725_BDATAL);
  /* Delay time is from page no 16/26 from the datasheet  (256 − ATIME)* 2.4ms */
  HAL_Delay(50); // Set delay for (256 − 0xEB)* 2.4ms = 50ms
}

/* Get Red, Green and Blue color from Raw Data */
RGB getRGB()
{
	RGB output;
    uint16_t rawRed, rawGreen, rawBlue, rawClear;
    getRawData(&rawRed, &rawGreen, &rawBlue, &rawClear);
    if(rawClear == 0)
    {
      output.r = 0;
      output.g = 0;
      output.b = 0;
    }
    else
    {
      output.r = (int)rawRed * 255 / rawClear;
      output.g = (int)rawGreen * 255 / rawClear;
      output.b = (int)rawBlue * 255 / rawClear;
    }

    return output;
}

/* Normalize RGB Values to generalize to different lighting conditions */
RGB normalize_rgb(int r, int g, int b) {
    RGB normalized;
    int total = r + g + b;
    if (total == 0) {
        normalized.r = 0;
        normalized.g = 0;
        normalized.b = 0;
    } else {
        normalized.r = (float)r / total;
        normalized.g = (float)g / total;
        normalized.b = (float)b / total;
    }
    return normalized;
}

/* Function to convert between RGB and HSV color space */
HSV rgb_to_hsv(float r, float g, float b) {
    HSV hsv;
    float min_val = fminf(fminf(r, g), b);
    float max_val = fmaxf(fmaxf(r, g), b);
    float delta = max_val - min_val;

    // Calculate value (V)
    hsv.v = max_val;

    // Calculate saturation (S)
    if (max_val != 0)
        hsv.s = (delta / max_val) * 255;
    else {
        hsv.s = 0;
        hsv.h = -1; // Undefined hue
        return hsv;
    }

    // Calculate hue (H)
    if (r == max_val)
        hsv.h = (g - b) / delta; // Between yellow and magenta
    else if (g == max_val)
        hsv.h = 2 + (b - r) / delta; // Between cyan and yellow
    else
        hsv.h = 4 + (r - g) / delta; // Between magenta and cyan

    hsv.h *= 60; // Convert to degrees
    if (hsv.h < 0)
        hsv.h += 360;

    return hsv;
}

#endif /* ROUTINES_H_ */
