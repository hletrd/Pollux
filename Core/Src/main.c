/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


#define firmware_version "0.2"
#define hardware_version "2.0"
#define firmware_date __DATE__
#define firmware_time __TIME__
#define product_name "Carina EQdriver"

#define epsilon 1e-7

#define RA 0
#define DEC 1

#define CPU_CLOCK 168000000

#define tim_ra_prescaler 168
#define tim_ra_counter 10
#define tim_dec_prescaler 168
#define tim_dec_counter 10

#define tim_guide_prescaler 16800
#define tim_guide_counter 100
#define tim_acc_prescaler 16800
#define tim_acc_counter 50
#define tim_goto_prescaler 16800
#define tim_goto_counter 50

#define DIRSET_RA 1
#define DIRSET_DEC 1

#define ra_fast_crit_current 50
#define ra_fast_crit_ustep 700
#define dec_fast_crit_current 50
#define dec_fast_crit_ustep 700
#define MODE_FAST 0
#define MODE_SLOW 1

#define STATE_STOP 0
#define STATE_TRACK 1

#define STATE_SLEW_WAIT 0
#define STATE_SLEW_HC 1
#define STATE_SLEW_GOTO 2
#define STATE_SLEW_HC_STOPPING 3

#define STATE_DYN_STOP 0
#define STATE_DYN_ACC 1
#define STATE_DYN_UNIFRM 2
#define STATE_DYN_DACC 3



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;
TIM_HandleTypeDef htim12;
TIM_HandleTypeDef htim13;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

//**(Virtually constants)
float ra_motor_reducer = 13.125; //1000x
float ra_final_reducer = 144.0; //1000x

float dec_motor_reducer = 13.125; //1000x
float dec_final_reducer = 144.0; //1000x

int32_t ra_motor_step = 200;
int32_t dec_motor_step = 200;

int32_t ra_motor_ustep_max = 32;
int32_t dec_motor_ustep_max = 32;
//********************

//Acc, dacc rate
float ra_acc = 700;
float ra_dacc = 700;

float dec_acc = 700;
float dec_dacc = 700;
//

//ustep table
int32_t ra_ustep_slow = 8;
int32_t dec_ustep_slow = 8;

int32_t ra_ustep_fast = 8;
int32_t dec_ustep_fast = 8;
//

int32_t tim_counter_ra_max = 1000000000;
int32_t tim_counter_dec_max = 1000000000;

int32_t ra_ustep_now, dec_ustep_now;

float fast_spd_max = 1440;

float gotospeed_table[4];

float guide_speed = 0.5;
float guide_ra_now = 0, guide_dec_now = 0;

float gotospeed_hc_ra;
float gotospeed_hc_dec;

//uint32_t time_now = 1606382480;
//2020-11-18 22:00:00
uint32_t time_offset_local = 32400;

int64_t ra_pos_now, dec_pos_now; //RA, DEC pos of moving mount.
int64_t ra_pos_target, dec_pos_target;
int64_t ra_pos_offset, dec_pos_offset;

//absolute RA of target object (tracking compensation)
double ra_pos_diff;

int64_t ra_ustep_per_rev, dec_ustep_per_rev;

float sdrl_day = 86164.0905, solar_day = 86400.0;
float ra_spd_ratio_sdrl = 1, dec_spd_ratio_sdrl = 1;
float ra_spd_ratio_solar = 1;

float ra_tick_per_step;
float dec_tick_per_step;

int32_t ra_timer_remainder; //to track more accurately
int32_t tim_counter_ra_remainder = 1e9;

float tim_ra_freq;
float tim_dec_freq;

float tim_guide_freq;
float tim_acc_freq;
float tim_goto_freq;

int dir_ra_target = 1, dir_dec_target = 1;
int dir_ra_now = 1, dir_dec_now = 1;

float ra_current_fast = 0.4, ra_current_slow = 0.03;
float dec_current_fast = 0.4, dec_current_slow = 0.03;

int state_ra_currentmode = MODE_SLOW;
int state_dec_currentmode = MODE_SLOW;
int state_ra_ustepmode = MODE_SLOW;
int state_dec_ustepmode = MODE_SLOW;

float ra_spd_now = 0;
float dec_spd_now = 0;
float ra_spd_target = 0;
float dec_spd_target = 0;

int state_ra = STATE_STOP;
int state_dec = STATE_STOP;

int state_ra_slew = STATE_SLEW_WAIT;
int state_dec_slew = STATE_SLEW_WAIT;

int state_dyn_ra = STATE_STOP;
int state_dyn_dec = STATE_STOP;

float ra_drivecurrent_now;
float dec_drivecurrent_now;

//up, down, left, right
int HC_STATE[4] = {0, 0, 0, 0};
int GUIDE_STATE[4] = {0, 0, 0, 0};

int debugmode = 1;

float vin, iin;

float site_lon, site_lat;

uint8_t ser_buf[ser_bufsize];
int ser_pos;
int32_t ser_last;

int adc_mode;
int dbg_counter;

float melody_queue[melody_buf_len];
int melody_len_queue[melody_buf_len];
int melody_play_cnt;
int melody_play_pos;
int melody_counter;

#define C_Hz 261.63
#define D_Hz 293.67
#define E_Hz 329.63
#define F_Hz 349.23
#define G_Hz 392.00
#define A_Hz 440.00
#define B_Hz 493.88

#define d_Hz 277.18
#define e_Hz 311.13
#define g_Hz 369.99
#define a_Hz 415.30
#define b_Hz 466.16

#define melody_base_counter 10000

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM12_Init(void);
static void MX_TIM13_Init(void);
static void MX_TIM14_Init(void);
/* USER CODE BEGIN PFP */

void led_set(int lednum, int ledstate);
void hc_slew();
void tim_guide_callback();
int32_t get_ra_second();
int32_t get_ra_second_target();
int32_t convert_ra_to_second(float ra_abs);
int64_t convert_ra_to_ustep(float ra_second);
int32_t get_dec_second();
int32_t convert_dec_to_second(float dec_abs);
int64_t convert_dec_to_ustep(float dec_second);
void set_ra_second(int32_t ra_second);
void set_ra_second_target(int32_t ra_second);
void set_dec_second(int32_t dec_second);
void set_dec_second_target(int32_t dec_second);
void sync();
int atoi_2(uint8_t* input) ;
void goto_slew();
void serial_decode();
void uart_print(uint8_t* output);
void uart_printi(int output);
void uart_printf(float output);
void uart_printlli(int64_t output);
RTC_TimeTypeDef get_time_now();
RTC_DateTypeDef get_date_now();
void set_time_now(RTC_TimeTypeDef time_now);
void set_date_now(RTC_DateTypeDef date_now);
void serial_process();
void goto_target();
void set_current(int axis, float current);
void set_ustep(int axis, int step);
void stop_motor(int axis);
void start_motor(int axis);
void reset_motor(int axis);
void tim_ra_callback();
void tim_dec_callback();
void applydir();
void input_sen();
void tim_acc_callback();
void set_ra_now(int64_t radelta);
void set_dec_now(int64_t decdelta);
void debug();
void melody_add(int note, int octave, int length);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void led_set(int lednum, int ledstate) {
	int pin = 0;
	switch (lednum) {
		case 0:
			pin = LED_Y_Pin;
			break;
		case 1:
			pin = LED_B_Pin;
			break;
		case 2:
			pin = LED_W_Pin;
			break;
		case 3:
			pin = LED_G_Pin;
			break;
	}
	if (ledstate) {
		HAL_GPIO_WritePin(GPIOB, pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(GPIOB, pin, GPIO_PIN_RESET);
	}
}

void hc_slew() {
	if (HC_STATE[0] == 1 && HC_STATE[1] == 0) { //Up
		dec_spd_target = gotospeed_hc_dec;
		state_dec_slew = STATE_SLEW_HC;
	} else if (HC_STATE[0] == 0 && HC_STATE[1] == 1) { //Down
		dec_spd_target = -gotospeed_hc_dec;
		state_dec_slew = STATE_SLEW_HC;
	} else if (HC_STATE[0] == 0 && HC_STATE[1] == 0) { //dec release
		dec_spd_target = 0;
		state_dec_slew = STATE_SLEW_HC_STOPPING;
	} else {
		dec_spd_target = 0;
		state_dec_slew = STATE_SLEW_HC_STOPPING;
	}
	if (HC_STATE[2] == 1 && HC_STATE[3] == 0) { //Left
		ra_spd_target = -gotospeed_hc_ra;
		state_ra_slew = STATE_SLEW_HC;
	} else if (HC_STATE[2] == 0 && HC_STATE[3] == 1) { //Right
		ra_spd_target = gotospeed_hc_ra;
		state_ra_slew = STATE_SLEW_HC;
	} else if (HC_STATE[2] == 0 && HC_STATE[3] == 0) { //ra release
		if (state_ra == STATE_TRACK) {
			ra_spd_target = 1;
		} else {
			ra_spd_target = 0;
		}
		state_ra_slew = STATE_SLEW_HC_STOPPING;
	} else {
		if (state_ra == STATE_TRACK) {
			ra_spd_target = 1;
		} else {
			ra_spd_target = 0;
		}
		state_ra_slew = STATE_SLEW_HC_STOPPING;
	}
}

void guide_callback() {
	/*GUIDE_STATE[0] = HAL_GPIO_ReadPin(GUIDE2_GPIO_Port, GUIDE2_Pin) == GPIO_PIN_RESET ? 1 : 0;
	GUIDE_STATE[1] = HAL_GPIO_ReadPin(GUIDE3_GPIO_Port, GUIDE3_Pin) == GPIO_PIN_RESET ? 1 : 0;
	GUIDE_STATE[2] = HAL_GPIO_ReadPin(GUIDE1_GPIO_Port, GUIDE1_Pin) == GPIO_PIN_RESET ? 1 : 0;
	GUIDE_STATE[3] = HAL_GPIO_ReadPin(GUIDE4_GPIO_Port, GUIDE4_Pin) == GPIO_PIN_RESET ? 1 : 0;*/

	if (GUIDE_STATE[0] == 1 && GUIDE_STATE[1] == 0) { //Up
		guide_dec_now = guide_speed;
	} else if (GUIDE_STATE[0] == 0 && GUIDE_STATE[1] == 1) { //Down
		guide_dec_now = -guide_speed;
	} else if (GUIDE_STATE[0] == 0 && GUIDE_STATE[1] == 0) { //dec release
		guide_dec_now = 0;
	} else {
		guide_dec_now = 0;
	}
	if (GUIDE_STATE[2] == 1 && GUIDE_STATE[3] == 0) { //Left
		guide_ra_now = guide_speed;
	} else if (GUIDE_STATE[2] == 0 && GUIDE_STATE[3] == 1) { //Right
		guide_ra_now = -guide_speed;
	} else if (GUIDE_STATE[2] == 0 && GUIDE_STATE[3] == 0) { //ra release
		guide_ra_now = 0;
	} else {
		guide_ra_now = 0;
	}
}

int32_t get_ra_second() {
	int64_t ra_pos_tmp = ra_pos_now + ra_pos_diff;
	if (ra_pos_tmp < 0) ra_pos_tmp += ra_ustep_per_rev;
	ra_pos_tmp %= ra_ustep_per_rev;
	float tmp = ((float)ra_pos_tmp) / ra_ustep_per_rev;
	return convert_ra_to_second(tmp);
}

int32_t get_ra_second_target() {
	int64_t ra_pos_tmp = ra_pos_target;
	if (ra_pos_tmp < 0) ra_pos_tmp += ra_ustep_per_rev;
	ra_pos_tmp %= ra_ustep_per_rev;
	float tmp = ra_pos_target / (float)ra_ustep_per_rev;
	return convert_ra_to_second(tmp);
}

int32_t convert_ra_to_second(float ra_abs) {
	int32_t ra_second_ratio = 24*60*60;
	return ra_second_ratio * ra_abs;
}

int64_t convert_ra_to_ustep(float ra_second) {
	double ra_second_ratio = 24*60*60;
	double tmp = ra_second / ra_second_ratio;
	return (int64_t)(tmp * ra_ustep_per_rev);
}

int32_t get_dec_second() {
	int64_t dec_pos_tmp = dec_pos_now + dec_pos_offset;
	if (dec_pos_tmp < 0) dec_pos_tmp += dec_ustep_per_rev;
	float tmp = ((float)dec_pos_tmp) / dec_ustep_per_rev;
	return convert_dec_to_second(tmp);
}

int32_t convert_dec_to_second(float dec_abs) {
	int32_t dec_second_ratio = 90*60*60;
	int32_t result;
	if (dec_abs < 0.25) {
		result = (0.25-dec_abs)*4*dec_second_ratio;
	} else if (dec_abs < 0.5) {
		result = (0.25-dec_abs)*4*dec_second_ratio;
	} else if (dec_abs < 0.75) {
		result = (dec_abs-0.75)*4*dec_second_ratio;
	} else {
		result = (dec_abs-0.75)*4*dec_second_ratio;
	}
	return result;
}

int64_t convert_dec_to_ustep(float dec_second) {
	double dec_second_ratio = 90*60*60;
	double tmp = dec_second / dec_second_ratio;
	return (int64_t)(tmp * dec_ustep_per_rev);
}

void set_ra_second(int32_t ra_second) { //overwrite current setttings (SYNC)
	//float ra_second_ratio = 24*60*60;
	ra_pos_diff = (double)(ra_pos_target - ra_pos_now);
}

void set_ra_second_target(int32_t ra_second) {
	float ra_second_ratio = 24*60*60;
	ra_pos_target = (float) ra_second / ra_second_ratio * ra_ustep_per_rev;
}

void set_dec_second(int32_t dec_second) { //overwrite current setttings (SYNC)
	int64_t dec_pos_tmp = dec_pos_now + dec_pos_offset;
	if (dec_pos_tmp < 0) dec_pos_tmp += dec_ustep_per_rev;
	float tmp = (float) (dec_pos_tmp) / dec_ustep_per_rev;
	float dec_second_ratio = 90*60*60;
	//considering meridian
	if (tmp + ((float)dec_second/4.0/dec_second_ratio)+0.25 < 0.5) { //align to closer point
		tmp = -((float)dec_second/4.0/dec_second_ratio)+0.25;
	} else {
		tmp = ((float)dec_second/4.0/dec_second_ratio)+0.75;
	}
	dec_pos_now = (int64_t) (tmp * dec_ustep_per_rev);
}

void set_dec_second_target(int32_t dec_second) {
	int64_t dec_pos_tmp = dec_pos_now + dec_pos_offset;
	if (dec_pos_tmp < 0) dec_pos_tmp += dec_ustep_per_rev;
	float tmp = (float) (dec_pos_tmp) / dec_ustep_per_rev;
	float dec_second_ratio = 90*60*60;
	//considering meridian
	if (tmp < 0.5) {
		tmp = -((float)dec_second/4.0/dec_second_ratio)+0.25;
	} else {
		tmp = ((float)dec_second/4.0/dec_second_ratio)+0.75;
	}
	dec_pos_target = (int64_t) (tmp * dec_ustep_per_rev);
}

void sync() {
	ra_pos_diff = (ra_pos_target - ra_pos_now) % ra_ustep_per_rev;
	dec_pos_offset = (dec_pos_now - dec_pos_target) % dec_ustep_per_rev;
}

int atoi_2(uint8_t* input) {
	int result;
	if (input[0] >= 48 && input[0] < 58 && input[1] >= 48 && input[1] < 58) {
		result = (input[0] - 48) * 10 + (input[1] - 48);
	} else {
		result = 0;
	}
	return result;
}

void goto_slew() {

}

void serial_decode() {
	uint8_t output[100];
	int32_t tmp;
	RTC_TimeTypeDef rtctime;
	RTC_DateTypeDef rtcdate;
	if (ser_pos >= 2 && ser_buf[0] == ':' && ser_buf[ser_pos-1] == '#') {
		switch (ser_buf[1]) {
			case 'A': //Alignment commands
				break;
			case 'B': //Reticule/Accessory control: unsupported
				break;
			case 'C': //Sync control
				if (ser_pos < 4) break;
				switch(ser_buf[2]) {
					case 'L': //sync to current target
						sync();
						break;
				}
				break;
			case 'D': //Distance bars: unsupported
				break;
			case 'f': //fan command
				if (ser_pos < 4) break;
				switch(ser_buf[2]) {
					case 'T': //temperature
						break;
				}
				break;
			case 'F': //focuser control: unsupported
				break;
			case 'g': //GPS commands
				break;
			case 'G':
				if (ser_pos < 4) break;
				switch (ser_buf[2]) {
					case 'a': //returns current local time in 12 hour format
						rtctime = get_time_now();
						sprintf(output, "%02d:%02d:%02d#", rtctime.Hours%12, rtctime.Minutes, rtctime.Seconds);
						uart_print(output);
						break;
					case 'C': //returns current date
						rtcdate = get_date_now();
						sprintf(output, "%02d/%02d/%02d#", rtcdate.Month, rtcdate.Date, rtcdate.Year%100);
						uart_print(output);
						break;
					case 'D': //returns current declination
						tmp = get_dec_second();
						if (tmp > 0)
							sprintf(output, "%c%02ld*%02ld'%02ld#", '+', tmp/3600, tmp/60%60, tmp%60);
						else
							sprintf(output, "%c%02ld*%02ld'%02ld#", '-', -tmp/3600, -tmp/60%60, -tmp%60);
						uart_print(output);
						break;
					case 'd': //returns current object declination
						tmp = convert_dec_to_second((float)dec_pos_target / dec_ustep_per_rev);
						if (tmp > 0)
							sprintf(output, "%c%02ld*%02ld'%02ld#", '+', tmp/3600, tmp/60%60, tmp%60);
						else
							sprintf(output, "%c%02ld*%02ld'%02ld#", '-', -tmp/3600, -tmp/60%60, -tmp%60);
						uart_print(output);
						break;
					case 'G': //returns UTC offset
						if (time_offset_local % 3600 == 0) {
							sprintf(output, "s%02ld#", time_offset_local / 3600);
							uart_print(output);
						} else {
							sprintf(output, "s%02ld.%01ld#", time_offset_local / 3600, time_offset_local / 3600 * 10 % 10);
							uart_print(output);
						}
						break;
					case 'g': //returns current longitude
						sprintf(output, "%.2f", -site_lon);
						uart_print(output);
						break;
					case 'L': //returns current local time in 24 hour format
						rtctime = get_time_now();
						sprintf(output, "%02d:%02d:%02d#", rtctime.Hours, rtctime.Minutes, rtctime.Seconds);
						uart_print(output);
						break;
					case 'R': //returns current RA position
						tmp = get_ra_second();
						sprintf(output, "%02ld:%02ld:%02ld#", tmp / 3600, tmp / 60 % 60, tmp % 60);
						uart_print(output);
						break;
					case 'r': //returns current target RA position
						tmp = get_ra_second_target();
						sprintf(output, "%02ld:%02ld:%02ld#", tmp / 3600, tmp / 60 % 60, tmp % 60);
						uart_print(output);
						break;
					case 'S': //TODO: returns siderial time
						break;
					case 'T': //get tracking rate? TODO
						break;
					case 't': //returns current latitude
						sprintf(output, "%.2f", site_lat);
						uart_print(output);
						break;
					case 'V': //Telescope version informations (incompatible with LX200s)
						if (ser_pos < 5) break;
						switch(ser_buf[3]) {
							case 'D':
								uart_print(firmware_date);
								break;
							case 'N':
								uart_print(firmware_version);
								break;
							case 'P':
								uart_print(product_name);
								break;
							case 'T':
								uart_print(firmware_time);
								break;
						}
						uart_print("#");
						break;
				}
				break;
			case 'h': //Home position commands: unsupported
				break;
			case 'H': //Time format commands
				break;
			case 'I': //Initialize command: unsupported
				break;
			case 'L': //Object library command: unsupported
				break;
			case 'M': //Telescope movement
				if (ser_pos < 4) break;
				switch (ser_buf[2]) {
					case 'e':
						HC_STATE[3] = 1;
						break;
					case 'n':
						HC_STATE[0] = 1;
						break;
					case 's':
						HC_STATE[1] = 1;
						break;
					case 'w':
						HC_STATE[2] = 1;
						break;
					case 'S':
						goto_target();
						break;
				}
				hc_slew();
				break;
			case 'P': //High precision toggle
				break;
			case 'Q': //Quit movement
				if (ser_pos == 3) { //TODO: halt current GOTO
					HC_STATE[0] = 0;
					HC_STATE[1] = 0;
					HC_STATE[2] = 0;
					HC_STATE[3] = 0;
					hc_slew();
					break;
				}
				switch (ser_buf[2]) {
					case 'w':
						HC_STATE[2] = 0;
						break;
					case 'n':
						HC_STATE[0] = 0;
						break;
					case 's':
						HC_STATE[1] = 0;
						break;
					case 'e':
						HC_STATE[3] = 0;
						break;
				}
				hc_slew();
				break;
			case 'r': //Field derotator: unsupported
				break;
			case 'R': //Slew rate commands
				if (ser_pos < 4) break;
				switch (ser_buf[2]) {
					case 'C':
						gotospeed_hc_ra = gotospeed_table[1];
						gotospeed_hc_dec = gotospeed_table[1];
						break;
					case 'G':
						gotospeed_hc_ra = gotospeed_table[0];
						gotospeed_hc_dec = gotospeed_table[0];
						break;
					case 'M':
						gotospeed_hc_ra = gotospeed_table[2];
						gotospeed_hc_dec = gotospeed_table[2];
						break;
					case 'S':
						gotospeed_hc_ra = gotospeed_table[3];
						gotospeed_hc_dec = gotospeed_table[3];
						break;
					case 'A':
						if (ser_pos < 8) break;
						break;
					case 'E':
						if (ser_pos < 8) break;
						break;
				}
				break;
			case 'S': //Set command
				if (ser_pos < 4) break;
				switch (ser_buf[2]) {
					case 'C': //set date
						if (ser_pos == 12) {
							int yy, mm, dd;
							mm = atoi_2(ser_buf+3);
							dd = atoi_2(ser_buf+6);
							yy = atoi_2(ser_buf+9);
							if (0 < mm && mm < 13 && 0 < dd && dd < 32) {
								RTC_DateTypeDef datenow;
								datenow.Year = yy;
								datenow.Month = mm;
								datenow.Date = dd;
								set_date_now(datenow);
							} else {
								uart_print("0");
							}
							uart_print("1");
						} else {
							uart_print("0");
						}
						break;
					case 'd': //set dec
						if (ser_pos == 10 || ser_pos == 13) {
							int32_t dd, mm, ss, sign;
							int32_t dec_second;
							dd = atoi_2(ser_buf+4);
							mm = atoi_2(ser_buf+7);
							ss = 0;
							if (ser_pos == 13) ss = atoi_2(ser_buf+10);
							sign = (ser_buf[3]=='-')?-1:1;
							dec_second = (dd*60*60 + mm*60 + ss) * sign;
							set_dec_second_target(dec_second);
							uart_print("1");
						} else {
							uart_print("0");
						}
						break;
					case 'r': //set ra
						if (ser_pos == 11 || ser_pos == 12) {
							int32_t hh, mm, ss = 0;
							int32_t ra_second;
							hh = atoi_2(ser_buf+3);
							mm = atoi_2(ser_buf+6);
							if (ser_pos == 11) {
								if (ser_buf[9] >= 48 && ser_buf[9] < 58) {
									ss = 6 * (ser_buf[9] - 48);
								}
							} else {
								ss = atoi_2(ser_buf+9);
							}
							ra_second = (hh*60*60 + mm*60 + ss);
							set_ra_second_target(ra_second);
							uart_print("1");
						} else {
							uart_print("0");
						}
						break;
					case 'G': //Set local time - UTC
						if (ser_pos == 9) {
							int32_t sign = ser_buf[3] == '+'?1:-1;
							int32_t hour = 0;
							hour += (ser_buf[4] - 48) * 100;
							hour += (ser_buf[5] - 48) * 10;
							hour += (ser_buf[7] - 48);
							hour *= sign;

							struct tm time_tmp;
							RTC_TimeTypeDef timenow;
							RTC_DateTypeDef datenow;
							timenow = get_time_now();
							datenow = get_date_now();
							time_tmp.tm_year = datenow.Year+100; // 1900 vs 2000
							time_tmp.tm_mon = datenow.Month-1;
							time_tmp.tm_mday = datenow.Date;
							time_tmp.tm_wday = datenow.WeekDay;
							time_tmp.tm_hour = timenow.Hours;
							time_tmp.tm_min = timenow.Minutes;
							time_tmp.tm_sec = timenow.Seconds;
							time_t time_t_tmp = mktime(&time_tmp);
							time_t_tmp -= time_offset_local;
							time_offset_local = hour * 3600 / 10;
							time_t_tmp += time_offset_local;

							struct tm time_new;
							RTC_TimeTypeDef timenow_new;
							RTC_DateTypeDef datenow_new;

							time_new = *gmtime(&time_t_tmp);
							timenow_new.Hours = time_new.tm_hour;
							timenow_new.Minutes = time_new.tm_min;
							timenow_new.Seconds = time_new.tm_sec;
							datenow_new.Year = time_new.tm_year-100; // 1900 vs 2000
							datenow_new.Month = time_new.tm_mon+1;
							datenow_new.Date = time_new.tm_mday;
							datenow_new.WeekDay = time_new.tm_wday;
							set_time_now(timenow_new);
							set_date_now(datenow_new);
							uart_print("1");
						} else {
							uart_print("0");
						}
						break;
					case 'L': //Set local time
						if (ser_pos == 12) {
							int32_t hh, mm, ss;
							hh = atoi_2(ser_buf+3);
							mm = atoi_2(ser_buf+6);
							ss = atoi_2(ser_buf+9);
							RTC_TimeTypeDef timenow;
							//RTC_DateTypeDef datenow;
							timenow = get_time_now();
							//datenow = get_date_now();
							//int32_t yy = datenow.Year, mmm = datenow.Month, dd = datenow.Date;
							timenow.Hours = (uint8_t)hh;
							timenow.Minutes = (uint8_t)mm;
							timenow.Seconds = (uint8_t)ss;
							set_time_now(timenow);
							uart_print("1");
						} else {
							uart_print("0");
						}
						break;
				}
				break;
			case 'T': //Tracking commands
				break;
			case 'U': //Precision toggle
			break;
			case 'W': //Site select
			break;
			case 'J':
				if (ser_pos < 4) break;
				switch (ser_buf[2]) {
					case 'R': //reset
						if (ser_pos < 5) break;
						switch (ser_buf[3]) {
							case 'r': //reset RA
								reset_motor(RA);
								uart_print("Reset RA");
								break;
							case 'd': //reset DEC
								reset_motor(DEC);
								uart_print("Reset DEC");
								break;
						}
						break;
					case 'D': //toggle debug
						debugmode = 1 - debugmode;
						break;
				}
				break;
		}
	}
}

void melody_add(int note, int octave, int length) {
	float freq, counter;
	octave += 1;
	switch (note) {
	case 'C':
		freq = C_Hz * pow(2, octave);
		break;
	case 'D':
		freq = D_Hz * pow(2, octave);
		break;
	case 'E':
		freq = E_Hz * pow(2, octave);
		break;
	case 'F':
		freq = F_Hz * pow(2, octave);
		break;
	case 'G':
		freq = G_Hz * pow(2, octave);
		break;
	case 'A':
		freq = A_Hz * pow(2, octave);
		break;
	case 'B':
		freq = B_Hz * pow(2, octave);
		break;
	case 'd':
		freq = d_Hz * pow(2, octave);
		break;
	case 'e':
		freq = e_Hz * pow(2, octave);
		break;
	case 'g':
		freq = g_Hz * pow(2, octave);
		break;
	case 'a':
		freq = a_Hz * pow(2, octave);
		break;
	case 'b':
		freq = b_Hz * pow(2, octave);
		break;
	default:
		freq = 0;
	}
	if (freq == 0) counter = 0;
	else counter = 10500000.0 / freq;
	melody_queue[melody_play_cnt] = counter;
	melody_len_queue[melody_play_cnt] = length;
	melody_play_cnt++;
}

void uart_print(uint8_t* output) {
	HAL_UART_Transmit(&huart1, output, strlen(output), HAL_MAX_DELAY);
}

void uart_printi(int output) {
	uint8_t uart_buf[30];
	sprintf(uart_buf, "%d", output);
	uart_print(uart_buf);
}

void uart_printf(float output) {
	uint8_t uart_buf[30];
	sprintf(uart_buf, "%lf", output);
	uart_print(uart_buf);
}

void uart_printlli(int64_t output) {
	uint8_t uart_buf[30];
	sprintf(uart_buf, "%lld", output);
	uart_print(uart_buf);
}

RTC_TimeTypeDef get_time_now() {
	RTC_TimeTypeDef tmp_time;
	RTC_DateTypeDef tmp_date;
	HAL_RTC_GetTime(&hrtc, &tmp_time, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &tmp_date, RTC_FORMAT_BIN);
	return tmp_time;
}

RTC_DateTypeDef get_date_now() {
	RTC_TimeTypeDef tmp_time;
	RTC_DateTypeDef tmp_date;
	HAL_RTC_GetTime(&hrtc, &tmp_time, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &tmp_date, RTC_FORMAT_BIN);
	return tmp_date;
}

void set_time_now(RTC_TimeTypeDef time_now) {
	HAL_RTC_SetTime(&hrtc, &time_now, RTC_FORMAT_BIN);
}

void set_date_now(RTC_DateTypeDef date_now) {
	HAL_RTC_SetDate(&hrtc, &date_now, RTC_FORMAT_BIN);
}

void serial_process() {
	if (ser_buf[0] != ':') {
		ser_pos = 0;
	} else if (ser_buf[ser_pos-1] == '#') {
		serial_decode();
		ser_pos = 0;
	}
	if (ser_pos > ser_bufsize) { //Useless?
		ser_pos = 0;
	}
} //TODO: check if it works

void goto_target() {
	float ra_togo = ra_pos_target - (ra_pos_now + ra_pos_diff);
	float dec_togo = dec_pos_target - dec_pos_now; //TODO: meridian flip?
	//TODO: goto

	float ra_acc_time = 0, ra_acc_length_ustep = 0, ra_spd_calc = 0;
	float dec_acc_time = 0, dec_acc_length_ustep = 0, dec_spd_calc = 0;

	if ((state_ra_slew =! STATE_SLEW_WAIT) || (state_dec_slew != STATE_SLEW_WAIT)) {
		//When any motor is busy
		return;
	}

	//RA acc calculation
	ra_spd_calc = ra_spd_now;
	while(ra_spd_calc <= gotospeed_hc_ra) {
		ra_acc_time += 1.0/tim_acc_freq;
		ra_acc_length_ustep += ra_spd_calc;
		ra_spd_calc += ra_acc / tim_acc_freq;
	}

	dec_spd_calc = dec_spd_now;
	while(dec_spd_calc <= gotospeed_hc_dec) {
		dec_acc_time += 1.0/tim_acc_freq;
		dec_acc_length_ustep += dec_spd_calc;
		dec_spd_calc += dec_acc / tim_acc_freq;
	}

	//TODO: real goto
}

//TODO: codes for motor driver
//Code to control motor driving current
void set_current(int axis, float current) {

}

//TODO: codes for motor driver
void set_ustep(int axis, int step) {
	if (axis == RA) {
		ra_ustep_now = step;
		if (step == ra_ustep_fast) {
			state_ra_ustepmode = MODE_FAST;
		} else if (step == ra_ustep_slow) {
			state_ra_ustepmode = MODE_SLOW;
		}
	} else if (axis == DEC) {
		dec_ustep_now = step;
		if (step == dec_ustep_fast) {
			state_dec_ustepmode = MODE_FAST;
		} else if (step == dec_ustep_slow) {
			state_dec_ustepmode = MODE_SLOW;
		}
	}
}

void stop_motor(int axis) {
	if (axis == RA) {
	} else if (axis == DEC) {
	}
}

void start_motor(int axis) {
	if (axis == RA) {
	} else if (axis == DEC) {
	}
}

void reset_motor(int axis) {
	if (axis == RA) {
	} else if (axis == DEC) {
	}
}
//

//motor indexing
int tim_counter_ra, tim_counter_dec;
int tim_pulse_ra, tim_pulse_dec;
void tim_ra_callback() {
	tim_counter_ra++;
	tim_counter_ra_remainder++;

	if (tim_counter_ra > ra_tick_per_step) { //will count more, therefore run slower than actual speed.
		tim_counter_ra = 0;
		tim_pulse_ra = 1 - tim_pulse_ra;
		if (tim_pulse_ra == 1){
			if (dir_ra_now == 1) {
				set_ra_now(+(int64_t)tim_pulse_ra * (int64_t)ra_motor_ustep_max / (int64_t)ra_ustep_now);
			} else {
				set_ra_now(-(int64_t)tim_pulse_ra * (int64_t)ra_motor_ustep_max / (int64_t)ra_ustep_now);
			}
		}
		//TODO: Pulse
		//digitalWrite(pin_ra_step, tim_pulse_ra);
	} else if (tim_counter_ra > tim_counter_ra_max) {
		tim_counter_ra = 0;
	}
	if (tim_counter_ra_remainder > ra_timer_remainder) {
		tim_counter_ra_remainder = 0;
		tim_pulse_ra = 1 - tim_pulse_ra;
		if (tim_pulse_ra == 1){
			if (dir_ra_now == 1) {
				set_ra_now(+(int64_t)tim_pulse_ra * (int64_t)ra_motor_ustep_max / (int64_t)ra_ustep_now);
			} else {
				set_ra_now(-(int64_t)tim_pulse_ra * (int64_t)ra_motor_ustep_max / (int64_t)ra_ustep_now);
			}
		}
		//TODO: Pulse
		//digitalWrite(pin_ra_step, tim_pulse_ra);
	} else if (tim_counter_ra > tim_counter_ra_max) {
		tim_counter_ra = 0;
	}
}

void tim_dec_callback() {
	tim_counter_dec++;
	if (tim_counter_dec > dec_tick_per_step) {
		tim_counter_dec = 0;
		tim_pulse_dec = 1 - tim_pulse_dec;
		if (tim_pulse_dec == 1){
			if (dir_dec_now == 1) {
				set_dec_now(+(int64_t)tim_pulse_dec * (int64_t)dec_motor_ustep_max / (int64_t)dec_ustep_now);
			} else {
				set_dec_now(-(int64_t)tim_pulse_dec * (int64_t)dec_motor_ustep_max / (int64_t)dec_ustep_now);
			}
		}
		//TODO: pulse
		//digitalWrite(pin_dec_step, tim_pulse_dec);
	} else if (tim_counter_dec > tim_counter_dec_max) {
		tim_counter_dec = 0;
	}
}

void applydir() {
	if (dir_ra_now != dir_ra_target) {
		if (dir_ra_target == 0) {
			if (DIRSET_RA == 0) {
				HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, GPIO_PIN_RESET);
			} else {
				HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, GPIO_PIN_SET);
			}
		} else if (dir_ra_target == 1) {
			if (DIRSET_RA == 0) {
				HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, GPIO_PIN_SET);
			} else {
				HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, GPIO_PIN_RESET);
			}
		}
		dir_ra_now = dir_ra_target;
	}
	if (dir_dec_now != dir_dec_target) {
		if (dir_dec_target == 0) {
			if (DIRSET_DEC == 0) {
				HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, GPIO_PIN_RESET);
			} else {
				HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, GPIO_PIN_SET);
			}
		} else if (dir_dec_target == 1) {
			if (DIRSET_DEC == 0) {
				HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, GPIO_PIN_SET);
			} else {
				HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, GPIO_PIN_RESET);
			}
		}
		dir_dec_now = dir_dec_target;
	}
}

void tim_acc_callback() {
	if (abs(ra_spd_now - ra_spd_target) < epsilon) {
		//Do nothing
		state_dyn_ra = STATE_DYN_UNIFRM;
	} else {
		float ra_targetspeed = ra_spd_target + guide_ra_now;
		if (ra_spd_now < ra_targetspeed) {
			ra_spd_now += ra_acc / tim_acc_freq;
			if (ra_spd_now > ra_targetspeed) { //finished changing speed
				ra_spd_now = ra_targetspeed;
				if (state_ra_slew == STATE_SLEW_HC_STOPPING) state_ra_slew = STATE_SLEW_WAIT;
			}
		} else if (ra_spd_now > ra_targetspeed) {
			ra_spd_now -= ra_dacc / tim_acc_freq;
			if (ra_spd_now < ra_targetspeed) { //finished changing speed
				ra_spd_now = ra_targetspeed;
				if (state_ra_slew == STATE_SLEW_HC_STOPPING) state_ra_slew = STATE_SLEW_WAIT;
			}
		}
		if (abs(ra_spd_now) > ra_fast_crit_ustep && state_ra_ustepmode == MODE_SLOW) {
			//if(ra_pos_now % (ra_motor_ustep_max / ra_ustep_fast) == 0)
				set_ustep(RA, ra_ustep_fast);
		} else if (abs(ra_spd_now) < ra_fast_crit_ustep && state_ra_ustepmode == MODE_FAST) {
			set_ustep(RA, ra_ustep_slow);
		}
		if (abs(ra_spd_now) > ra_fast_crit_current && state_ra_currentmode == MODE_SLOW) {
			set_current(RA, ra_current_fast);
		} else if (abs(ra_spd_now) < ra_fast_crit_current && state_ra_currentmode == MODE_FAST) {
			set_current(RA, ra_current_slow);
		}
		ra_tick_per_step = tim_ra_freq / (ra_spd_ratio_sdrl * abs(ra_spd_now) * ra_ustep_now / ra_motor_ustep_max + epsilon) / 2; //divide by 2 for pulse

		float ra_tps_ceil = ceil(ra_tick_per_step);
		float ra_tps_error = ra_tps_ceil - ra_tick_per_step;
		ra_timer_remainder = 1 / (ra_tps_error / ra_tick_per_step + epsilon) * ra_tps_ceil / 2;

		if (ra_spd_now < 0) {
			dir_ra_target = 0;
		} else {
			dir_ra_target = 1;
		}
		if (dir_ra_now != dir_ra_target) {
			applydir();
		}
	}
	ra_pos_diff += ((-1.0)/tim_acc_freq) * ra_spd_ratio_sdrl;

	if (abs(dec_spd_now - dec_spd_target) < epsilon) {
		//Do nothing
		state_dyn_dec = STATE_DYN_UNIFRM;
	} else {
		float dec_targetspeed = dec_spd_target + guide_dec_now;
		if (dec_spd_now < dec_targetspeed) {
			dec_spd_now += dec_acc / tim_acc_freq;
			if (dec_spd_now > dec_targetspeed) { //finished changing speed
				dec_spd_now = dec_targetspeed;
				if (state_dec_slew == STATE_SLEW_HC_STOPPING) state_dec_slew = STATE_SLEW_WAIT;
			}
		} else if (dec_spd_now > dec_targetspeed) {
			dec_spd_now -= dec_dacc / tim_acc_freq;
			if (dec_spd_now < dec_targetspeed) { //finished changing speed
				dec_spd_now = dec_targetspeed;
				if (state_dec_slew == STATE_SLEW_HC_STOPPING) state_dec_slew = STATE_SLEW_WAIT;
			}
		}
		if (abs(dec_spd_now) > dec_fast_crit_ustep && state_dec_ustepmode == MODE_SLOW) {
			//if(dec_pos_now % (dec_motor_ustep_max / dec_ustep_fast) == 0)
				set_ustep(DEC, ra_ustep_fast);
		} else if (abs(dec_spd_now) < dec_fast_crit_ustep && state_dec_ustepmode == MODE_FAST) {
			set_ustep(DEC, ra_ustep_slow);
		}
		if (abs(dec_spd_now) > dec_fast_crit_current && state_dec_currentmode == MODE_SLOW) {
			set_current(DEC, dec_current_fast);
		} else if (abs(dec_spd_now) < dec_fast_crit_current && state_dec_currentmode == MODE_FAST) {
			set_current(DEC, dec_current_slow);
		}
		dec_tick_per_step = tim_dec_freq / (dec_spd_ratio_sdrl * abs(dec_spd_now) * dec_ustep_now / dec_motor_ustep_max + epsilon) / 2; //divide by 2 for pulse

		if (dec_spd_now < 0) {
			dir_dec_target = 0;
		} else {
			dir_dec_target = 1;
		}
		if (dir_dec_now != dir_dec_target) {
			applydir();
		}
	}
}

//RA, DEC setup
void set_ra_now(int64_t radelta) {
	ra_pos_now += radelta;
	if (ra_pos_now > ra_ustep_per_rev) {
		ra_pos_now -= ra_ustep_per_rev;
	} else if (ra_pos_now < 0) {
		ra_pos_now += ra_ustep_per_rev;
	}
}

void set_dec_now(int64_t decdelta) {
	dec_pos_now += decdelta;
	if (dec_pos_now > dec_ustep_per_rev) {
		dec_pos_now -= dec_ustep_per_rev;
	} else if (dec_pos_now < 0) {
		dec_pos_now += dec_ustep_per_rev;
	}
}

void debug() {
	if (adc_mode == 0) {
		HAL_ADC_Start_IT(&hadc1);
		adc_mode = 1;
	} else {
		HAL_ADC_Start_IT(&hadc1);
		adc_mode = 0;
	}

	if (debugmode == 1) {
		dbg_counter++;
		int tmp;
		uint8_t output[100];

		uart_print("Counter: ");
		uart_printi(dbg_counter);
		uart_print("\n");

		uart_print("RA speed now: ");
		uart_printf(ra_spd_now);
		uart_print("\n");
		uart_print("RA speed target: ");
		uart_printf(ra_spd_target);
		uart_print("\n");
		uart_print("RA ustep now: ");
		uart_printi(ra_ustep_now);
		uart_print("\n");
		uart_print("RA tick per step now: ");
		uart_printf(ra_tick_per_step);
		uart_print("\n");
		uart_print("RA tick remainder: ");
		uart_printi(ra_timer_remainder);
		uart_print("\n");
		uart_print("RA pos now(motor): ");
		uart_printlli(ra_pos_now);
		uart_print("\n");
		uart_print("RA pos pointing: ");
		uart_printf(get_ra_second()*ra_spd_ratio_solar);
		uart_print("\n");
		uart_print("RA motor current now(A): ");
		uart_printf(ra_drivecurrent_now);
		uart_print("\n");
		uart_print("RA dir now: ");
		uart_printi(dir_ra_now);
		uart_print("\n");
		uart_print("RA sidereal ratio: ");
		uart_printf(ra_spd_ratio_sdrl);
		uart_print("\n");
		uart_print("RA offset now: ");
		uart_printlli(ra_pos_offset);
		uart_print("\n");
		uart_print("RA diff: ");
		uart_printf(ra_pos_diff);
		uart_print("\n");
		tmp = get_ra_second();
		sprintf(output, "%02d %02d %02d", tmp / 3600, tmp / 60 % 60, tmp % 60);
		uart_print(output);
		uart_print("\n");

		uart_print("DEC speed now: ");
		uart_printf(dec_spd_now);
		uart_print("\n");
		uart_print("DEC speed target: ");
		uart_printf(dec_spd_target);
		uart_print("\n");
		uart_print("DEC ustep now: ");
		uart_printi(dec_ustep_now);
		uart_print("\n");
		uart_print("DEC tick per step now: ");
		uart_printf(dec_tick_per_step);
		uart_print("\n");
		uart_print("DEC pos now: ");
		uart_printlli(dec_pos_now);
		uart_print("\n");
		uart_print("DEC motor current now(A): ");
		uart_printf(dec_drivecurrent_now);
		uart_print("\n");
		uart_print("DEC dir now: ");
		uart_printi(dir_dec_now);
		uart_print("\n");
		uart_print("DEC offset now: ");
		uart_printlli(dec_pos_offset);
		uart_print("\n");
		tmp = get_dec_second();
		if (tmp > 0)
			sprintf(output, "%c%02d %02d %02d", '+', tmp/3600, tmp/60%60, tmp%60);
		else
			sprintf(output, "%c%02d %02d %02d", '-', -tmp/3600, -tmp/60%60, -tmp%60);
		uart_print(output);
		uart_print("\n");

		uart_print("Guide state ");
		uart_printi(GUIDE_STATE[0]);
		uart_printi(GUIDE_STATE[1]);
		uart_printi(GUIDE_STATE[2]);
		uart_printi(GUIDE_STATE[3]);
		uart_print("\n");

		uart_print("HC state ");
		uart_printi(HC_STATE[0]);
		uart_printi(HC_STATE[1]);
		uart_printi(HC_STATE[2]);
		uart_printi(HC_STATE[3]);
		uart_print("\n");

		uart_print("Input V(mV): ");
		uart_printf(vin);
		uart_print("Input I(mA): ");
		uart_printf(iin);
		uart_print("\n");



		RTC_TimeTypeDef time_tmp = get_time_now();
		uart_print("Time now:");
		sprintf(output, "%02d:%02d:%02d", time_tmp.Hours, time_tmp.Minutes, time_tmp.Seconds);
		uart_print(output);
		uart_print("\n");

		uart_print("\n\n");

		dbg_counter++;
	}
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	int adc_tmp;
	if (adc_mode == 0) {
		adc_tmp = HAL_ADC_GetValue(hadc);
		vin = (float)adc_tmp * 500 / 4096.0 * 1000.0;
	} else {
		adc_tmp = HAL_ADC_GetValue(hadc);
		iin = (float)adc_tmp / 16384.0 / 15.0 * 20.0 * 1000.0;
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_I2C1_Init();
  MX_RTC_Init();
  MX_TIM2_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_TIM12_Init();
  MX_TIM13_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */

  gotospeed_table[0] = 0.5;
  gotospeed_table[1] = fast_spd_max / 10.0;
  gotospeed_table[2] = fast_spd_max / 3.0;
  gotospeed_table[3] = fast_spd_max / 1.0;

  gotospeed_hc_ra = fast_spd_max;
  gotospeed_hc_dec = fast_spd_max;

  ra_tick_per_step = tim_counter_ra_max;
  dec_tick_per_step = tim_counter_dec_max;

  led_set(0, 1);
  led_set(1, 0);
  led_set(2, 0);
  led_set(3, 0);


  //calculate some vars
  ra_ustep_per_rev = (int64_t) ((float) ra_motor_step * ra_motor_reducer * ra_final_reducer * ra_motor_ustep_max);
  dec_ustep_per_rev = (int64_t) ((float) dec_motor_step * dec_motor_reducer * dec_final_reducer * dec_motor_ustep_max);

  ra_spd_ratio_sdrl = ra_ustep_per_rev / sdrl_day;
  ra_spd_ratio_solar = ra_ustep_per_rev / solar_day;
  dec_spd_ratio_sdrl = dec_ustep_per_rev / sdrl_day;

  if (debugmode == 1) {
  	uart_print("Carina EQDriver startup\n");
  	uart_print("RA step per rev: ");
  	uart_printlli(ra_ustep_per_rev);
  	uart_print("\n");
  	uart_print("DEC step per rev: ");
  	uart_printlli(dec_ustep_per_rev);
  	uart_print("\n");
  	uart_print("RA step per sec for sidereal: ");
  	uart_printf(ra_spd_ratio_sdrl);
  	uart_print("\n");
  	uart_print("DEC step per sec for sidereal: ");
  	uart_printf(dec_spd_ratio_sdrl);
  	uart_print("\n");
  	uart_print("\n");
  }

  //initialize

  //HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  /*melody_add('C', 1, 4);
  melody_add('E', 1, 4);
  melody_add('G', 1, 4);
  melody_add('C', 2, 8);*/
  melody_add('D', 1, 4);
  melody_add('B', 1, 4);
  melody_add('A', 1, 4);
  melody_add('G', 1, 4);

  melody_add('D', 1, 11);
  melody_add('.', 1, 1);
  melody_add('D', 1, 1);
  melody_add('.', 1, 1);
  melody_add('D', 1, 1);
  melody_add('.', 1, 1);

  melody_add('D', 1, 4);
  melody_add('B', 1, 4);
  melody_add('A', 1, 4);
  melody_add('G', 1, 4);

  melody_add('E', 1, 12);
  melody_add('.', 1, 4);

  melody_add('E', 1, 4);
  melody_add('C', 2, 4);
  melody_add('B', 1, 4);
  melody_add('A', 1, 4);

  melody_add('g', 1, 12);
  melody_add('.', 1, 4);

  melody_add('D', 2, 3);
  melody_add('.', 1, 1);
  melody_add('D', 2, 4);
  melody_add('C', 2, 4);
  melody_add('A', 1, 4);

  melody_add('B', 1, 16);

  melody_add('D', 1, 4);
  melody_add('B', 1, 4);
  melody_add('A', 1, 4);
  melody_add('G', 1, 4);

  melody_add('D', 1, 12);
  melody_add('.', 1, 4);

  melody_add('D', 1, 4);
  melody_add('B', 1, 4);
  melody_add('A', 1, 4);
  melody_add('G', 1, 4);

  melody_add('E', 1, 11);
  melody_add('.', 1, 1);
  melody_add('E', 1, 3);
  melody_add('.', 1, 1);

  melody_add('E', 1, 4);
  melody_add('C', 2, 4);
  melody_add('B', 1, 4);
  melody_add('A', 1, 4);

  melody_add('D', 2, 3);
  melody_add('.', 1, 1);
  melody_add('D', 2, 3);
  melody_add('.', 1, 1);
  melody_add('D', 2, 3);
  melody_add('.', 1, 1);
  melody_add('D', 2, 3);
  melody_add('.', 1, 1);

  melody_add('E', 2, 4);
  melody_add('D', 2, 4);
  melody_add('C', 2, 4);
  melody_add('A', 1, 4);

  melody_add('G', 1, 12);
  melody_add('.', 1, 4);

  melody_add('B', 1, 3);
  melody_add('.', 1, 1);
  melody_add('B', 1, 3);
  melody_add('.', 1, 1);
  melody_add('B', 1, 7);
  melody_add('.', 1, 1);

  melody_add('B', 1, 3);
  melody_add('.', 1, 1);
  melody_add('B', 1, 3);
  melody_add('.', 1, 1);
  melody_add('B', 1, 7);
  melody_add('.', 1, 1);

  melody_add('B', 1, 4);
  melody_add('D', 2, 4);
  melody_add('G', 1, 6);
  melody_add('A', 1, 2);

  melody_add('B', 1, 12);
  melody_add('.', 1, 4);

  melody_add('C', 2, 3);
  melody_add('.', 1, 1);
  melody_add('C', 2, 3);
  melody_add('.', 1, 1);
  melody_add('C', 2, 5);
  melody_add('.', 1, 1);
  melody_add('C', 2, 1);
  melody_add('.', 1, 1);

  melody_add('C', 2, 4);
  melody_add('B', 1, 3);
  melody_add('.', 1, 1);
  melody_add('B', 1, 3);
  melody_add('.', 1, 1);
  melody_add('B', 1, 1);
  melody_add('.', 1, 1);
  melody_add('B', 1, 1);
  melody_add('.', 1, 1);

melody_add('B', 1, 4);
melody_add('A', 1, 3);
melody_add('.', 1, 1);
melody_add('A', 1, 4);
melody_add('B', 1, 4);

melody_add('A', 1, 8);
melody_add('D', 2, 8);

melody_add('B', 1, 3);
melody_add('.', 1, 1);
melody_add('B', 1, 3);
melody_add('.', 1, 1);
melody_add('B', 1, 7);
melody_add('.', 1, 1);

melody_add('B', 1, 3);
melody_add('.', 1, 1);
melody_add('B', 1, 3);
melody_add('.', 1, 1);
melody_add('B', 1, 7);
melody_add('.', 1, 1);

melody_add('B', 1, 4);
melody_add('D', 2, 4);
melody_add('G', 1, 6);
melody_add('A', 1, 2);

melody_add('B', 1, 12);
melody_add('.', 1, 4);

melody_add('C', 2, 3);
melody_add('.', 1, 1);
melody_add('C', 2, 3);
melody_add('.', 1, 1);
melody_add('C', 2, 5);
melody_add('.', 1, 1);
melody_add('C', 2, 1);
melody_add('.', 1, 1);

melody_add('C', 2, 4);
melody_add('B', 1, 3);
melody_add('.', 1, 1);
melody_add('B', 1, 3);
melody_add('.', 1, 1);
melody_add('B', 1, 1);
melody_add('.', 1, 1);
melody_add('B', 1, 1);
melody_add('.', 1, 1);

melody_add('D', 2, 3);
melody_add('.', 1, 1);
melody_add('D', 2, 4);
melody_add('C', 2, 4);
melody_add('A', 1, 4);

melody_add('G', 1, 12);
melody_add('.', 1, 4);

  HAL_TIM_Base_Start_IT(&htim10);
  HAL_TIM_Base_Start_IT(&htim11);
  HAL_TIM_Base_Start_IT(&htim12);
  HAL_TIM_Base_Start_IT(&htim13);
  HAL_TIM_Base_Start_IT(&htim14);


  //setup 1x sidereal tracking mode for ra
  ra_pos_now = 0;
  ra_spd_target = 1;
  ra_spd_now = 0;
  dir_ra_target = 1;
  dir_ra_now = 1;
  state_ra = STATE_TRACK;
  state_dyn_ra = STATE_DYN_ACC;

  //stop for dec
  dec_pos_now = 0;
  dec_spd_target = 0;
  dec_spd_now = 0;
  dir_dec_target = 1;
  dir_dec_now = 1;
  state_dec = STATE_STOP;

  //initial setup: slow mode
  set_ustep(RA, ra_ustep_slow);
  set_ustep(DEC, dec_ustep_slow);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_HSE_DIV30;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 124;
  hrtc.Init.SynchPrediv = 3199;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x3;
  sTime.Minutes = 0x4;
  sTime.Seconds = 0x50;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_SET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_THURSDAY;
  sDate.Month = RTC_MONTH_DECEMBER;
  sDate.Date = 0x17;
  sDate.Year = 0x20;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 2000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 23;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 49;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 23;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 49;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 83;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 9999;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */

}

/**
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 8399;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 9999;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 2099;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 2499;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, ESP_PROG_Pin|NENBL2_Pin|STEP1_Pin|DIR1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, STEP2_Pin|DIR2_Pin|STDBY2_Pin|STDBY1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, NENBL1_Pin|LED_Y_Pin|LED_B_Pin|LED_W_Pin
                          |LED_G_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : ESP_PROG_Pin */
  GPIO_InitStruct.Pin = ESP_PROG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ESP_PROG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GUIDE1_Pin GUIDE2_Pin GUIDE3_Pin GUIDE4_Pin */
  GPIO_InitStruct.Pin = GUIDE1_Pin|GUIDE2_Pin|GUIDE3_Pin|GUIDE4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA3 INDEX2_Pin INDEX1_Pin
                           DIAG1_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|INDEX2_Pin|INDEX1_Pin
                          |DIAG1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : STEP2_Pin DIR2_Pin STDBY2_Pin STDBY1_Pin */
  GPIO_InitStruct.Pin = STEP2_Pin|DIR2_Pin|STDBY2_Pin|STDBY1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : DIAG2_Pin */
  GPIO_InitStruct.Pin = DIAG2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DIAG2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : NENBL2_Pin STEP1_Pin DIR1_Pin */
  GPIO_InitStruct.Pin = NENBL2_Pin|STEP1_Pin|DIR1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : NENBL1_Pin LED_Y_Pin LED_B_Pin LED_W_Pin
                           LED_G_Pin */
  GPIO_InitStruct.Pin = NENBL1_Pin|LED_Y_Pin|LED_B_Pin|LED_W_Pin
                          |LED_G_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

}

/* USER CODE BEGIN 4 */


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
