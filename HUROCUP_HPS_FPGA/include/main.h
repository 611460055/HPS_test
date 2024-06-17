
/*
 * main.h
 *
 *  Created on: 2019/09/11
 *      Author: Yu-Chih, Wang
 */

#ifndef MAIN_H_
#define MAIN_H_

/******************* Define******************************/
//#define main_debug    /* Turn debugging on */
/********************************************************/

/******************* Parameter **************************/

/********************************************************/

/******************* Include libarary*********************/
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/time.h>
#include "hwlib.h"
#include "socal/socal.h"
#include "socal/hps.h"
#include "socal/alt_gpio.h"
#include <iostream>
#include <Eigen/Eigen>
/********************************************************/

/******************* Include module**********************/
#include "hps_0.h"
#include "initial.h"
#include "data_module.h"
#include "motor_feedback.h"
#include "sensor.h"
/********************************************************/

Initial initial;
DataModule data_module(initial);
MotorFeedback motor_feedback(initial, data_module);
SensorDataProcess sensor(initial);

struct timeval tstart, tend;
double timeuse;

#endif /* MAIN_H_ */
