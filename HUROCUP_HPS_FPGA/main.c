/*
This program demonstrate how to use hps communicate with FPGA through light AXI Bridge.
uses should program the FPGA by GHRD project before executing the program
refer to user manual chapter 7 for details about the demo
*/

#include "include/main.h"
 

int main()
{	
	usleep(1000 * 1000);
	initial.initial_system();
	data_module.set_stand();


	while(1)
	{
		data_module.load_database();
		if(data_module.motion_execute_flag_)
		{
			// if(data_module.stand_flag)
			// {
			// 	data_module.set_stand();
			// }
			data_module.motion_execute();
		}

		/*-----------*/
		sensor.load_imu(); //獲得IMU值
		/*---壓感---*/
		sensor.load_press_left(); 
		sensor.load_press_right();
		/*----------*/
		sensor.load_sensor_setting(); //balance補償([raw,pitch,com]PID,[sup,nsup]foot_offset)
		sensor.sensor_package_generate(motor_feedback); //建立感測器資料;回傳IMU值給IPC
		
		// /*---馬達回授---*/
		if(motor_feedback.read_feedback)
		{
			motor_feedback.load_motor_data_left_foot();
			motor_feedback.load_motor_data_right_foot();
			motor_feedback.pushData();
			motor_feedback.read_feedback = false;
		}
		/*-------------*/
	}

	initial.clear_memory_mapping();

	return( 0 );
}