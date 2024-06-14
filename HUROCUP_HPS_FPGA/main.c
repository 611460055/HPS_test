/*
This program demonstrate how to use hps communicate with FPGA through light AXI Bridge.
uses should program the FPGA by GHRD project before executing the program
refer to user manual chapter 7 for details about the demo
*/

#include "include/main.h"
 

int main()
{	
	usleep(1000 * 1000);
	init.initial_system();

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
	}

	init.clear_memory_mapping();

	return( 0 );
}