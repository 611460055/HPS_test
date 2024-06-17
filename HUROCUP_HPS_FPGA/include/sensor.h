#ifndef SENSOR_H_
#define SENSOR_H_

/******************* Include libarary*********************/
#include <stdio.h>
#include <string.h>
#include <sstream>      // save_data
#include <fstream>      // save_data
#include <vector>       // save_data
#include <map>          // save_data
#include <cstring>      // save_data
/********************************************************/

#include "initial.h"
#include "motor_feedback.h"

class SensorDataProcess
{
public:
	SensorDataProcess(Initial& init_);
	~SensorDataProcess();

	void sensor_data_send();
	void sensor_package_generate(MotorFeedback& motor_feedback);
	void send_sensor_data_to_ipc();
	void load_sensor_setting();
	void update_sensor_setting();

    void load_imu();
    void update_imu();

    void set_desire_rpy();

    double rpy_[3];
    float  accel_[3];
    int gyro_[3];
    double imu_desire_[3];
    double roll_pid_[3];
    double pitch_pid_[3];
    double com_pid_[3];
    double foot_offset_[3];
    double imu_reset_[3];

    //fall down
    bool fall_Down_Flag_;
    bool stop_Walk_Flag_;
    char fall_Down_Status_; // S = stand ; F = forward fall down ;B = backward fall down
    unsigned int stand_status_package_;
        
    //press sensor 
    void load_press_left();
    void load_press_right();
    void update_press_right();
    void update_press_left();

    int press_right_[4];
    int press_left_[4];
    int data_flag = 0;
    bool sensor_request_;
    bool imu_offset_reset_;   // imu offset reset
    bool force_state_;
    bool gain_set_;
    bool roll_PID_set_;
    bool pitch_PID_set_;
    bool com_PID_set_;
    bool foot_offset_set_;
    // bool data_flag = false;

private:
    Initial& init;
    int rpy_from_fpga_[3];
    double rpy_raw_[3];
    float accel_raw_[3];
    int   gyro_raw_[3];
    double rpy_offset_[3],gyro_offset_[3];
    unsigned char sensor_data_to_ipc_[70];
    // double imu_desire_[3];
    bool update_sensor_setting_flag_;
    bool get_sensor_setting_flag_;
    bool update_imu_flag_;
    bool get_imu_flag_;
    unsigned int sensor_setting_[2];
    unsigned int imu_[6];

    // bool sensor_request_;
    // bool imu_offset_reset_;   // imu offset reset
    // bool force_state_;
    // bool gain_set_;
    // bool roll_PID_set_;
    // bool pitch_PID_set_;
    // bool com_PID_set_;
    // bool foot_offset_set_;

    //press sensor
    bool update_press_right_flag_;
    bool update_press_left_flag_;
    //right press
    unsigned int press_receive_data_right[2];
    int press_right_raw_[4];
    int press_right_offset_[4];    
    //left  press
    unsigned int press_receive_data_left[2];
    int press_left_raw_[4];
    int press_left_offset_[4];         
    //for press test end
};

#endif