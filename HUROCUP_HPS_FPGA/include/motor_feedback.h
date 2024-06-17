#ifndef MOTOR_FEEDBACK_H_
#define MOTOR_FEEDBACK_H_

/******************* Include libarary*********************/
#include <stdio.h>
#include <string.h>
#include <sstream>      // save_data
#include <fstream>      // save_data
#include <vector>       // save_data
#include <map>          // save_data
#include <cstring>      // save_data
/********************************************************/

/******************* Include module**********************/
#include "initial.h"
// #include "Walkinggait.h"
#include "data_module.h"
/********************************************************/

class MotorFeedback
{
public:
    MotorFeedback(Initial& init_, DataModule& data_module_);
    ~MotorFeedback();

    bool read_feedback;

    void load_motor_data_left_foot();
    void update_motor_data_left_foot();
    int motor_data_left_foot_[6];

    void load_motor_data_right_foot();
    void update_motor_data_right_foot();
    int motor_data_right_foot_[6];

    void load_motor_data_left_hand();
    void update_motor_data_left_hand();
    int motor_data_left_hand_[4];

    void load_motor_data_right_hand();
    void update_motor_data_right_hand();
    int motor_data_right_hand_[4];

    void adjust_left_foot();
    void update_adjust_left_foot();
    int adjust_left_foot_[6];
    int tmp_lf[6];

    void adjust_right_foot();
    void update_adjust_right_foot();
    int adjust_right_foot_[6];
    int tmp_rf[6];

    int torque_update_motor_data_left_foot_flag_;
    int torque_update_motor_data_right_foot_flag_;
    int torque_update_motor_data_left_hand_flag_;
    int torque_update_motor_data_right_hand_flag_;

    std::string DtoS(double value);
    std::map<std::string, std::vector<double>> map_feedback;      
    void saveData();
    void pushData();

private:
    Initial& init;
    DataModule& data_module;
    bool update_motor_data_left_foot_flag_;
    bool update_motor_data_right_foot_flag_;
    bool update_motor_data_left_hand_flag_;
    bool update_motor_data_right_hand_flag_;
    bool update_adjust_left_foot_flag_;
    bool update_adjust_right_foot_flag_;
    int name_cont_;
	    
};

#endif