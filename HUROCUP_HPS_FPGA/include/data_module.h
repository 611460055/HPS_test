#ifndef DATA_MODULE_H_
#define DATA_MODULE_H_

/******************* Define******************************/
#define DATA_MODULE_DEBUG    /* Turn debugging on */
/********************************************************/

/******************* Parameter **************************/

/******************* Include libarary*********************/
#include <stdio.h>
#include <iostream>
#include <sstream>      // save_data
#include <fstream>      // save_data
#include <vector>       // save_data
#include <map>          // save_data
#include <cstring>      // save_data
/********************************************************/

/******************* Include module**********************/
#include "Initial.h"
// #include "Inverse_kinematic.h"
/********************************************************/

/******************** Function **************************/
// void RS232DataInterrupt( void* context, alt_u32 id );
// void Flash_Access(void);
// void MotionExecute();

/********************************************************/

using namespace std;

class DataModule
{
public:
    DataModule();
    ~DataModule();

    void load_database();
    void update_database();
    void motion_execute();
    void set_stand();
    void create_package(int angle[21], int speed[21], int motion_delay);
    unsigned short update_crc(unsigned short , unsigned char *, unsigned short);
    void do_motion();
    // void stand_button_press();
    // void stand_button_execute();

    std::string DtoS(double value);
	std::map<std::string, std::vector<double>> map_data_moudle;       
    void save_data();
    void push_data();

    unsigned char data_module_cmd_;
    bool motion_execute_flag_;
    bool stand_flag;
    bool update_stand_flag_;
    int total_angle_[21];   // output motor angle, 輸出的馬達角度
    int total_speed_[21];   // output motor speed, 輸出的馬達速度
    int base_angle_[21];
    int base_speed_[21];
    // int stand_angle[12];
    // int stand_speed[12];

    int standing_angle_[21];
    int standing_speed_[21];
private:
    bool update_database_flag_;
    int database_[21];
    unsigned char packet_char_[203];
    int name_cont_;
    
};

#endif /*DATA_MODULE_H_*/
