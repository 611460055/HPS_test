#include "include/Data_Module.h"

// extern Locus locus;
// extern InverseKinematic IK;
extern Initial init;

DataModule::DataModule()
{
	data_module_cmd_ = 0;
	std::vector<double> temp;
	if(map_data_moudle.empty())
	{
		map_data_moudle["motor_01"] = temp;
		map_data_moudle["motor_02"] = temp;
		map_data_moudle["motor_03"] = temp;
		map_data_moudle["motor_04"] = temp;
		map_data_moudle["motor_05"] = temp;
		map_data_moudle["motor_06"] = temp;
		map_data_moudle["motor_07"] = temp;
		map_data_moudle["motor_08"] = temp;
		map_data_moudle["motor_09"] = temp;
		map_data_moudle["motor_10"] = temp;
		map_data_moudle["motor_11"] = temp;
		map_data_moudle["motor_12"] = temp;
		map_data_moudle["motor_13"] = temp;
		map_data_moudle["motor_14"] = temp;
		map_data_moudle["motor_15"] = temp;
		map_data_moudle["motor_16"] = temp;
		map_data_moudle["motor_17"] = temp;
		map_data_moudle["motor_18"] = temp;
		map_data_moudle["motor_19"] = temp;
		map_data_moudle["motor_20"] = temp;
		map_data_moudle["motor_21"] = temp;
	}
}

DataModule::~DataModule()
{

}

void DataModule::load_database()
{
	int state = 0;
	int count = 0;

	for(;;)
	{
		if(state == 0)
		{
			update_database_flag_ = false;
			if(*(uint32_t *)init.p2h_set_hps_read_database_addr)
			{
				state = 1;
				continue;
			}
			else
			{
				break;
			}
		}
		else if(state == 1)
		{
			if(count <= 20)
			{
				database_[count] = *(uint32_t *)init.p2h_database_addr;
				count++;
				*(uint32_t *)init.h2p_read_database_pulse_addr = 1;
				*(uint32_t *)init.h2p_read_database_pulse_addr = 0;
				continue;
			}
			else
			{
				update_database_flag_ = true;
				motion_execute_flag_ = true;
				state = 0;
				break;
			}
		}
	}
	update_database();
}

void DataModule::update_database()
{
	if(update_database_flag_)
	{
		int i;
		short tmp_angle = 0;
		data_module_cmd_ = *(uint32_t *)init.p2h_pc_command_addr;

		for(i=0; i<21; i++)
		{
			switch(data_module_cmd_)
			{
			case 2: // standing
				stand_flag = true;

				base_speed_[i] = (database_[20-i] & 0x0000FFFF);		// base speed
				base_angle_[i] = ((database_[20-i] & 0xFFFF0000)>> 16);	// base angle, 馬達基礎角度
				
				total_speed_[i] = standing_angle_[i];	
				total_angle_[i] = standing_speed_[i];	// IK calculated angle, IK計算角度

				break;
			case 3: // execute
				total_speed_[i] = ((database_[20-i] & 0xFFFF0000)>> 16);
				tmp_angle = ((database_[20-i] & 0xFFFF0000)>> 16);

				if(tmp_angle & 0x8000)	// tmp_angle < 0
				{
					base_angle_[i] -= (tmp_angle & 0x7FFF);
					if(base_angle_[i] < 0)
						base_angle_[i] = 0;
					
					total_angle_[i] -= (tmp_angle & 0x7FFF);
					if(total_angle_[i] < 0)
						total_angle_[i] = 0;
				}
				else
				{
					base_angle_[i] += tmp_angle;
					if(base_angle_[i] > 4095)
						base_angle_[i] = 4095;
					
					total_angle_[i] += tmp_angle;
					if(total_angle_[i] > 4095)
						total_angle_[i] = 4095;
				}

				break;
			case 4:	// Lock stand, for check base
				total_speed_[i] = (database_[20-i] & 0x0000FFFF);
				total_angle_[i] = ((database_[20-i] & 0xFFFF0000)>> 16);

				break;
			}
		}

		/*if(stand_flag)
		{
			for(int i=0;i<12;i++)
			{
				standspeed[i] = Calculate_standspeed[i];
				standangle[i] = Calculate_standangle[i];
			}
		}*/
	}
}

/*void DataModule::stand_button_press()
{
	int state = 0;
	int count = 0;
	
	cout << "begin read stand" << endl;
	for(;;)
	{
		if(state == 0)
		{
			update_stand_flag_ = false;
			motion_execute_flag_ = false;
			cout << *(uint32_t *)init.p2h_stand_button_addr << endl;
			if(*(uint32_t *)init.p2h_stand_button_addr)
			{
				cout<<"aa"<<endl;
				state = 1;
				continue;
			}
			else
			{
				cout<<"bb"<<endl;
				break;
			}
			
		}
		else if(state == 1)
		{
			cout << "ready stand" << endl;
			update_stand_flag_ = true;
			motion_execute_flag_ = true;
			state = 0;
			stand_button_execute();
			break;
		}
	}
}*/

/*void DataModule::stand_button_execute()
{
	if(update_stand_flag_)
	{
		cout << "button stand" << endl;
		stand_flag = true;
		for(int i=0; i<21; i++)
		{
			total_speed_[i] = Walking_standspeed[i];
			total_angle_[i] = Walking_standangle[i];
		}
		
		for(int i=0; i<12; i++)
		{
			Calculate_standspeed[i] = standspeed[i];
			Calculate_standangle[i] = standangle[i];
		}
		cout<< "stand!!"<< endl;
	}
}*/

void DataModule::motion_execute()	//做motion動作
{
	//pushData();
	// saveData();
	// printf("motion exe\n");
	create_package(total_angle_, total_speed_, 30);

	//printf("motion_execute ");

	do_motion();
	// IK.initial_inverse_kinematic();

	data_module_cmd_ = 0;
	update_database_flag_ = false;
	motion_execute_flag_ = false;
}

void DataModule::set_stand()	//設定站姿參數
{
/*開機初始站姿*/
	int i=0;
	for(i=0; i<21; i++)
	{
		if(i==12||i==18)
		{
			total_speed_[i] = 80;
			base_speed_[i] = 80;
			standing_speed_[i] = 80;
		}			
		else
		{
			total_speed_[i] = 40;
			base_speed_[i] = 40;
			standing_speed_[i] = 40;
		}			
	}
	//-------------------------------------------------
	//---------------stand上半身初始值------------------
	total_angle_[0] = 3072;
	total_angle_[1] = 1898;
	total_angle_[2] = 2048;
	total_angle_[3] = 2048;
	total_angle_[4] = 1024;
	total_angle_[5] = 2198;
	total_angle_[6] = 2048;
	total_angle_[7] = 2048;
	total_angle_[8] = 2048;
	//---------------stand下半身初始值------------------
	total_angle_[9] = 2048;
	total_angle_[10] = 2028;
	total_angle_[11] = 1652;
	total_angle_[12] = 2737;
	total_angle_[13] = 2410;
	total_angle_[14] = 2047;
	total_angle_[15] = 2068;
	total_angle_[16] = 2058;
	total_angle_[17] = 2410;
	total_angle_[18] = 1337;
	total_angle_[19] = 1674;
	total_angle_[20] = 2047;

	for(i=0; i<21; i++)
	{
		standing_angle_[i] = total_angle_[i];
	}
	
	//----------------base初始值----------------------
	base_angle_[0] = 3072;
	base_angle_[1] = 2048;
	base_angle_[2] = 2048;
	base_angle_[3] = 2048;
	base_angle_[4] = 1024;
	base_angle_[5] = 2048;
	base_angle_[6] = 2048;
	base_angle_[7] = 2048;
	base_angle_[8] = 2048;
	base_angle_[9] = 2048;
	base_angle_[10] = 2048;
	base_angle_[11] = 2048;
	base_angle_[12] = 2048;
	base_angle_[13] = 2048;
	base_angle_[14] = 2048;
	base_angle_[15] = 2048;	
	base_angle_[16] = 2048;
	base_angle_[17] = 2048;
	base_angle_[18] = 2048;
	base_angle_[19] = 2048;
	base_angle_[20] = 2048;
	//-------------------------------------------------
	motion_execute();
}

void DataModule::create_package(int angle[21], int speed[21], int motion_delay)
{	
	int i=0;
	unsigned short blk_size = 0;

	for(i=0; i<21; i++)
	{
		*((uint32_t *)init.robot_motion_addr+(2*i+1)) = speed[i];
		*((uint32_t *)init.robot_motion_addr+(2*i)) = angle[i];
	}

	// calculate crc
	// Header
	packet_char_[0] = 0xFF;
	packet_char_[1] = 0xFF;
	packet_char_[2] = 0xFD;
	// Reserved
	packet_char_[3] = 0x00;
	// ID
	packet_char_[4] = 0xFE;
	// Length      The length after the Packet Length field (Instruction, Parameter, CRC fields). Packet Length = number of Parameters + 3
	packet_char_[5] = 0x2b;       //0x2b = 43(decimal)
	packet_char_[6] = 0;
	// Instruction
	packet_char_[7] = 0x83;       //write         // 0x83 = sync write
	// Parameter
	packet_char_[8] = 0x70;       // addressL             // Velocity: 0x70 = 112 Position: 0x74(hex) = 116(dec)
	packet_char_[9] = 0;

	packet_char_[10] = 0x08;      // data length(byte)
	packet_char_[11] = 0x00;

	//left hand
	for(i=0; i<4; i++)
	{
		packet_char_[12+i*9] = i+1;      //MotorID
		packet_char_[13+i*9] = speed[i] & 0xFF;    //Profile Velocity      //initial value = 0
		packet_char_[14+i*9] = (speed[i] >> 8) & 0xFF;
		packet_char_[15+i*9] = (speed[i] >> 16) & 0xFF;
		packet_char_[16+i*9] = (speed[i] >> 24) & 0xFF;
		packet_char_[17+i*9] = angle[i] & 0xFF;    // positionL
		packet_char_[18+i*9] = (angle[i] >> 8) & 0xFF;
		packet_char_[19+i*9] = (angle[i] >> 16) & 0xFF;
		packet_char_[20+i*9] = (angle[i] >> 24) & 0xFF;
	}

	blk_size = 5 + packet_char_[5];
	unsigned short lh_crc_value = update_crc(0, packet_char_, blk_size);

	//right hand
	for(i=0; i<4; i++)
	{
		packet_char_[12+i*9] = i+5;      //MotorID
		packet_char_[13+i*9] = speed[i+4] & 0xFF;    //Profile Velocity      //initial value = 0
		packet_char_[14+i*9] = (speed[i+4] >> 8) & 0xFF;
		packet_char_[15+i*9] = (speed[i+4] >> 16) & 0xFF;
		packet_char_[16+i*9] = (speed[i+4] >> 24) & 0xFF;
		packet_char_[17+i*9] = angle[i+4] & 0xFF;    // positionL
		packet_char_[18+i*9] = (angle[i+4] >> 8) & 0xFF;
		packet_char_[19+i*9] = (angle[i+4] >> 16) & 0xFF;
		packet_char_[20+i*9] = (angle[i+4] >> 24) & 0xFF;
	}

	blk_size = 5 + packet_char_[5];
	unsigned short rh_crc_value = update_crc(0, packet_char_, blk_size);

	//foot
	packet_char_[5] = 0x46; // 0x46 = 70  77-7

	packet_char_[12] = 0x09;
	packet_char_[13] = speed[8] & 0xFF;
	packet_char_[14] = (speed[8] >> 8) & 0xFF;
	packet_char_[15] = (speed[8] >> 16) & 0xFF;
	packet_char_[16] = (speed[8] >> 24) & 0xFF;
	packet_char_[17] = angle[8] & 0xFF;
	packet_char_[18] = (angle[8] >> 8) & 0xFF;
	packet_char_[19] = (angle[8] >> 16) & 0xFF;
	packet_char_[20] = (angle[8] >> 24) & 0xFF;

	//left foot
	for(i=0; i<6; i++)
	{
		packet_char_[21+i*9] = i+10;      //MotorID
		packet_char_[22+i*9] = speed[i+9] & 0xFF;    //Profile Velocity      //initial value = 0
		packet_char_[23+i*9] = (speed[i+9] >> 8) & 0xFF;
		packet_char_[24+i*9] = (speed[i+9] >> 16) & 0xFF;
		packet_char_[25+i*9] = (speed[i+9] >> 24) & 0xFF;
		packet_char_[26+i*9] = angle[i+9] & 0xFF;    // positionL
		packet_char_[27+i*9] = (angle[i+9] >> 8) & 0xFF;
		packet_char_[28+i*9] = (angle[i+9] >> 16) & 0xFF;
		packet_char_[29+i*9] = (angle[i+9] >> 24) & 0xFF;
	}

	blk_size = 5 + packet_char_[5];
	unsigned short lf_crc_value = update_crc(0, packet_char_, blk_size);

	//right foot
	for(i=0; i<6; i++)
	{
		packet_char_[21+i*9] = i+16;      //MotorID
		packet_char_[22+i*9] = speed[i+15] & 0xFF;    //Profile Velocity      //initial value = 0
		packet_char_[23+i*9] = (speed[i+15] >> 8) & 0xFF;
		packet_char_[24+i*9] = (speed[i+15] >> 16) & 0xFF;
		packet_char_[25+i*9] = (speed[i+15] >> 24) & 0xFF;
		packet_char_[26+i*9] = angle[i+15] & 0xFF;    // positionL
		packet_char_[27+i*9] = (angle[i+15] >> 8) & 0xFF;
		packet_char_[28+i*9] = (angle[i+15] >> 16) & 0xFF;
		packet_char_[29+i*9] = (angle[i+15] >> 24) & 0xFF;
	}

	blk_size = 5 + packet_char_[5];
	unsigned short rf_crc_value = update_crc(0, packet_char_, blk_size);

	*((uint32_t *)init.robot_motion_addr+(42)) = motion_delay;
	*((uint32_t *)init.robot_motion_addr+(43)) = 0x00000070;
	*((uint32_t *)init.robot_motion_addr+(44)) = (lh_crc_value << 16) + rh_crc_value;
	*((uint32_t *)init.robot_motion_addr+(45)) = (lf_crc_value << 16) + rf_crc_value;
}

void DataModule::do_motion()
{
	bool avalon_locus_idle = false;
	//printf("do_motion \n");
	for(;;)
	{
		if(avalon_locus_idle)
		{
			*(uint32_t *)(init.h2p_avalon_locus_addr_1) = 0x01;
			//usleep(100);
			*(uint32_t *)(init.h2p_avalon_locus_addr_1) = 0;
			break;
		}
		else
		{
			avalon_locus_idle = *(uint32_t *)(init.h2p_avalon_locus_addr_1);
		}
	}
}

string DataModule::DtoS(double value)
{
    string str;
    std::stringstream buf;
    buf << value;
    str = buf.str();
    return str;
}

void DataModule::save_data()
{
    char path[200] = "/data";
	std::string tmp = std::to_string(name_cont_);
	tmp = "/data_module"+tmp+".csv";
    strcat(path, tmp.c_str());

	
    fstream fp;
    fp.open(path, std::ios::out);
	std::string savedText;
    std::map<std::string, std::vector<double>>::iterator it_data_moudle;

	for(it_data_moudle = map_data_moudle.begin(); it_data_moudle != map_data_moudle.end(); it_data_moudle++)
	{
		savedText += it_data_moudle->first;
		if(it_data_moudle == --map_data_moudle.end())
		{
			savedText += "\n";
			fp<<savedText;
			savedText = "";
		}
		else
		{
			savedText += ",";
		}		
	}
	it_data_moudle = map_data_moudle.begin();
	int max_size = it_data_moudle->second.size();

	for(it_data_moudle = map_data_moudle.begin(); it_data_moudle != map_data_moudle.end(); it_data_moudle++)
	{
		if(max_size < it_data_moudle->second.size())
            max_size = it_data_moudle->second.size();
	}
	for(int i = 0; i < max_size; i++)
    {
        for(it_data_moudle = map_data_moudle.begin(); it_data_moudle != map_data_moudle.end(); it_data_moudle++)
        {
            if(i < it_data_moudle->second.size())
            {
                if(it_data_moudle == --map_data_moudle.end())
                {
                    savedText += std::to_string(it_data_moudle->second[i]) + "\n";
                    fp<<savedText;
                    savedText = "";
                }
                else
                {
                    savedText += std::to_string(it_data_moudle->second[i]) + ",";
                }
            }
            else
            {
                if(it_data_moudle == --map_data_moudle.end())
                {
                    savedText += "none\n";
                    fp<<savedText;
                    savedText = "";
                }
                else
                    savedText += "none,";
            }
        }
    }
    fp.close();
    for(it_data_moudle = map_data_moudle.begin(); it_data_moudle != map_data_moudle.end(); it_data_moudle++)
        it_data_moudle->second.clear();

    name_cont_++;

}

void DataModule::push_data()
{
	map_data_moudle.find("motor_01")->second.push_back((double)total_angle_[0]);
	map_data_moudle.find("motor_02")->second.push_back((double)total_angle_[1]);
	map_data_moudle.find("motor_03")->second.push_back((double)total_angle_[2]);
	map_data_moudle.find("motor_04")->second.push_back((double)total_angle_[3]);
	map_data_moudle.find("motor_05")->second.push_back((double)total_angle_[4]);
	map_data_moudle.find("motor_06")->second.push_back((double)total_angle_[5]);
	map_data_moudle.find("motor_07")->second.push_back((double)total_angle_[6]);
	map_data_moudle.find("motor_08")->second.push_back((double)total_angle_[7]);
	map_data_moudle.find("motor_09")->second.push_back((double)total_angle_[8]);
	map_data_moudle.find("motor_10")->second.push_back((double)total_angle_[9]);
	map_data_moudle.find("motor_11")->second.push_back((double)total_angle_[10]);
	map_data_moudle.find("motor_12")->second.push_back((double)total_angle_[11]);
	map_data_moudle.find("motor_13")->second.push_back((double)total_angle_[12]);
	map_data_moudle.find("motor_14")->second.push_back((double)total_angle_[13]);
	map_data_moudle.find("motor_15")->second.push_back((double)total_angle_[14]);
	map_data_moudle.find("motor_16")->second.push_back((double)total_angle_[15]);
	map_data_moudle.find("motor_17")->second.push_back((double)total_angle_[16]);
	map_data_moudle.find("motor_18")->second.push_back((double)total_angle_[17]);
	map_data_moudle.find("motor_19")->second.push_back((double)total_angle_[18]);
	map_data_moudle.find("motor_20")->second.push_back((double)total_angle_[19]);
	map_data_moudle.find("motor_21")->second.push_back((double)total_angle_[20]);
}

unsigned short DataModule::update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size)
{
	unsigned short i, j;
	unsigned short crc_table[256] = {
		0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
		0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
		0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
		0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
		0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
		0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
		0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
		0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
		0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
		0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
		0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
		0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
		0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
		0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
		0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
		0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
		0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
		0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
		0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
		0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
		0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
		0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
		0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
		0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
		0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
		0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
		0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
		0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
		0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
		0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
		0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
		0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
	};

	for(j = 0; j < data_blk_size; j++)
	{
		i = ((unsigned short)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
		crc_accum = (crc_accum << 8) ^ crc_table[i];
	}

	return crc_accum;
}
