/*
 *  ultrasonic.cpp
 *  Communicate with ultrasonics.
 *  Author: Kaka Xie
 *  Date:2017/11/28
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt8MultiArray.h"
#include "pthread.h"

#include <algorithm>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include <signal.h>
#include <iostream>
#include <vector>
#include "stdlib.h"
#include "cstdlib"
#include "string"
#include "sstream"
#include <mrobot_driver_msgs/vci_can.h>
#include <roscan/can_long_frame.h>
#include <ultrasonic.h>
#include <common.h>

void Ultrasonic::update_status(void)
{
    for(uint8_t i = 0; i < ultrasonic_real_num; i++)
    {
        if(ros::Time::now() - start_measure_time[i] >= ros::Duration(1.5))
        {
            this->err_status[i] = ERR_COMMUNICATE_TIME_OUT;
            this->distance[i] = DISTANCE_ERR_TIME_OUT;
            this->distance_raw[i] = DISTANCE_ERR_TIME_OUT;
            this->distance_hw_test[i] = DISTANCE_ERR_TIME_OUT;
        }
    }
}

void Ultrasonic::get_version(uint8_t ul_id)
{
    if(ul_id > 15)
    {
        return ;
    }
    mrobot_driver_msgs::vci_can can_msg;
    CAN_ID_UNION id;
    memset(&id, 0x0, sizeof(CAN_ID_UNION));
    id.CanID_Struct.SourceID = CAN_SOURCE_ID_GET_VERSION;
    id.CanID_Struct.SrcMACID = 1;
    id.CanID_Struct.DestMACID = ULTRASONIC_CAN_SRC_MAC_ID_BASE+ ul_id;
    id.CanID_Struct.FUNC_ID = 0x02;
    id.CanID_Struct.ACK = 0;
    id.CanID_Struct.res = 0;

    can_msg.ID = id.CANx_ID;
    can_msg.DataLen = 2;
    can_msg.Data.resize(2);
    can_msg.Data[0] = 0x00;
    can_msg.Data[1] = 0;
    this->pub_to_can_node.publish(can_msg);
}

int Ultrasonic::start_measurement(uint8_t ul_id)
{
    int error = 0;
    if(ul_id > 15)
    {
        return -1;
    }
    mrobot_driver_msgs::vci_can can_msg;
    CAN_ID_UNION id;
    memset(&id, 0x0, sizeof(CAN_ID_UNION));
    id.CanID_Struct.SourceID = CAN_SOURCE_ID_START_MEASUREMENT;
    id.CanID_Struct.SrcMACID = 1;
    id.CanID_Struct.DestMACID = ULTRASONIC_CAN_SRC_MAC_ID_BASE+ ul_id;
    id.CanID_Struct.FUNC_ID = 0x02;
    id.CanID_Struct.ACK = 0;
    id.CanID_Struct.res = 0;

    can_msg.ID = id.CANx_ID;
    can_msg.DataLen = 2;
    can_msg.Data.resize(2);
    can_msg.Data[0] = 0x00;
    can_msg.Data[1] = 0;
    this->pub_to_can_node.publish(can_msg);
    return error;
}

void Ultrasonic::ultrasonic_en(uint8_t ul_id, bool en)
{
    if(ul_id >= ultrasonic_real_num)
    {
        return ;
    }
    mrobot_driver_msgs::vci_can can_msg;
    CAN_ID_UNION id;
    memset(&id, 0x0, sizeof(CAN_ID_UNION));
    id.CanID_Struct.SourceID = CAN_SOURCE_ID_MEASUREMENT_EN;
    id.CanID_Struct.SrcMACID = 1;
    id.CanID_Struct.DestMACID = ULTRASONIC_CAN_SRC_MAC_ID_BASE+ ul_id;
    id.CanID_Struct.FUNC_ID = 0x02;
    id.CanID_Struct.ACK = 0;
    id.CanID_Struct.res = 0;

    can_msg.ID = id.CANx_ID;
    can_msg.DataLen = 2;
    can_msg.Data.resize(2);
    can_msg.Data[0] = 0x00;
    can_msg.Data[1] = en;
    this->pub_to_can_node.publish(can_msg);
}
#define BROADCAST_CAN_SRC_ID    0x60
int Ultrasonic::broadcast_measurement(uint32_t group)
{
    //ROS_ERROR("%s",__func__);
    int error = 0;
    mrobot_driver_msgs::vci_can can_msg;
    CAN_ID_UNION id;
    memset(&id, 0x0, sizeof(CAN_ID_UNION));
    id.CanID_Struct.SourceID = CAN_SOURCE_ID_START_MEASUREMENT;
    id.CanID_Struct.SrcMACID = 1;
    id.CanID_Struct.DestMACID = BROADCAST_CAN_SRC_ID;
    id.CanID_Struct.FUNC_ID = 0x02;
    id.CanID_Struct.ACK = 0;
    id.CanID_Struct.res = 0;

    can_msg.ID = id.CANx_ID;
    can_msg.DataLen = 5;
    can_msg.Data.resize(5);
    can_msg.Data[0] = 0x00;
    *(uint32_t*)&can_msg.Data[1] = group;
    //ROS_ERROR("group is %d",group);
    //ROS_ERROR("can_msg.Data[1] is %d",can_msg.Data[1]);
    //ROS_ERROR("can_msg.Data[2] is %d",can_msg.Data[2]);
    //ROS_ERROR("can_msg.Data[3] is %d",can_msg.Data[3]);
    //ROS_ERROR("can_msg.Data[4] is %d",can_msg.Data[4]);
    this->pub_to_can_node.publish(can_msg);
    return error;
}

int Ultrasonic::set_group(uint8_t ul_id, uint8_t group)
{
    int error = 0;
    if(ul_id >= ultrasonic_real_num)
    {
        ROS_ERROR("ul_id is not right, set group failed ! !");
        return -1;
    }
    mrobot_driver_msgs::vci_can can_msg;
    CAN_ID_UNION id;
    memset(&id, 0x0, sizeof(CAN_ID_UNION));
    id.CanID_Struct.SourceID = CAN_SOURCE_ID_SET_GROUP;
    id.CanID_Struct.SrcMACID = 1;
    id.CanID_Struct.DestMACID = ULTRASONIC_CAN_SRC_MAC_ID_BASE+ ul_id;
    id.CanID_Struct.FUNC_ID = 0x02;
    id.CanID_Struct.ACK = 0;
    id.CanID_Struct.res = 0;

    can_msg.ID = id.CANx_ID;
    can_msg.DataLen = 2;
    can_msg.Data.resize(2);
    can_msg.Data[0] = 0x00;
    can_msg.Data[1] = group;
    this->pub_to_can_node.publish(can_msg);
    ROS_ERROR("set ultrasonic %d to group %d",ul_id, group);
    return error;
}


void Ultrasonic::update_measure_en(uint32_t ul_en)
{
    if(ul_en<<(32 - ultrasonic_real_num) == measure_en_ack<<(32 - ultrasonic_real_num))
        return;
    if(ros::Time::now() - sensor_en_start_time < ros::Duration(4))
    {
        for(uint8_t i = 0; i < ultrasonic_real_num; i++)
        {
            if( (ul_en^measure_en_ack) &(1<<i) )
            {
                this->ultrasonic_en(i,(ul_en>>i) &0x01);
                //ROS_ERROR("set %d to %d",i, (ul_en>>i) &0x01);
            }
        }
    }

}

bool Ultrasonic::is_ultrasonic_work_mode(int mode)
{
    if((mode >= ULTRASONIC_MODE_FORWARD) && (mode < ULTRASONIC_MODE_MAX))
    {
        return true;
    }
    return false;
}

void Ultrasonic::updata_work_mode(void)
{
    int get_work_mode = 0;
    n.getParam("/noah_sensors/ultrasonic_work_mode",get_work_mode);
    if(is_ultrasonic_work_mode(get_work_mode) == true)
    {
        if(this->work_mode != get_work_mode)
        {
            ROS_WARN("change work mode from %d to %d",this->work_mode,get_work_mode);
            this->work_mode = get_work_mode;
            this->is_mode_init = 0;
        }
    }
}

void Ultrasonic::updata_measure_range(void)
{
    static double max_range_tmp = 0;
    static double min_range_tmp = 0;
    n.getParam("/noah_sensors/ultrasonic/max_range",this->max_range);
    n.getParam("/noah_sensors/ultrasonic/min_range",this->min_range);
    if(max_range > min_range + 0.0001)
    {
        if((abs(max_range_tmp - max_range) > 0.0001) || (abs(min_range_tmp - min_range) > 0.0001))
        {
            ROS_WARN("change max_range from %f to %f",max_range_tmp,this->max_range);
            ROS_WARN("change min_range from %f to %f",min_range_tmp,this->min_range);
        }
        max_range_tmp = this->max_range;
        min_range_tmp = this->min_range;
    }
}

int Ultrasonic::get_machine_version(void)
{
    if(ros::param::has("/noah_machine_version"))
    {
        ros::param::get("/noah_machine_version",machine_version);
        ROS_INFO("get noah machine version: %s",machine_version.data());
    }
    else
        return -1;

    //ROS_INFO("compare result: %d",ultrasonic->machine_version.compare( "EVT-5-1"));
    if(machine_version.compare( "EVT-5-1") < 0)
    {
        ROS_INFO("14 ultrasonic");
        ultrasonic_real_num = 14;
        int group_mode_forward_tmp[2][4] =
        {
            {10,11,0xff,0xff},
            {0,1,12,13},
        };

        int group_mode_backward_tmp[2][2] =
        {
            {5,6},
            {0xff,0xff},
        };

        int group_mode_turning_tmp[3][6] =
        {
            {  10,  11,   5,   6,0xff,0xff},
            {   0,   1,  12,  13,0xff,0xff},
            {   2,   4,   3,   7,   8,   9}
        };

        memcpy(group_mode_forward,group_mode_forward_tmp, sizeof(group_mode_forward_tmp));
        memcpy(group_mode_backward,group_mode_backward_tmp, sizeof(group_mode_backward_tmp));
        memcpy(group_mode_turning,group_mode_turning_tmp, sizeof(group_mode_turning_tmp));

        std::cout<<"forward group"<<endl;
        for(int i = 0; i < sizeof(group_mode_forward_tmp)/sizeof(group_mode_forward_tmp[0]); i++)
        {
            for(int j = 0; j < sizeof(group_mode_forward_tmp[0])/sizeof(group_mode_forward_tmp[0][0]); j++)
            {
                std::cout<<group_mode_forward[i][j]<<",";
            }
            std::cout<<std::endl;
        }

        std::cout<<"backward group"<<endl;
        for(int i = 0; i < sizeof(group_mode_backward_tmp)/sizeof(group_mode_backward_tmp[0]); i++)
        {
            for(int j = 0; j < sizeof(group_mode_backward_tmp[0])/sizeof(group_mode_backward_tmp[0][0]); j++)
            {
                std::cout<<group_mode_backward[i][j]<<",";
            }
            std::cout<<std::endl;
        }

        std::cout<<"turning group"<<endl;
        for(int i = 0; i < sizeof(group_mode_turning_tmp)/sizeof(group_mode_turning_tmp[0]); i++)
        {
            for(int j = 0; j < sizeof(group_mode_turning_tmp[0])/sizeof(group_mode_turning_tmp[0][0]); j++)
            {
                std::cout<<group_mode_turning[i][j]<<",";
            }
            std::cout<<std::endl;
        }
    }
    else
    {
        ROS_INFO("16 ultrasonic");
        ultrasonic_real_num = 16;
        int group_mode_forward_tmp[2][4] =
        {
            {10,11,0xff,0xff},
            {0,1,12,13},
        };

        int group_mode_backward_tmp[2][2] =
        {
            {5,6},
            {14,15},
        };

        int group_mode_turning_tmp[3][6] =
        {
            {  10,  11,   5,   6,14,15},
            {   0,   1,  12,  13,0xff,0xff},
            {   2,   4,   3,   7,   8,   9}
        };

        memcpy(group_mode_forward,group_mode_forward_tmp, sizeof(group_mode_forward_tmp));
        memcpy(group_mode_backward,group_mode_backward_tmp, sizeof(group_mode_backward_tmp));
        memcpy(group_mode_turning,group_mode_turning_tmp, sizeof(group_mode_turning_tmp));

        std::cout<<"forward group"<<endl;
        for(int i = 0; i < sizeof(group_mode_forward_tmp)/sizeof(group_mode_forward_tmp[0]); i++)
        {
            for(int j = 0; j < sizeof(group_mode_forward_tmp[0])/sizeof(group_mode_forward_tmp[0][0]); j++)
            {
                std::cout<<group_mode_forward[i][j]<<",";
            }
            std::cout<<std::endl;
        }

        std::cout<<"backward group"<<endl;
        for(int i = 0; i < sizeof(group_mode_backward_tmp)/sizeof(group_mode_backward_tmp[0]); i++)
        {
            for(int j = 0; j < sizeof(group_mode_backward_tmp[0])/sizeof(group_mode_backward_tmp[0][0]); j++)
            {
                std::cout<<group_mode_backward[i][j]<<",";
            }
            std::cout<<std::endl;
        }

        std::cout<<"turning group"<<endl;
        for(int i = 0; i < sizeof(group_mode_turning_tmp)/sizeof(group_mode_turning_tmp[0]); i++)
        {
            for(int j = 0; j < sizeof(group_mode_turning_tmp[0])/sizeof(group_mode_turning_tmp[0][0]); j++)
            {
                std::cout<<group_mode_turning[i][j]<<",";
            }
            std::cout<<std::endl;
        }
    }
    return 0;
}


void Ultrasonic::pub_json_msg( const nlohmann::json j_msg)
{
    std_msgs::String pub_json_msg;
    std::stringstream ss;

    ss.clear();
    ss << j_msg;
    pub_json_msg.data = ss.str();
    this->version_ack_pub.publish(pub_json_msg);
}


void Ultrasonic::get_mcu_version_callback(const std_msgs::String data)
{
    json j;
    j.clear();

    for(uint8_t i = 0; i < ultrasonic_real_num; i++)
    {
        j =
        {
            {"version_ack","sensors"},
            {
                "data",
                {
                    {this->ultrasonic_num[i], this->version[i]},
                },
            }
        };
        this->pub_json_msg(j);
    }
}


bool Ultrasonic::is_ultrasonic_can_id(CAN_ID_UNION id)
{
    if((id.CanID_Struct.SrcMACID >= 0x60)&&(id.CanID_Struct.SrcMACID <= 0x6f))
    {
        return true ;
    }
    return false;
}


#define NOT_ULTRASONIC_ID      0xff
uint8_t Ultrasonic::parse_ultrasonic_id(CAN_ID_UNION id)
{
    if((id.CanID_Struct.SrcMACID >= 0x60)&&(id.CanID_Struct.SrcMACID <= 0x6f))
    {
        return id.CanID_Struct.SrcMACID - ULTRASONIC_CAN_SRC_MAC_ID_BASE;
    }
    return NOT_ULTRASONIC_ID;
}


double Ultrasonic::merge_min_distance_data(double data1, double data2)
{
    double min_data = 0;
    if( (abs(data1) >= 0.000001)  &&  (abs(data2) >= 0.000001) )
    {
        min_data = min(data1, data2);
    }
    else
    {
        if(abs(data1 - DISTANCE_ERR_TIME_OUT) <= 0.00001)
        {
            min_data = data2;
        }
        else if(abs(data2 - DISTANCE_ERR_TIME_OUT) <= 0.00001)
        {
            min_data = data1;
        }
        else
        {
            min_data = max(data1, data2);
        }
    }
    return min_data;
}
void Ultrasonic::merge_all_min_distance_data(void)
{
    if(16 == ultrasonic_real_num )
    {
        this->distance[5] = this->merge_min_distance_data(this->distance_raw[5], this->distance_raw[14]);
        this->distance[6] = this->merge_min_distance_data(this->distance_raw[6], this->distance_raw[15]);
        this->distance[12] = this->merge_min_distance_data(this->distance_raw[12], this->distance_raw[13]);
    }
    if(14 == ultrasonic_real_num )
    {
        this->distance[12] = this->merge_min_distance_data(this->distance_raw[12], this->distance_raw[13]);
    }
}

void Ultrasonic::rcv_from_can_node_callback(const mrobot_driver_msgs::vci_can::ConstPtr &c_msg)
{
    mrobot_driver_msgs::vci_can can_msg;
    mrobot_driver_msgs::vci_can long_msg;
    CAN_ID_UNION id;
    uint8_t ul_id;
    //ROS_INFO("%s",__func__);
    long_msg = this->long_frame.frame_construct(c_msg);
    mrobot_driver_msgs::vci_can* msg = &long_msg;
    if( msg->ID == 0 )
    {
        return;
    }
    if(this->is_log_on == true)
    {
        for(uint8_t i = 0; i < msg->DataLen; i++)
        {
            ROS_INFO("msg->Data[%d] = 0x%x",i,msg->Data[i]);
        }
    }
    can_msg.ID = msg->ID;
    id.CANx_ID = can_msg.ID;
    can_msg.DataLen = msg->DataLen;

    if(this->is_ultrasonic_can_id(id) == false)
    {
        ROS_ERROR("not ultrasonic CAN id");
        return ;
    }

    if((ul_id = this->parse_ultrasonic_id(id)) == NOT_ULTRASONIC_ID)
    {
        ROS_ERROR("ultrasonic CAN id not right");
        return ;
    }
    if(ul_id >= ultrasonic_real_num)
    {
        ROS_ERROR("wtf ! ! !");
        return;
    }

    this->online[ul_id] = 1;

    if(id.CanID_Struct.SourceID == CAN_SOURCE_ID_START_MEASUREMENT)
    {
        if(id.CanID_Struct.ACK == 1)
        {
            uint16_t distance_tmp = 0;
            static uint32_t cnt = 0;
            this->start_measure_time[ul_id] = ros::Time::now();

            distance_tmp = msg->Data[0];
            distance_tmp += msg->Data[1]<<8;

            if((distance_tmp < 3) && (distance_tmp > 0))
            {
                distance_tmp = 3;
            }

            this->distance_raw[ul_id] = double(distance_tmp)/100;
            if((this->distance_raw[ul_id] >= this->max_range - 0.00001) || (abs(this->distance_raw[ul_id]) <= 0.00001))  //distance > DISTANCE_MAX or do not have obstacle
            {
                this->distance_raw[ul_id] = this->max_range;
            }
            if(this->distance_raw[ul_id] <= this->min_range + 0.00001)
            {
                this->distance_raw[ul_id] = this->min_range;
            }

            this->distance[ul_id] = this->distance_raw[ul_id];
            this->distance_hw_test[ul_id] = this->distance_raw[ul_id];
            measure_en_ack |= 1<<ul_id; //we can receive measurement data, so this ultrasonic is enable !
        }
    }

    if(id.CanID_Struct.SourceID == CAN_SOURCE_ID_MEASUREMENT_EN)
    {
        if(id.CanID_Struct.ACK == 1)
        {
            if(msg->Data[0] == 0)
            {
                measure_en_ack &= ~(1<<ul_id);
            }
            else if(msg->Data[0] == 1)
            {
                measure_en_ack |= 1<<ul_id;
            }
            ROS_WARN("get ultrasonic id %d enable is %d ",ul_id,msg->Data[0]);
            ROS_WARN("measure_en_ack: %x ",measure_en_ack);
            ROS_WARN("measure_en: %x ",sonar_en);

        }
    }

    if(id.CanID_Struct.SourceID == CAN_SOURCE_ID_SET_GROUP)
    {
        if(id.CanID_Struct.ACK == 1)
        {
            if(msg->Data[0] > 0)
            {
                group_id_t group_id;
                group_id.id = ul_id;
                group_id.group = msg->Data[0];
                if(group_id_vec.size() > 0)
                {
                    for(vector<group_id_t>::iterator it = group_id_vec.begin(); it != group_id_vec.end(); it++)
                    {
                        if( ( (*it).group == group_id.group) && ( (*it).id == group_id.id))
                        {
                            group_id_vec.erase(it);
                            break;
                        }
                    }
                }
                ROS_WARN("ultrasonic %d group is %d",ul_id,group[ul_id]);
            }

        }
    }

    if(id.CanID_Struct.SourceID == CAN_SOURCE_ID_GET_VERSION)
    {
        uint8_t len;
        if(id.CanID_Struct.ACK == 1)
        {
            len = msg->Data[0];
            version[ul_id].resize(len);
            version[ul_id].clear();
            for(uint8_t i = 0; i < len; i++)
            {
                version[ul_id].push_back(*(char *)&(msg->Data[i+1]));
            }
            //memcpy(version[ul_id].cbegin(),&msg->Data[1], len);
            n.setParam(ultrasonic_version_param[ul_id],version[ul_id]);
            ROS_WARN("ultrasonic %d version is %s",ul_id,version[ul_id].data());

        }
    }
}


void Ultrasonic::work_mode_callback(const std_msgs::UInt8MultiArray set_mode)
{
    if(set_mode.data.size() == 1)
    {
        if((set_mode.data[0] < ULTRASONIC_MODE_MAX) && (set_mode.data[0] >= 0))
        {
            this->work_mode = set_mode.data[0];
            this->is_mode_init = 0;
            this->set_work_mode_start_time = ros::Time::now();
        }
    }
}


void Ultrasonic::pub_ultrasonic_data_to_navigation(double * ul_data)
{
#define NAVIGATION_ULTRASONIC_NUM   13
    uint32_t en_sonar = sonar_en;
    static bool close_all_flag = 0;
    this->ultrasonic_msgs.header.stamp = ros::Time::now();
    this->ultrasonic_msgs.header.frame_id = ultrasonic_frame_all;

    this->ultrasonic_data.header.stamp = ros::Time::now();
    this->ultrasonic_data.radiation_type = ULTRASOUND;
    this->ultrasonic_data.field_of_view = 0.61;
    this->ultrasonic_data.min_range = this->min_range;
    this->ultrasonic_data.max_range = this->max_range;

    n.getParam("ultrasonic_test",param_get_test);
#if 1

    this->ultrasonic_msgs.sonars.resize(NAVIGATION_ULTRASONIC_NUM );
    this->merge_all_min_distance_data();
    for(int i=0;i<NAVIGATION_ULTRASONIC_NUM ;i++)
    {
        this->ultrasonic_data.header.frame_id = this->ultrasonic_frames[i];
        this->ultrasonic_data.range = this->distance[i];
        this->ultrasonic_msgs.sonars[i] = ultrasonic_data;
    }
    this->ultrasonic_pub_to_navigation_all.publish(this->ultrasonic_msgs);

    if(close_all_flag == 0)
    {
        for(int i=0;i<NAVIGATION_ULTRASONIC_NUM ;i++)
        {
            if(en_sonar == 0)
            {
                close_all_flag = 1;

                if(i >= 3)
                {
                    this->ultrasonic_data.min_range = this->min_range;
                    this->ultrasonic_data.max_range = this->max_range;
                }
                this->ultrasonic_data.header.frame_id = this->ultrasonic_frames[i];
                this->ultrasonic_data.range = 5.0;
                usleep(2000);
                this->ultrasonic_pub_to_navigation.publish(this->ultrasonic_data);
            }
            else if( (en_sonar & (0x00000001<<i)) /*&& (current_work_mode_ul & (0x000000001 << i))*/ )
            {
                close_all_flag = 0;

                if(i >= 3)
                {
                    this->ultrasonic_data.min_range = this->min_range;
                    this->ultrasonic_data.max_range = this->max_range;
                }
                this->ultrasonic_data.header.frame_id = this->ultrasonic_frames[i];
                this->ultrasonic_data.range = this->distance[i];
                usleep(2000);
                this->ultrasonic_pub_to_navigation.publish(this->ultrasonic_data);
            }
        }
    }

    if(en_sonar == 0)
    {
        close_all_flag = 1;
    }
    else
    {
        close_all_flag = 0;
    }

#endif

}


