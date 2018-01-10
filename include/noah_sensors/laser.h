#include "ros/ros.h"
#include "std_msgs/String.h"
#include "json.hpp"
#include <mrobot_driver_msgs/vci_can.h>
#include <roscan/can_long_frame.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/PointCloud2.h>
#include <sonar_msgs/sonar_msgs.h>
//#include <range_sensor_layer/sonar_msgs.h>
using json = nlohmann::json;

#ifndef _LASER__H
#define _LASER__H

#define LASER_CAN_SRC_MAC_ID_BASE      0x70

#define LASER_CAN_SOURCE_ID_GET_VERSION		  0x01
#define LASER_CAN_SOURCE_ID_START_MEASUREMENT     0x80

#define LASER_NUM_MAX                 13 


#define LASER_DISTANCE_MAX                        1.20 
#define LASER_ERR_COMMUNICATE_TIME_OUT            1
#define LASER_DISTANCE_ERR_TIME_OUT               2.55 
#if 0
enum{
    INFRARED = 0,
    ULTRASOUND = 1,  
    HALL = 2,
};
#endif


class Laser 
{
    public:

        Laser(bool log_on = false)
        {
            is_log_on = log_on;

            pub_to_can_node = n.advertise<mrobot_driver_msgs::vci_can>("laser_to_can", 1000);
            sub_from_can_node = n.subscribe("can_to_micro_laser", 1000, &Laser::rcv_from_can_node_callback, this);
            pub_to_navigation = n.advertise<sensor_msgs::Range>("laser_msg",20);
laser_pub_to_navigation_all = n.advertise<sonar_msgs::sonar_msgs>("laser_msg_all",20);
	    version_ack_pub = n.advertise<std_msgs::String>("mcu_version_ack", 1000);
	    get_mcu_version_sub = n.subscribe("get_mcu_version", 10, &Laser::get_mcu_version_callback, this);
        }
        
        int start_measurement(uint8_t ul_id);
	void get_version(uint8_t ul_id);
        void rcv_from_can_node_callback(const mrobot_driver_msgs::vci_can::ConstPtr &c_msg);
        void update_status(void);
	void get_mcu_version_callback(const std_msgs::String data);
        void pub_laser_data_to_navigation(double *data);
        bool is_log_on;
        can_long_frame  long_frame;
        ros::Time start_measure_time[LASER_NUM_MAX];
        uint8_t err_status[LASER_NUM_MAX];


        double distance[LASER_NUM_MAX] = {0};
        sensor_msgs::Range laser_data;
        ros::Publisher lasercloud_pub;
        ros::Publisher laser_pub;
        ros::Publisher pub_to_navigation;
ros::Publisher laser_pub_to_navigation_all;
sonar_msgs::sonar_msgs laser_msgs;

    private:
        ros::NodeHandle n;
        ros::Subscriber laser_sub;
        ros::Subscriber sub_from_can_node;
	ros::Subscriber get_mcu_version_sub;

        ros::Publisher  pub_to_can_node;
	ros::Publisher  version_ack_pub;
	std::string version[LASER_NUM_MAX];

std::string laser_version_param[LASER_NUM_MAX] = {"mcu_laser_0_version","mcu_laser_1_version","mcu_laser_2_version","mcu_laser_3_version","mcu_laser_4_version","mcu_laser_5_version","mcu_laser_6_version","mcu_laser_7_version", "mcu_laser_8_version","mcu_laser_9_version","mcu_laser_10_version","mcu_laser_11_version","mcu_laser_12_version"};

std::string laser_num[LASER_NUM_MAX] = {"laser_0","laser_1","laser_2","laser_3","laser_4","laser_5","laser_6","laser_7", "laser_8","laser_9","laser_10","laser_11","laser_12"};


        std::string laser_frames[LASER_NUM_MAX] = {"laser_frame_0","laser_frame_1","laser_frame_2","laser_frame_3","laser_frame_4","laser_frame_5","laser_frame_6","laser_frame_7","laser_frame_8","laser_frame_9","laser_frame_10","laser_frame_11","laser_frame_12"};

std::string laser_frame_all = "laser_frame_all";
        //uint32_t laser_en = 0xffffffff;
        uint8_t laser_real_num = LASER_NUM_MAX;

	json j;
        void pub_json_msg(const nlohmann::json j_msg);
        bool is_laser_can_id(CAN_ID_UNION id);
        uint8_t parse_laser_id(CAN_ID_UNION id);
};


#endif
