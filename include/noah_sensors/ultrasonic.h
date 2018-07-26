#include "ros/ros.h"
#include "std_msgs/String.h"
#include "json.hpp"
#include <mrobot_driver_msgs/vci_can.h>
#include <sonar_msgs/sonar_msgs.h>
//#include <range_sensor_layer/sonar_msgs.h>
#include <roscan/can_long_frame.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/PointCloud2.h>
#include <common.h>

using json = nlohmann::json;

#ifndef _ULTRASONIC__H
#define _ULTRASONIC__H

#define ULTRASONIC_CAN_SRC_MAC_ID_BASE      0x60

#define CAN_SOURCE_ID_GET_VERSION           0x01

#define CAN_SOURCE_ID_START_MEASUREMENT     0x80
#define CAN_SOURCE_ID_MEASUREMENT_EN        0x81
#define CAN_SOURCE_ID_SET_GROUP             0x83

#define ULTRASONIC_NUM_MAX                  16 
#define FILTER_BUF_SIZE                     2 

#define GROUP_PERIOD                        100//ms

#define DISTANCE_MAX                        2.00
#define DISTANCE_MIN                        0.01
#define ERR_COMMUNICATE_TIME_OUT            1
#define DISTANCE_ERR_TIME_OUT               2.55 

enum
{
    
    ULTRASONIC_MODE_FORWARD     = 0,
    ULTRASONIC_MODE_BACKWARD    = 1,
    ULTRASONIC_MODE_TURNING     = 2,
    ULTRASONIC_MODE_STOP        = 3,
    ULTRASONIC_MODE_MAX,
    ULTRASONIC_MODE_NONE,
};

typedef struct
{
    uint8_t id;
    uint8_t group;
}group_id_t;

class Ultrasonic
{
    public:

        Ultrasonic(bool log_on = false)
        {
            is_log_on = log_on;
            //ultrasonic_pub = n.advertise<std_msgs::String>("ultrasonic_to_can",1000);
            sensor_en = n.subscribe("/map_server_mrobot/region_params_changer/sensor_params",1000,sensor_en_cb);
            pub_to_can_node = n.advertise<mrobot_driver_msgs::vci_can>("ultrasonic_to_can", 1000);
            version_ack_pub = n.advertise<std_msgs::String>("mcu_version_ack", 1000);

            sub_from_can_node = n.subscribe("can_to_ultrasonic", 1000, &Ultrasonic::rcv_from_can_node_callback, this);

            work_mode_sub = n.subscribe("ultrasonic_set_work_mode", 10, &Ultrasonic::work_mode_callback, this);

            get_mcu_version_sub = n.subscribe("get_mcu_version", 10, &Ultrasonic::get_mcu_version_callback, this);


            ultrasonic_pub_to_navigation = n.advertise<sensor_msgs::Range>("sonar_msg",20);
            ultrasonic_pub_to_navigation_all = n.advertise<sonar_msgs::sonar_msgs>("sonar_msg_all",20);
            work_mode_ack_pub = n.advertise<std_msgs::UInt8MultiArray>("ultrasonic_work_mode_ack",20);
            set_work_mode_start_time = ros::Time::now();


            test_data_pub = n.advertise<std_msgs::UInt8MultiArray>("sensors_test_data",20);//just for hardware test

            group_id_vec.clear();
            ultrasonic_msgs.sonars.resize(ULTRASONIC_NUM_MAX - 1);
        }
        
        int start_measurement(uint8_t ul_id);
        void get_version(uint8_t ul_id);
        void ultrasonic_en(uint8_t ul_id, bool en);
        int broadcast_measurement(uint32_t group);
        int set_group(uint8_t ul_id, uint8_t group);
        void rcv_from_can_node_callback(const mrobot_driver_msgs::vci_can::ConstPtr &c_msg);
        void work_mode_callback(const std_msgs::UInt8MultiArray data);
        void get_mcu_version_callback(const std_msgs::String data);
        void update_status(void);
        void pub_ultrasonic_data_to_navigation(double *data);
        void update_measure_en(uint32_t ul_en);
        void updata_work_mode(void);
        void updata_measure_range(void);
        int get_machine_version(void);

        void merge_all_min_distance_data(void);


        uint8_t is_mode_init = 0;
        bool is_log_on;
        can_long_frame  long_frame;
        ros::Time start_measure_time[ULTRASONIC_NUM_MAX];
        ros::Time set_work_mode_start_time;
        uint8_t err_status[ULTRASONIC_NUM_MAX];
        uint8_t online[ULTRASONIC_NUM_MAX];
        uint8_t work_mode = ULTRASONIC_MODE_FORWARD;
        uint32_t current_work_mode_ul = 0;

        json j;
        void pub_json_msg(const nlohmann::json j_msg);

        std_msgs::UInt8MultiArray test_data;
        ros::Publisher  test_data_pub;


        uint32_t forward_separate[2];
        int group_mode_forward[2][4] = 
                {
                    {10,11,0xff,0xff},
                    {0,1,12,13},
                    //{13},
                };

        uint32_t backward_separate[2];
        int group_mode_backward[2][2] = 
                {
                    {5,6},
                    {14,15},
                };

        uint32_t turning_separate[3];
        int group_mode_turning[3][6] = 
                {
                    {  10,  11,   5,   6,  14,  15},
                    {   0,   1,  12,  13,0xff,0xff},
                    {   2,   4,   3,   7,   8,   9}
                    //{   2,   4,   3,0xff,0xff,0xff},
                    //{   7,   8,   9,0xff,0xff,0xff}
                };

        vector<group_id_t> group_id_vec;
        double max_range = DISTANCE_MAX;
        double min_range = DISTANCE_MIN;
        double distance[ULTRASONIC_NUM_MAX] = {0};
        double distance_raw[ULTRASONIC_NUM_MAX] = {0};
        double distance_hw_test[ULTRASONIC_NUM_MAX] = {0};//for hardware test
        double distance_buf[ULTRASONIC_NUM_MAX][FILTER_BUF_SIZE] = {{0}};
        uint8_t distance_buf_proc[ULTRASONIC_NUM_MAX][FILTER_BUF_SIZE] = {{0}};

        uint8_t group[ULTRASONIC_NUM_MAX] = {0};

        std::string version[ULTRASONIC_NUM_MAX];
        sensor_msgs::Range ultrasonic_data;
        ros::Publisher ultrasonic_pub_to_navigation;
        ros::Publisher ultrasonic_pub_to_navigation_all;
        ros::Publisher work_mode_ack_pub;

        std::string machine_version;

        int param_get_test = 0;


#if 0
        uint8_t id_group[6][3] = 
        {
            //{10,  11,   3,   7 },
            //{0,   13,   5,   8 },
            //{1,   9,  4,  0xff},
            //{2,  6,  12,  0xff}

            {0,  3,   7 },
            {13,  9,  5},
            {11, 4,  0xff},
            {10,  2,  0xff},
            //{0xff,  2,  0xff},
            {1,  8,  0xff},
            {12,  6,  0xff}
		
        };
#else
        uint8_t id_group[14][1] = 
        {

            {0 },
            {1 },
            {2},
            {3},
            //{10,  2,  0xff},
            {4},
            {5},
            {6},
            {7},
            {8},
            //{10,  2,  0xff},
            {9},
            {10},
            {11},
            {12},
            {13}
        };
#endif

    private:
        ros::NodeHandle n;
        ros::Subscriber noah_ultrasonic_sub;
        ros::Subscriber sub_from_can_node;
        ros::Subscriber sensor_en;
        ros::Subscriber work_mode_sub;
        ros::Subscriber get_mcu_version_sub;



        ros::Publisher  ultrasonic_pub;
        ros::Publisher  pub_to_can_node;
        ros::Publisher  version_ack_pub;


        std::string ultrasonic_version_param[ULTRASONIC_NUM_MAX] = {"mcu_ultrasonic_0_version","mcu_ultrasonic_1_version","mcu_ultrasonic_2_version","mcu_ultrasonic_3_version","mcu_ultrasonic_4_version","mcu_ultrasonic_5_version","mcu_ultrasonic_6_version","mcu_ultrasonic_7_version", "mcu_ultrasonic_8_version","mcu_ultrasonic_9_version","mcu_ultrasonic_10_version","mcu_ultrasonic_11_version","mcu_ultrasonic_12_version","mcu_ultrasonic_13_version","mcu_ultrasonic_14_version","mcu_ultrasonic_15_version"};

        std::string ultrasonic_num[ULTRASONIC_NUM_MAX] = {"ultrasonic_0","ultrasonic_1","ultrasonic_2","ultrasonic_3","ultrasonic_4","ultrasonic_5","ultrasonic_6","ultrasonic_7", "ultrasonic_8","ultrasonic_9","ultrasonic_10","ultrasonic_11","ultrasonic_12","ultrasonic_13","ultrasonic_14","ultrasonic_15"};
        std::string ultrasonic_frames[ULTRASONIC_NUM_MAX] = {"sonar_frame_0","sonar_frame_1","sonar_frame_2","sonar_frame_3","sonar_frame_4","sonar_frame_5","sonar_frame_6","sonar_frame_7", "sonar_frame_8","sonar_frame_9","sonar_frame_10","sonar_frame_11","sonar_frame_12","sonar_frame_13","sonar_frame_14","sonar_frame_15"};

        std::string ultrasonic_frame_all = "sonar_frame_all";

        uint8_t ultrasonic_real_num = ULTRASONIC_NUM_MAX;

        uint32_t measure_en_ack = 0xffffffff;
        sonar_msgs::sonar_msgs ultrasonic_msgs;

        bool is_ultrasonic_can_id(CAN_ID_UNION id);
        bool is_ultrasonic_work_mode(int mode);
        uint8_t parse_ultrasonic_id(CAN_ID_UNION id);
        double merge_min_distance_data(double data1, double data2);

};


#endif
