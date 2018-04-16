/* 
 *  main.cpp 
 *  Author: Kaka Xie 
 *  Date:2017/11/30
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt8MultiArray.h"
//#include "geometry_msgs/Twist.h"
//#include "geometry_msgs/PoseStamped.h"
//#include "geometry_msgs/TwistStamped.h"
//#include "tf/transform_broadcaster.h"
#include <signal.h>
//#include <sstream>
//#include <math.h>
#include <stdio.h>
//#include <vector>
#include <iostream>
//#include <pthread.h>

#include <ultrasonic.h>
#include <laser.h>
#include <hall.h>
#include <common.h>
#include<stdlib.h>
uint16_t laser_test_data[LASER_NUM_MAX] = {0};
void sigintHandler(int sig)
{
    ROS_INFO("killing on exit");
    ros::shutdown();
}

#define ULTRASONIC_INIT_TIME_OUT        5.0     //unit: Second

#define MODE_TEST_DURATION_MAX          4000    //unit: Second
#define MODE_TEST_DURATION_MIN          1000     //unit: Second

/*

   ultrasonic work mode:

   ULTRASONIC_MODE_FORWARD     = 0,
   ULTRASONIC_MODE_BACKWARD    = 1,
   ULTRASONIC_MODE_TURNING     = 2,
 */


int main(int argc, char **argv)
{
    ros::init(argc, argv, "noah_sensors");
    bool is_log_on = 0;
    bool test_log_on = false;
    ROS_INFO("creating noah sensors node...");
    if(ros::param::has("noah_sensors_can_data_log_on"))
    {
        ros::param::get("/noah_sensors_can_data_log_on",is_log_on);
        ROS_INFO("can data log is %d",is_log_on);
    }
    else
    {
        is_log_on = false;
        ROS_INFO("can data log is off");
    }

    if(ros::param::has("test_log_on"))
    {
        ros::param::get("/test_log_on",test_log_on);
        ROS_INFO("test log is %d",test_log_on);
    }
    else
    {
        ROS_INFO("test log is off");
    }
    Ultrasonic *ultrasonic = new Ultrasonic(is_log_on); 
    Laser *laser = new Laser(is_log_on); 
    Hall *hall = new Hall(is_log_on); 

    if(ros::param::has("/noah_sensors/ultrasonic/max_range"))
    {
        ros::param::get("/noah_sensors/ultrasonic/max_range",ultrasonic->max_range);
        ROS_INFO("ultrasonic max range is %f",ultrasonic->max_range);
    }
    else
    {
        ultrasonic->max_range = 2.0;
        ROS_WARN("ultrasonic: default max range 2.00");
    }

    if(ros::param::has("/noah_sensors/ultrasonic/min_range"))
    {
        ros::param::get("/noah_sensors/ultrasonic/min_range",ultrasonic->min_range);
        ROS_INFO("ultrasonic max range is %f",ultrasonic->min_range);
    }
    else
    {
        ultrasonic->max_range = 0.03;
        ROS_WARN("ultrasonic: default min range 0.03");
    }

    uint32_t rate = 1000;
    ros::Rate loop_rate(rate);
    uint32_t cnt = 0;
    bool get_version_init = 0;
    static uint8_t pre_mode = ULTRASONIC_MODE_NONE;
    ros::Time mode_test_start_time = ros::Time::now();
    ros::Duration mode_test_duration(random()%(MODE_TEST_DURATION_MAX - MODE_TEST_DURATION_MIN) + MODE_TEST_DURATION_MIN);//random 100~1000 seconds

    ultrasonic->work_mode = ULTRASONIC_MODE_FORWARD;
    sleep(1.5);
    while(ros::ok())
    {
        if(get_version_init == 0)
        {
            static uint8_t cnt = 0;
            //for(uint8_t i = 0; i < ULTRASONIC_NUM_MAX; i++)
            if(cnt % (ULTRASONIC_NUM_MAX + LASER_NUM_MAX)< ULTRASONIC_NUM_MAX)
            {
                ultrasonic->get_version(cnt % (ULTRASONIC_NUM_MAX + LASER_NUM_MAX)); 
                usleep(1000 * 30);
                ROS_INFO("start to get ultrasonic %d version",cnt % (ULTRASONIC_NUM_MAX + LASER_NUM_MAX));
            }
            else if(cnt % (ULTRASONIC_NUM_MAX + LASER_NUM_MAX) < ULTRASONIC_NUM_MAX + LASER_NUM_MAX)
            {
                laser->get_version(cnt % (ULTRASONIC_NUM_MAX + LASER_NUM_MAX)- ULTRASONIC_NUM_MAX); 
                usleep(1000 * 30);
                ROS_INFO("start to get laser %d version",cnt % (ULTRASONIC_NUM_MAX + LASER_NUM_MAX)- ULTRASONIC_NUM_MAX);
            }
            cnt++;
            if(cnt >= 3*(ULTRASONIC_NUM_MAX + LASER_NUM_MAX))
            {
                get_version_init = 1; 
            }
        }

        /*---------------------  test code  --------------------------*/
        else if(ultrasonic->is_mode_init == 0)
        {
            ROS_WARN("start to init work mode ...");
            if(ultrasonic->work_mode == ULTRASONIC_MODE_TURNING)
            {
                if(pre_mode != ULTRASONIC_MODE_TURNING)
                {
                    pre_mode = ULTRASONIC_MODE_TURNING;
                    group_id_t group_id;
                    ultrasonic->current_work_mode_ul = 0;
                    for(uint8_t i=0; i<sizeof(ultrasonic->group_mode_turning) / sizeof(ultrasonic->group_mode_turning[0]); i++)
                    {
                        ultrasonic->turning_separate[i] = 0;
                        for(uint8_t j=0; j<sizeof(ultrasonic->group_mode_turning[0]) / sizeof(ultrasonic->group_mode_turning[0][0]); j++)
                        {
                            group_id.id = ultrasonic->group_mode_turning[i][j];
                            if(group_id.id < ULTRASONIC_NUM_MAX)
                            {
                                if(group_id.id < 32)
                                {
                                    ultrasonic->current_work_mode_ul |= 1<<group_id.id;
                                    ultrasonic->turning_separate[i] |= 1<<group_id.id;
                                }
                            }
                        }
                    }
                }
            }
            else if(ultrasonic->work_mode == ULTRASONIC_MODE_FORWARD)
            {
                if(pre_mode != ULTRASONIC_MODE_FORWARD)
                {
                    pre_mode = ULTRASONIC_MODE_FORWARD;
                    group_id_t group_id;
                    ultrasonic->current_work_mode_ul = 0;

                    for(uint8_t i=0; i<sizeof(ultrasonic->group_mode_forward) / sizeof(ultrasonic->group_mode_forward[0]); i++)
                    {
                        ultrasonic->forward_separate[i] = 0;
                        for(uint8_t j=0; j<sizeof(ultrasonic->group_mode_forward[0]) / sizeof(ultrasonic->group_mode_forward[0][0]); j++)
                        {
                            group_id.id = ultrasonic->group_mode_forward[i][j];
                            if(group_id.id < ULTRASONIC_NUM_MAX)
                            {
                                if(group_id.id < 32)
                                {
                                    ultrasonic->current_work_mode_ul |= 1<<group_id.id;
                                    ultrasonic->forward_separate[i] |= 1<<group_id.id;
                                }
                            }
                        }
                    }
                }
            }
            else if(ultrasonic->work_mode == ULTRASONIC_MODE_BACKWARD)
            {
                if(pre_mode != ULTRASONIC_MODE_BACKWARD)
                {
                    pre_mode = ULTRASONIC_MODE_BACKWARD;
                    group_id_t group_id;
                    ultrasonic->current_work_mode_ul = 0;

                    for(uint8_t i=0; i<sizeof(ultrasonic->group_mode_backward) / sizeof(ultrasonic->group_mode_backward[0]); i++)
                    {
                        ultrasonic->backward_separate[i] = 0;
                        for(uint8_t j=0; j<sizeof(ultrasonic->group_mode_backward[0]) / sizeof(ultrasonic->group_mode_backward[0][0]); j++)
                        {
                            group_id.id = ultrasonic->group_mode_backward[i][j];
                            if(group_id.id < ULTRASONIC_NUM_MAX)
                            {
                                if(group_id.id < 32)
                                {
                                    ultrasonic->current_work_mode_ul |= 1<<group_id.id;
                                    ultrasonic->backward_separate[i] |= 1<<group_id.id;
                                }
                            }
                        }
                    }
                }
            }


            std_msgs::UInt8MultiArray ack;

            ultrasonic->is_mode_init = 1;

            ack.data.push_back(0);
            ack.data.push_back(ultrasonic->work_mode);
            ultrasonic->work_mode_ack_pub.publish(ack);
            //ultrasonic->ack_work_mode(ultrasonic->work_mode);
            ros::param::set("/noah_sensors/ultrasonic_work_mode_ack",ultrasonic->work_mode);
        }

        /*---------------------  test code  --------------------------*/

        else if(ultrasonic->is_mode_init == 1)
        {

#if 1//ultrasonic
            if(cnt % (uint32_t)(rate / 11 ) == 0)
            {   
                static uint8_t ul_id = 0;
                static uint8_t group = 0;
                if(ultrasonic->work_mode == ULTRASONIC_MODE_FORWARD)
                {
                    ultrasonic->broadcast_measurement(ultrasonic->forward_separate[group] & sonar_en);
                    if(group >= sizeof(ultrasonic->group_mode_forward) / sizeof(ultrasonic->group_mode_forward[0]) - 1)
                    {
                        group = 0;
                    }
                    else
                    {
                        group++;
                    }
                }
                else if(ultrasonic->work_mode == ULTRASONIC_MODE_BACKWARD)
                {
                    ultrasonic->broadcast_measurement(ultrasonic->backward_separate[group] & sonar_en);
                    if(group >= sizeof(ultrasonic->group_mode_backward) / sizeof(ultrasonic->group_mode_backward[0]) - 1)
                    {
                        group = 0;
                    }
                    else
                    {
                        group++;
                    }
                }
                else if(ultrasonic->work_mode == ULTRASONIC_MODE_TURNING)
                {
                    ultrasonic->broadcast_measurement(ultrasonic->turning_separate[group] & sonar_en);
                    if(group >= sizeof(ultrasonic->group_mode_turning) / sizeof(ultrasonic->group_mode_turning[0]) - 1)
                    {
                        group = 0;
                    }
                    else
                    {
                        group++;
                    }
                }
                if(ul_id < ULTRASONIC_NUM_MAX)
                {
                    ul_id++;
                }
                else
                {
                    ul_id = 0;
                }

            }
            /*---------------------  test code  --------------------------*/

            if(cnt % (uint32_t)(rate / 10) == 0)
            {
                ultrasonic->update_status();
                ultrasonic->pub_ultrasonic_data_to_navigation(ultrasonic->distance);
            }

            if(cnt % (uint32_t)(rate / 20) == 0)
            {
                ultrasonic->update_measure_en(sonar_en);
            }

#endif
#if 0
            if(ros::Time::now() - mode_test_start_time >= ros::Duration(mode_test_duration)) 
            {
                ultrasonic->work_mode++;
                ultrasonic->is_mode_init = 0;
                if(ultrasonic->work_mode >= ULTRASONIC_MODE_MAX - 1)
                {
                    ultrasonic->work_mode = ULTRASONIC_MODE_FORWARD;
                }
                mode_test_start_time = ros::Time::now();
                ultrasonic->set_work_mode_start_time = ros::Time::now();
                mode_test_duration = ros::Duration(random()%(MODE_TEST_DURATION_MAX - MODE_TEST_DURATION_MIN) + MODE_TEST_DURATION_MIN);
                ROS_INFO("random time %f, next work mode is %d",mode_test_duration.toSec(),ultrasonic->work_mode);

            }
#endif

#if 1 //laser
            if(cnt % (uint32_t)(rate / 70) == 0)
            {   
                static uint32_t i = 0;
                if(laser_en & (1<<(i % LASER_NUM_MAX)))                    
                    laser->start_measurement(i % LASER_NUM_MAX);
                i++;
            }
            if(cnt % (uint32_t)(rate / 10) == 0)
            {
                laser->update_status();
                laser->pub_laser_data_to_navigation(laser->distance);
            }
#endif


            if(cnt % (uint32_t)(rate / 10) == 0)
            {
                ultrasonic->updata_work_mode();
                ultrasonic->updata_measure_range();
            }
            if(test_log_on)
            {
                if(cnt % (uint32_t)(rate / 5) == 0)
                {
                    //ROS_ERROR("hardware test log is on !!!");
                    ultrasonic->test_data.data.clear();
                    for(uint8_t i = 0; i < ULTRASONIC_NUM_MAX; i++)
                    {
                        ultrasonic->test_data.data.push_back((uint8_t)(ultrasonic->distance[i]*100));
                    }
                    for(uint8_t i = 0; i < LASER_NUM_MAX; i++)
                    {
                        ultrasonic->test_data.data.push_back((uint8_t)(laser_test_data[i]));
                    }
                    ultrasonic->test_data_pub.publish(ultrasonic->test_data);
                }
            }
            cnt++;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

}



