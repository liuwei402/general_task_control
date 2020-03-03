
#include <iostream>
#include <thread>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "std_msgs/String.h"

#include "general_task_control/motion.h"
#include "general_task_control/motion_controlAction.h"

#define MOTION_CONTROL_ACTION "motion_control"

using MotionControlActionServer = actionlib::SimpleActionServer<general_task_control::motion_controlAction>;
using MotionControlGoalConstPtr = general_task_control::motion_controlGoalConstPtr;
using MotionControlFeedback = general_task_control::motion_controlFeedback;

void execute(const MotionControlGoalConstPtr &goal,MotionControlActionServer *server)
{
    ROS_INFO("==========begin==========");
    MotionControlFeedback feedback;
    for(int i=0;i<10;i++)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        feedback.progress = i/10.0;
        server->publishFeedback(feedback);
        ROS_INFO("publish feedback.progress %f",feedback.progress);
    }
    server->setSucceeded();
    ROS_INFO("===========end===========");
}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"motion_control_server_test");
    ros::NodeHandle n;

    MotionControlActionServer server(n,MOTION_CONTROL_ACTION,boost::bind(&execute,_1,&server),false);
    server.start();

    ROS_INFO("===============motion control server test===================");
    ROS_INFO("motion control server started...");

    ros::spin();

    return 0;
}