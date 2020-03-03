
#include <iostream>
#include <thread>

/* ros framework headers */
#include <ros/ros.h>
#include "std_msgs/String.h"

/* authors headers */
#include "general_task_control/Global.h"
#include "general_task_control/IotCommunicate.h"
#include "general_task_control/TaskControl.h"

using namespace GeneralTaskControl;

static TaskControl *taskControl = nullptr;

void taskControlThread()
{
    if(taskControl != nullptr)
    {
        taskControl->exec();
    }
}

int main(int argc,char** argv)
{
    ros::init(argc, argv, "general_task_control");
    ros::NodeHandle nh;

    ROS_INFO("================General task control===================");

    IotCommunicate iotCommunicate(&nh);
    taskControl = new TaskControl(&nh,&iotCommunicate);
    
    std::thread taskControlThead(taskControlThread);
    
    ros::spin();

    taskControlThead.join();
    delete taskControl;
    taskControl = nullptr;

    return 0;
}