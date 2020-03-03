#include <iostream>
#include <fstream>
#include <stdio.h>
#include <ctime>

#include <ros/ros.h>
#include "std_msgs/String.h"

using namespace std;

#define ROS_TOPIC_TASK_DATA "task_data"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "send_task_data");
    ros::NodeHandle n;
    
    system("pwd");
    

    ROS_INFO("===============send TaskData===================");

    ros::Publisher taskDataPublisher = n.advertise<std_msgs::String>(ROS_TOPIC_TASK_DATA, 1000);

    ifstream myfile("task_data.json");
    string content = "";
    string line = "";
    std_msgs::String taskData;
    if (myfile.is_open())
    {
        while (getline(myfile, line))
        {
            content += line;
            content += "\n";
            line = "";
        }
        myfile.close();
        cout << "task_data.json : " << endl;
        cout << content << endl;
        cout << "Task data loaded , press 'Enter' to send task data." << endl;

        taskData.data = content;
    }
    else
    {
        cout << "open task_data.json failed" << endl;
    }

    while (ros::ok())
    {
        getchar();
        cout << "send task data..." << endl;
        taskDataPublisher.publish(taskData);
    }

    return 0;
}