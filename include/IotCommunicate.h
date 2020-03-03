
#ifndef _IOTCOMMUNICATE_H_
#define _IOTCOMMUNICATE_H_

#include <ros/ros.h>
#include "std_msgs/String.h"

#include <iostream>
#include <queue>
#include <mutex>

#include "general_task_control/device_info_basic.h"
#include "general_task_control/device_info_task.h"
#include "general_task_control/device_info_exception.h"

namespace GeneralTaskControl
{

struct Task
{
    double      x;
    double      y;
    double      z;
    double      angle;
    bool        workState;
    std::string workData;
    
    inline void clear()
    {
        x = 0.0;
        y = 0.0;
        z = 0.0;
        angle = 0.0;
        workState = false;
        workData = "";
    }
};

using TaskData = std::queue<Task>;
using TaskDataQueue = std::queue<TaskData>;

class IotCommunicate
{
public:
    IotCommunicate(ros::NodeHandle *_nh);
    ~IotCommunicate();

    /* callback: topic "task_data" */
    void taskDataCallBack(const std_msgs::String &taskData);

    /* device info basic publish */
    void basicPublish(const general_task_control::device_info_basic &basic);

    /* device info task publish */
    void taskPublish(const general_task_control::device_info_task &task);

    /* device info exception publish */
    void exceptionPublish(const general_task_control::device_info_exception &exception);

    /* get taskDataQueue size */
    int taskDataQueueSize();

    /* get taskDataQueue front */
    TaskData taskDataQueueFront();

    /* taskDataQueue pop */
    void taskDataQueuePop();

private:
    void init();
    void release();
    /* parse task data && persistent to taskDataQueue */
    void parseTaskData(const std_msgs::String &taskData);

private:
    ros::NodeHandle *nh;
    ros::Subscriber taskDataSubcriber;
    ros::Publisher basicPublisher;
    ros::Publisher taskPublisher;
    ros::Publisher exceptionPublisher;
    
    int lastTaskId;
    TaskDataQueue taskDataQueue;
    std::mutex taskDataQueueMutex;
};

} /* namespace GeneralTaskControl */

#endif