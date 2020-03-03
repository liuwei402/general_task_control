
#ifndef _CURRENT_POSITION_H_
#define _CURRENT_POSITION_H_

#include <ros/ros.h>
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"

#include <iostream>
#include <mutex>

namespace GeneralTaskControl
{

using Position = geometry_msgs::PoseStamped;
using PositionConstPtr = geometry_msgs::PoseStamped::ConstPtr;

class CurrentPosition
{
public:
    CurrentPosition(ros::NodeHandle *_nh);
    ~CurrentPosition();

    Position getPosition();

    /* current position callback */
    void currentPositionCallBack(const PositionConstPtr &_position);

private:
    void init();
    void release();

private:
    ros::NodeHandle *nh;

    ros::Subscriber currentPostionSubscriber;

    Position position;
    std::mutex positionMutex;
};

} /* namespace GeneralTaskControl */

#endif