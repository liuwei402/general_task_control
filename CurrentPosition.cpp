
#include "general_task_control/CurrentPosition.h"

namespace GeneralTaskControl
{

#define ROS_SUB_TOPIC_CURRENT_POSITION "current_pose"

CurrentPosition::CurrentPosition(ros::NodeHandle *_nh)
    : nh(_nh)
{
    init();
}

CurrentPosition::~CurrentPosition()
{
    release();
}

Position CurrentPosition::getPosition()
{
    Position tmp;

    positionMutex.lock();
    tmp = position;
    positionMutex.unlock();

    return tmp;
}

void CurrentPosition::currentPositionCallBack(const PositionConstPtr &_position)
{
    positionMutex.lock();
    position = *_position;
    positionMutex.unlock();
}

void CurrentPosition::init()
{
    currentPostionSubscriber = nh->subscribe(ROS_SUB_TOPIC_CURRENT_POSITION, 1000,
                                             &CurrentPosition::currentPositionCallBack, this);
}

void CurrentPosition::release()
{
}

} /* namespace GeneralTaskControl */